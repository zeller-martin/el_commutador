from serial import Serial
import threading
import numpy as np
import os
import time
import tkinter as tk
from tkinter import filedialog
from scipy.spatial.transform import Rotation

# --- Stepper Motor Controller Class ---
class StepperController: 
    
    def __init__(self, port, sense=1, baud_rate=115200, microstep=True):
        self.lock = threading.Lock()
        self._connect(port, baud_rate)
        self.reset()
        self._deactivate_microstep()
        self.steps_per_turn = 200
        self.microstep = microstep

        if self.microstep:
            self._activate_microstep()

        self.position = 0
        self._position_motor = 0
        self.set_step_time(312)  # microseconds
        self.sense = sense  # Direction sense of rotation
    
    def _connect(self, port, baud_rate):
        """Establish serial connection to stepper driver."""
        self.port = port
        self.baud_rate = baud_rate
        print(f'Attempting to connect to {port} ...')
        self.ser = Serial(port, baud_rate)
        print(f'Successfully connected to {port}.')

    def _write(self, string):
        """Send command to stepper motor."""
        self.ser.write(string.encode('utf-8'))

    def reset(self):
        """Reset motor and internal position."""
        self._write('R')
        self.position = 0

    def stop(self):
        """Stop motor movement."""
        self._write('S')

    def resume(self):
        """Resume motor movement."""
        self._write('G')

    def pos_reset(self):
        """Reset motor position with current settings."""
        self._write('R')
        old_step_time = 1 * self.step_time
        if self.microstep:
            self._activate_microstep()
        self.position = 0
        self.set_step_time(old_step_time)

    def set_position(self, pos):
        """Set motor to a new target position."""
        with self.lock:
            self.position = pos
            self._position_motor = int(self.steps_per_turn * self.position / (2 * np.pi))
            self._write(f'P{int(self.sense * self._position_motor)}X')

    def set_step_time(self, t):
        """Set time per motor step in microseconds."""
        with self.lock:
            self._write(f'T{t}X')
            self.step_time = t

    def _activate_microstep(self):
        """Enable microstepping (finer resolution)."""
        with self.lock:
            self.microstep_active = True
            self.steps_per_turn = 3200
            self._write('M')

    def _deactivate_microstep(self):
        """Disable microstepping."""
        with self.lock:
            self.microstep_active = False
            self.steps_per_turn = 200
            self._write('N')

    def query_position(self):
        """Query current motor position."""
        with self.lock:
            self._write('Q')
            return self.sense * 2 * np.pi * int.from_bytes(self.ser.read(4), 'little', signed=True) / self.steps_per_turn


# --- CSV Source Class (live reading from a .csv file) ---
class CSV_source:

    def __init__(self, filename):
        self.filename = filename
        self.position = 0
        self.csv_file = open(self.filename, 'r')
        self._csv_reader = threading.Thread(target=self._read_orientation_csv,
                                            name='csv_reader',
                                            daemon=True)
        self._csv_reader.start()

    def _read_orientation_csv(self):
        """Continuously reads orientation data from a .csv file and updates position."""

        def get_single_row():
            """Read and process a single row from the csv."""
            buffer = ''
            while (newchar := self.csv_file.read(1)) != '\n':
                if len(newchar):
                    buffer += newchar
            row = np.array([float(num) for num in buffer.split(',')[1:]])
            rot_euler = Rotation.from_quat(row).as_euler('xyz')
            return rot_euler[0]  # yaw

        self.csv_file.seek(0, 2)  # move to end of file
        while (newchar := self.csv_file.read(1)) != '\n':
            pass

        last_read = get_single_row()

        while True:
            yaw = get_single_row()
            diff = (yaw - last_read + np.pi) % (2 * np.pi) - np.pi
            last_read = 1 * yaw
            self.position += diff


# --- Dummy Source for Simulation / Testing ---
class Dummy_source:

    def __init__(self):
        self.position = 0
        self.filename = 'none'
        self._csv_reader = threading.Thread(target=self._read_orientation_csv,
                                            name='csv_reader',
                                            daemon=True)
        self._csv_reader.start()

    def _read_orientation_csv(self):
        while True:
            self.position += 0.1 * np.random.rand() - 0.05
            time.sleep(0.01)


# --- GUI Application ---
class App:

    def __init__(self, port, title='Box 1', source=None, screen_coordinates=(10, 10)):
        self.stepper = StepperController(port)
        self.source = Dummy_source() if source is None else CSV_source(source)
        self.offset = 0
        self.offset_offset = 0
        self._update_dt = 0.02  # update interval in seconds

        self.tk = tk.Tk()
        self.tk.title('Commutator app')
        self.tk.label = tk.Label(self.tk, text=title)
        self.tk.label.pack()

        # --- File selection button ---
        self.button_explore = tk.Button(self.tk,
                                        text="Choose .csv file manually",
                                        command=self.choose_csv)
        self.button_explore.pack()

        # --- Status labels ---
        self.target_label = tk.Label(self.tk, text=f'target source: {self.source.filename}')
        self.target_label.pack()

        self.source_pos = tk.Label(self.tk, text=f'target position: {np.round(self.source.position / np.pi, 1)}π')
        self.source_pos.pack()

        self.target_pos = tk.Label(self.tk, text=f'target position: {np.round(self.source.position / np.pi, 1)}π')
        self.target_pos.pack()

        current_pos = self.stepper.query_position()
        self.current_pos = tk.Label(self.tk, text=f'current position: {np.round(current_pos / np.pi, 1)}π', fg='red')
        self.current_pos.pack()

        # --- Visualization canvas ---
        self._csize = 60
        self._circwidth = 5
        self._radius = self._csize / 2 - self._circwidth
        self.canvas = tk.Canvas(self.tk, width=self._csize, height=self._csize)
        self.canvas.pack()

        # Circles and lines for motor & target
        self.traj = self.canvas.create_oval(self._csize / 2 - self._radius, self._csize/2 - self._radius,
                                            self._csize/2 + self._radius, self._csize/2 + self._radius, dash=(3, 3))
        self.line = self.canvas.create_line(self._csize / 2, self._csize / 2 - self._radius,
                                            self._csize / 2, self._csize / 2)
        self.circ = self.canvas.create_oval(self._csize/2 - self._circwidth / 2, self._circwidth - self._circwidth/2,
                                            self._csize/2 + self._circwidth/2, 5 + self._circwidth/2, fill='black')
        self.line2 = self.canvas.create_line(self._csize / 2, self._csize / 2 - self._radius,
                                             self._csize / 2, self._csize / 2, dash=(3, 3), fill='red')
        self.circ2 = self.canvas.create_oval(self._csize/2 - self._circwidth / 2, self._circwidth - self._circwidth/2,
                                             self._csize/2 + self._circwidth/2, 5 + self._circwidth/2, outline='red')

        # --- Speed control ---
        def rps_command(rps):
            rps = float(rps)
            self.step_time_fast = int(10**6 / (self.stepper.steps_per_turn * rps))

        self.steptime_label = tk.Label(self.tk, text='\nmaximal speed / ( 2π / s )')
        self.steptime_label.pack()

        self.steptime_slider = tk.Scale(self.tk, from_=0.1, to=2, orient=tk.HORIZONTAL, length=200,
                                        resolution=0.1, command=rps_command)
        self.steptime_slider.pack()
        self.steptime_slider.set(1 / (10**-6 * self.stepper.step_time * self.stepper.steps_per_turn))
        self.step_time_fast = int(10**6 / (self.stepper.steps_per_turn * self.steptime_slider.get()))

        # --- Offset slider ---
        def target_command(val):
            val = float(val)
            self.offset = val * np.pi
            self.target_pos.config(text=f'target position: {np.round((self.offset + self.source.position) / np.pi, 1)}π')

        self.offset_label = tk.Label(self.tk, text='\nmanual offset / π')
        self.offset_label.pack()

        self.offset_slider = tk.Scale(self.tk, from_=-6, to=6, orient=tk.HORIZONTAL,
                                      length=200, resolution=0.1, command=target_command)
        self.offset_slider.pack()
        self.offset_slider.set(0)

        # --- Reset button ---
        def reset_manual_offset():
            self.offset_offset += self.offset
            self.offset_slider.set(0)

        self.reset_button = tk.Button(self.tk, text="reset slider", command=reset_manual_offset)
        self.reset_button.pack()

        # --- Stop/Resume control ---
        def stop_event():
            self.stepper.stop()
            self.stop_button.config(text='RESUME', command=resume_event)

        def resume_event():
            self.stepper.resume()
            self.stop_button.config(text='STOP', command=stop_event)

        self.stop_button = tk.Button(self.tk, text="STOP", command=stop_event)
        self.stop_button.pack()

        # --- Start updater thread ---
        self._updater = threading.Thread(target=self._update_loop, name='updater', daemon=True)
        self._updater.start()

        self.tk.geometry(f'200x400+{screen_coordinates[0]}+{screen_coordinates[1]}')

        def exit_routine():
            self.stepper.reset()
            self.tk.destroy()

        self.tk.protocol("WM_DELETE_WINDOW", exit_routine)
        self.tk.mainloop()

    def _update_loop(self):
        """Continuously updates GUI and synchronizes motor with source."""
        while True:
            target_pos = self.offset + self.offset_offset + self.source.position
            self.source_pos.config(text=f'source position: {np.round(self.source.position / np.pi, 1)}π')
            self.target_pos.config(text=f'target position: {np.round(target_pos / np.pi, 1)}π')

            current_pos = self.stepper.query_position()
            self.current_pos.config(text=f'motor position: {np.round(current_pos / np.pi, 1)}π')

            # If difference exceeds threshold, update motor
            delta = np.abs(current_pos - target_pos)
            if delta > (np.pi / 16):
                self.stepper.set_position(target_pos)

            # Update canvas visualization
            for is_target, pos, line, circ in [(True, target_pos, self.line, self.circ), 
                                               (False, current_pos, self.line2, self.circ2)]:
                x = self._csize / 2 + np.cos(pos) * self._radius
                y = self._csize / 2 + np.sin(pos) * self._radius
                self.canvas.coords(circ, x - self._circwidth/2, y - self._circwidth/2, x + self._circwidth/2, y + self._circwidth/2)
                self.canvas.coords(line, x, y, self._csize / 2, self._csize / 2)

            if self.stepper.step_time != self.step_time_fast:
                self.stepper.set_step_time(self.step_time_fast)

            time.sleep(self._update_dt)

    def choose_csv(self):
        """Allows user to manually choose a new CSV file."""
        filename = filedialog.askopenfilename(initialdir='C:/',
                                              title="Select a File",
                                              filetypes=(("csv files", "*.csv*"), ("all files", "*.*")))
        if filename:
            self._assign_source(filename)

    def _assign_source(self, filename):
        """Load a new CSV source."""
        self.stepper.pos_reset()
        self.source = CSV_source(filename)
        self.target_label.config(text=f'target source: {self.source.filename}')


# --- External interface to start the app ---
def run_commutator(port, title, source, screen_coordinates):
    def _thread_app(port, title, source, screen_coordinates):
        app = App(port, title, source, screen_coordinates)

    thread = threading.Thread(target=_thread_app, args=(port, title, source, screen_coordinates))
    thread.start()
    
if __name__ == '__main__':
    run_commutator(port = 'COM57', title = 'Box 1', source = None, screen_coordinates = (400, 500) )