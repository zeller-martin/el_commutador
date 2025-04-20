// ------------------- Pin Definitions -------------------
#define stp         1   // Step pin for stepper driver
#define dir         0   // Direction pin
#define MS1         6   // Microstep control pin 1
#define MS2         5   // Microstep control pin 2
#define EN          7   // Enable pin
#define LED_DISABLE 8   // Optional LED for showing disabled state

#define MIN_SPEED_FAC 0.02  // Minimum speed factor for smooth deceleration

// ------------------- State Variables -------------------
long pos      = 0;        // Current motor position (in steps)
long target   = 0;        // Target position
long delta    = 0;        // Distance to move
long step_t   = 5000;     // Base step delay in microseconds
long _step_t  = step_t;   // Adjusted step delay for dynamic speed
long steps_per_turn = 200; // Steps per full rotation (200 or 3200 in microstep)

float fac     = 1;        // Speed scaling factor

// ------------------- Timing -------------------
elapsedMicros clock_time;
unsigned long next_off_time = 0;
unsigned long next_on_time  = 0;

// ------------------- Communication Buffer -------------------
byte send_arr[4]; // Used for sending 4-byte integers (position)

// ------------------- Flags -------------------
bool step_on   = false;
bool in_pause  = false;


// ------------------- Functions -------------------

// Parses a long integer from the serial buffer until 'X' is received
void assign_param(long int *param) {
  String received_string;
  char c;

  while (1) {
    if (Serial.available()) {
      c = Serial.read();
      if (c == 'X') {
        break;
      } else {
        received_string += c;
      }
    }
  }

  received_string.replace("\n", "");
  *param = received_string.toInt();
}

// Resets all stepper-related pins to default inactive states
void resetBEDPins() {
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(EN, HIGH);
}

// Enables or disables microstepping (sets steps_per_turn accordingly)
void activate_microstep(bool yes) {
  if (yes) {
    digitalWrite(MS1, HIGH);
    digitalWrite(MS2, HIGH);
    steps_per_turn = 3200;  // 1/16 microstepping
  } else {
    digitalWrite(MS1, LOW);
    digitalWrite(MS2, LOW);
    steps_per_turn = 200;   // Full step
  }
}

// Flash LED to enable assignment of serial port to physical device
void ping() {
  for (byte i = 0; i < 10; ++i) {
    digitalWrite(LED_DISABLE, HIGH);
    delay(50);
    digitalWrite(LED_DISABLE, LOW);
    delay(50);
  }
}


// ------------------- Setup -------------------

void setup() {
  // Configure pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(LED_DISABLE, OUTPUT);

  resetBEDPins();

  // Initialize serial communication
  Serial.begin(115200);

  // Enable the stepper driver
  digitalWrite(EN, LOW);

  // Enable microstepping by default
  activate_microstep(true);
}


// ------------------- Main Loop -------------------

void loop() {
  // --- Handle Serial Commands ---
  if (Serial.available()) {
    byte recv_byte = Serial.read();

    switch (recv_byte) {
      case 'P': assign_param(&target); break;       // Set new target position
      case 'F': ping(); break;                      // Flash status LED
      case 'R':                                     // Reset state and enable microstep
        pos = 0;
        target = 0;
        clock_time = 0;
        next_on_time = 0;
        next_off_time = 0;
        activate_microstep(true);
        in_pause = false;
        break;
      case 'T': assign_param(&step_t); break;       // Set base step time
      case 'M': activate_microstep(true); break;    // Enable microstepping
      case 'N': activate_microstep(false); break;   // Disable microstepping
      case 'S':                                     // Stop motor
        in_pause = true;
        digitalWrite(LED_DISABLE, HIGH);
        break;
      case 'G':                                     // Resume motor
        in_pause = false;
        digitalWrite(LED_DISABLE, LOW);
        break;
      case 'Q':                                     // Send current position as 4 bytes
        send_arr[0] =  pos        & 0xFF;
        send_arr[1] = (pos >>  8) & 0xFF;
        send_arr[2] = (pos >> 16) & 0xFF;
        send_arr[3] = (pos >> 24) & 0xFF;
        Serial.write(send_arr, 4);
        break;
    }
  }

  // --- Determine Direction and Distance to Target ---
  if (pos > target) {
    digitalWrite(dir, HIGH);     // Set direction backward
    delta = pos - target;
  } else {
    digitalWrite(dir, LOW);      // Set direction forward
    delta = target - pos;
  }

  // --- Dynamically Adjust Step Time Based on Distance ---
  if (delta > (steps_per_turn / 8)) {
    _step_t = step_t;
  } else {
    fac = float(step_t) * (1 / (((1 - MIN_SPEED_FAC) * (float(delta) / (steps_per_turn / 8))) + MIN_SPEED_FAC));
    _step_t = long(fac);
  }

  // --- Step Motor if Not Paused ---
  if (!in_pause) {
    if (pos != target) {
      digitalWrite(EN, LOW);  // Ensure driver is enabled

      if (!step_on && clock_time >= next_on_time) {
        digitalWrite(stp, HIGH);            // Start step pulse
        digitalWrite(LED_BUILTIN, HIGH);
        step_on = true;
        next_off_time = _step_t;
        clock_time = 0;
      } else if (step_on && clock_time >= next_off_time) {
        digitalWrite(stp, LOW);             // End step pulse
        digitalWrite(LED_BUILTIN, LOW);
        step_on = false;
        next_on_time = _step_t;
        clock_time = 0;

        // Update position
        if (digitalRead(dir)) {
          pos--;
        } else {
          pos++;
        }
      }
    } else {
      // Hold current state if target reached
      digitalWrite(stp, LOW);
      step_on = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
