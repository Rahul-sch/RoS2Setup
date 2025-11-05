// ARDUINO #3: TOP MECHANISM CONTROL
// Controls 4 DC motors (2 modules) for arm/gripper
// Receives commands:
//   U1  = Module 1 UP
//   D1  = Module 1 DOWN
//   U2  = Module 2 UP
//   D2  = Module 2 DOWN
//   S   = STOP all motors
//   M <mod> <speed>  = Set module speed (-255 to 255)

#define M1_RPWM 4
#define M1_LPWM 3
#define M2_RPWM 6
#define M2_LPWM 5
#define M3_RPWM 10
#define M3_LPWM 9
#define M4_RPWM 12
#define M4_LPWM 11

#define DEFAULT_SPEED 200  // Default PWM 0-255

// Current speeds for each module (-255 to 255)
int moduleSpeed[2] = {0, 0};
int targetSpeed[2] = {0, 0};

void setup() {
    Serial.begin(115200);  // Match other Arduinos

    int pins[] = {M1_RPWM, M1_LPWM, M2_RPWM, M2_LPWM,
                  M3_RPWM, M3_LPWM, M4_RPWM, M4_LPWM};

    for (int i = 0; i < 8; i++) {
        pinMode(pins[i], OUTPUT);
        analogWrite(pins[i], 0);
    }

    Serial.println("Top mechanism ready - commands: U1, D1, U2, D2, S, M <mod> <speed>");
}

// --- Stop all motors ---
void stopAll() {
    int pins[] = {M1_RPWM, M1_LPWM, M2_RPWM, M2_LPWM,
                  M3_RPWM, M3_LPWM, M4_RPWM, M4_LPWM};

    for (int i = 0; i < 8; i++) {
        analogWrite(pins[i], 0);
    }

    targetSpeed[0] = 0;
    targetSpeed[1] = 0;
    moduleSpeed[0] = 0;
    moduleSpeed[1] = 0;
}

// --- Set motor speeds for a module ---
void setModuleSpeed(int module, int speed) {
    // module: 0 or 1 (Module 1 or Module 2)
    // speed: -255 to 255 (negative = DOWN, positive = UP)

    int r1, l1, r2, l2;

    if (module == 0) {
        r1 = M1_RPWM;
        l1 = M1_LPWM;
        r2 = M2_RPWM;
        l2 = M2_LPWM;
    } else if (module == 1) {
        r1 = M3_RPWM;
        l1 = M3_LPWM;
        r2 = M4_RPWM;
        l2 = M4_LPWM;
    } else {
        return;
    }

    // Clamp speed to -255 to 255
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        // UP direction
        analogWrite(r1, speed);
        analogWrite(l1, 0);
        analogWrite(r2, speed);
        analogWrite(l2, 0);
    } else if (speed < 0) {
        // DOWN direction
        analogWrite(r1, 0);
        analogWrite(l1, -speed);
        analogWrite(r2, 0);
        analogWrite(l2, -speed);
    } else {
        // STOP
        analogWrite(r1, 0);
        analogWrite(l1, 0);
        analogWrite(r2, 0);
        analogWrite(l2, 0);
    }
}

// --- Smooth speed ramping (like drive motors) ---
void updateSpeeds() {
    for (int i = 0; i < 2; i++) {
        if (moduleSpeed[i] != targetSpeed[i]) {
            int delta = targetSpeed[i] - moduleSpeed[i];
            // Ramp by 65% each iteration
            moduleSpeed[i] += (int)(delta * 0.65f);

            // Snap to target if close enough
            if (abs(targetSpeed[i] - moduleSpeed[i]) < 10) {
                moduleSpeed[i] = targetSpeed[i];
            }

            setModuleSpeed(i, moduleSpeed[i]);
        }
    }
}

void loop() {
    static String input = "";

    // Read serial commands
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (input.length() > 0) {
                input.trim();

                // Parse command
                if (input.startsWith("S")) {
                    // STOP all
                    stopAll();
                    Serial.println("STOP");

                } else if (input.startsWith("U") || input.startsWith("D")) {
                    // Simple UP/DOWN commands: U1, D1, U2, D2
                    char dir = input[0];
                    if (input.length() >= 2) {
                        int module = input[1] - '0';

                        if (module == 1 || module == 2) {
                            int modIdx = module - 1;  // 0 or 1
                            targetSpeed[modIdx] = (dir == 'U') ? DEFAULT_SPEED : -DEFAULT_SPEED;

                            Serial.print("Module ");
                            Serial.print(module);
                            Serial.println((dir == 'U') ? " UP" : " DOWN");
                        }
                    }

                } else if (input.startsWith("M")) {
                    // Advanced command: M <module> <speed>
                    // Example: M 1 200  (Module 1 at speed 200)
                    //          M 2 -150 (Module 2 at speed -150)
                    int module, speed;
                    if (sscanf(input.c_str(), "M %d %d", &module, &speed) == 2) {
                        if (module == 1 || module == 2) {
                            int modIdx = module - 1;
                            targetSpeed[modIdx] = speed;

                            Serial.print("Module ");
                            Serial.print(module);
                            Serial.print(" speed set to ");
                            Serial.println(speed);
                        }
                    }
                }

                input = "";
            }
        } else {
            input += c;
        }
    }

    // Update motor speeds smoothly
    updateSpeeds();

    delay(10);  // Small delay for smooth ramping
}
