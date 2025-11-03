#include <Servo.h>
#include <AccelStepper.h>

Servo driveMotors[4];
int drivePins[4] = {2, 3, 4, 5};
int driveSpeeds[4] = {1500, 1500, 1500, 1500};
long targetSpeed[4] = {1500, 1500, 1500, 1500};
bool speedsReached = true;

const int numSteppers = 4;
const int dirPins[4]={46,47,52,53};
const int stepPins[4]={44,45,50,51};
const int sleepPins[4]={42,43,48,49};

AccelStepper stepper0(AccelStepper::DRIVER, stepPins[0], dirPins[0]);
AccelStepper stepper1(AccelStepper::DRIVER, stepPins[1], dirPins[1]);
AccelStepper stepper2(AccelStepper::DRIVER, stepPins[2], dirPins[2]);
AccelStepper stepper3(AccelStepper::DRIVER, stepPins[3], dirPins[3]);
AccelStepper* steppers[numSteppers] = {&stepper0, &stepper1, &stepper2, &stepper3};

long targetSteps[4]  = {0, 0, 0, 0};
bool anglesReached = true;
unsigned long lastMotionTime[4] = {0, 0, 0, 0};
const unsigned long SLEEP_AFTER = 600;  // was 200ms; longer to avoid chatter
bool awake[4] = {false, false, false, false};

const int stepsPerRevolution = 200;     // Default motor steps per revolution
const float gearRatio = 4.0f;           // Default additional gear ratio
const float gearboxRatio = 10.0f;       // Default gearbox ratio

// Calibratable steps-per-degree (runtime adjustable via "SPD <value>")
float stepsPerDeg = 1.6;
long FULL_TURN_STEPS = lround(360.0f * stepsPerDeg);

void updateFullTurnFromSPD() {
  if (stepsPerDeg < 0.0001f) {
    stepsPerDeg = 0.0001f;
  }
  FULL_TURN_STEPS = lround(360.0f * stepsPerDeg);
}

unsigned long lastDebugPrint = 0;
const unsigned long DEBUG_INTERVAL = 200;  // Print debug every 200ms when active

void wake(int i) {
  if (!awake[i]) {
    digitalWrite(sleepPins[i], HIGH);
    awake[i] = true;
    // Serial.print("Waking stepper ");  // DISABLED for speed
    // Serial.println(i);
  }
}

void maybeSleep(int i) {
  if (awake[i] && steppers[i]->distanceToGo() == 0 && targetSpeed[i] == 1500) {
    if (millis() - lastMotionTime[i] >= SLEEP_AFTER) {
      digitalWrite(sleepPins[i], LOW);
      awake[i] = false;
    }
  }
}

long wrapNearest(long current, long desired) {
  long cur = current % FULL_TURN_STEPS;
  if (cur < 0) cur += FULL_TURN_STEPS;
  long des = desired % FULL_TURN_STEPS;
  if (des < 0) des += FULL_TURN_STEPS;
  long diff = des - cur;
  if (diff > FULL_TURN_STEPS / 2) diff -= FULL_TURN_STEPS;
  if (diff < -FULL_TURN_STEPS / 2) diff += FULL_TURN_STEPS;
  return current + diff;
}

void setStepperTarget(int idx, int angleDeg) {
  int desired = ((angleDeg % 360) + 360) % 360;
  long desiredSteps = lround(desired * stepsPerDeg);
  long current = steppers[idx]->currentPosition();
  long goal = wrapNearest(current, desiredSteps);
  // Deadband: ignore tiny moves < 0.5 deg
  long deadband = lround(0.5f * stepsPerDeg);
  if (labs(goal - current) < deadband) {
    return;
  }
  targetSteps[idx] = goal;
  wake(idx);
  steppers[idx]->moveTo(goal);
  lastMotionTime[idx] = millis();
  anglesReached = false;
  // Serial.print("Set stepper ");  // DISABLED for speed
  // Serial.print(idx);
  // Serial.print(" target angle ");
  // Serial.print(angleDeg);
  // Serial.print(" deg, steps goal ");
  // Serial.print(goal);
  // Serial.print(" from current ");
  // Serial.print(current);
  // Serial.print(" (diff ");
  // Serial.print(goal - current);
  // Serial.println(")");
}

void setupSteppers() {
  for (int i = 0; i < numSteppers; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    pinMode(sleepPins[i], OUTPUT);
    digitalWrite(stepPins[i], LOW);
    digitalWrite(dirPins[i], LOW);
    digitalWrite(sleepPins[i], HIGH);  // Keep drivers awake
    awake[i] = true;
    steppers[i]->setMaxSpeed(800.0f);
    steppers[i]->setAcceleration(300.0f);
    steppers[i]->setMinPulseWidth(3);
    steppers[i]->setCurrentPosition(0);
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    driveMotors[i].attach(drivePins[i]);
    driveMotors[i].writeMicroseconds(1500);  // Initialize to neutral (stopped)
    delay(50);  // Brief delay for ESCs to recognize signal
  }
  setupSteppers();
  Serial.println("Setup complete - ready for commands");
}

void loopSteppers() {
  bool allReached = true;
  bool anyMoving = false;
  for (int i = 0; i < numSteppers; i++) {
    long d2g = steppers[i]->distanceToGo();
    if (d2g != 0) {
      steppers[i]->run();
      lastMotionTime[i] = millis();
      allReached = false;
      anyMoving = true;
    } else {
      maybeSleep(i);
    }
  }
  anglesReached = allReached;

  // Debug print if moving, throttled - DISABLED for speed
  // if (anyMoving && (millis() - lastDebugPrint >= DEBUG_INTERVAL)) {
  //   Serial.println("Stepper status:");
  //   for (int i = 0; i < numSteppers; i++) {
  //     Serial.print("  Stepper ");
  //     Serial.print(i);
  //     Serial.print(": pos ");
  //     Serial.print(steppers[i]->currentPosition());
  //     Serial.print(", d2go ");
  //     Serial.print(steppers[i]->distanceToGo());
  //     Serial.print(", speed ");
  //     Serial.println(steppers[i]->speed());
  //   }
  //   lastDebugPrint = millis();
  // }
}

void setDriveSpeeds() {
  bool allEqual = true;
  // Serial.println("setDriveSpeeds called - current state:");  // DISABLED for speed
  // for (int i = 0; i < 4; i++) {
  //   Serial.print("  Motor ");
  //   Serial.print(i);
  //   Serial.print(": current ");
  //   Serial.print(driveSpeeds[i]);
  //   Serial.print(", target ");
  //   Serial.println(targetSpeed[i]);
  // }

  for (int i = 0; i < 4; i++) {
    if (driveSpeeds[i] != targetSpeed[i]) {
      allEqual = false;
      long delta = targetSpeed[i] - driveSpeeds[i];
      driveSpeeds[i] += lround(delta * 0.65f);
      if (abs(targetSpeed[i] - driveSpeeds[i]) < 30) {
        driveSpeeds[i] = targetSpeed[i];
      }
      // Serial.print("Updating motor ");  // DISABLED for speed
      // Serial.print(i);
      // Serial.print(" to ");
      // Serial.println(driveSpeeds[i]);
      driveMotors[i].writeMicroseconds(driveSpeeds[i]);
    }
  }

  if (allEqual) {
    speedsReached = true;
  } else {
    speedsReached = true;
    for (int i = 0; i < 4; i++) {
      if (driveSpeeds[i] != targetSpeed[i]) {
        speedsReached = false;
        break;
      }
    }
  }

  // Serial.print("setDriveSpeeds complete - speedsReached now ");  // DISABLED for speed
  // Serial.println(speedsReached ? "true" : "false");
}

void loop() {
  if (Serial.available()) {  // Process one line at a time without blocking
    String input = Serial.readStringUntil('\n');
    input.trim();
    // Serial.print("Received input: ");  // DISABLED for speed
    // Serial.println(input);

    if (input.startsWith("D")) {
      // Flexible formats:
      //  D v                -> apply v to all 4
      //  D v1 v2 v3 v4 ...  -> use first 4 values (ignore extras)
      // Value rules per wheel:
      //  - 1000..2000 => absolute µs (with min effective delta enforcement)
      //  -  -500..500 => offset from 1500 (clamped to 1000..2000)
      // Enforce a minimum effective delta so tiny changes like 1510 nudge the ESC
      const long MIN_DRIVE_DELTA = 20; // µs

      // Tokenize to support more than 4 numbers (we'll take first 4)
      long vals[4] = {0,0,0,0};
      int count = 0;
      {
        // Copy string (String -> c_str is OK for sscanf/strtok with a temp buffer)
        char buf[64];
        input.toCharArray(buf, sizeof(buf));
        char* saveptr = nullptr;
        char* tok = strtok_r(buf, " ", &saveptr); // "D"
        while (tok && count <= 4) {
          if (count > 0) { // skip the initial "D"
            long v = atol(tok);
            if (count-1 < 4) vals[count-1] = v;
          }
          tok = strtok_r(nullptr, " ", &saveptr);
          count++;
        }
      }

      int numVals = (count-1); // number of numeric tokens after 'D'
      if (numVals == 1 || numVals >= 4) {
        if (numVals == 1) { vals[1] = vals[0]; vals[2] = vals[0]; vals[3] = vals[0]; }

        auto to_pwm = [MIN_DRIVE_DELTA](long v) -> long {
          if (v >= 1000 && v <= 2000) {
            long pwm = v;
            long delta = pwm - 1500L;
            if (delta != 0 && labs(delta) < MIN_DRIVE_DELTA) {
              pwm = 1500L + (delta > 0 ? MIN_DRIVE_DELTA : -MIN_DRIVE_DELTA);
            }
            if (pwm < 1000L) pwm = 1000L;
            if (pwm > 2000L) pwm = 2000L;
            return pwm;
          }
          if (v > -500 && v < 500) {
            long pwm = 1500L + v;
            long delta = pwm - 1500L;
            if (delta != 0 && labs(delta) < MIN_DRIVE_DELTA) {
              pwm = 1500L + (delta > 0 ? MIN_DRIVE_DELTA : -MIN_DRIVE_DELTA);
            }
            if (pwm < 1000L) pwm = 1000L;
            if (pwm > 2000L) pwm = 2000L;
            return pwm;
          }
          // Fallback clamp
          if (v < 1000L) return 1000L;
          if (v > 2000L) return 2000L;
          return v;
        };

        targetSpeed[0] = to_pwm(vals[0]);
        targetSpeed[1] = to_pwm(vals[1]);
        targetSpeed[2] = to_pwm(vals[2]);
        targetSpeed[3] = to_pwm(vals[3]);

        speedsReached = false;
        for (int i = 0; i < numSteppers; i++) { wake(i); }
      }
    } else if (input.startsWith("S")) {
      // Flexible formats:
      //  S a              -> applies a to all 4
      //  S a b c d        -> per-wheel absolute angles (degrees)
      long t1, t2, t3, t4;
      int matched = sscanf(input.c_str(), "S %ld %ld %ld %ld", &t1, &t2, &t3, &t4);
      if (matched == 1 || matched == 4) {
        if (matched == 1) { t2 = t3 = t4 = t1; }
        // SIMULTANEOUS MODE: Just steer, don't stop drive motors
        setStepperTarget(0, (int)t1);
        setStepperTarget(1, (int)t2);
        setStepperTarget(2, (int)t3);
        setStepperTarget(3, (int)t4);
      }
    } else if (input.startsWith("C")) {
      int modNum, amount;
      if (sscanf(input.c_str(), "C %d %d", &modNum, &amount) == 2) {
        if (modNum >= 0 && modNum < numSteppers) {
          long newPos = lround(amount * stepsPerDeg);
          steppers[modNum]->setCurrentPosition(newPos);
          // Serial.print("Calibrated stepper ");  // DISABLED for speed
          // Serial.print(modNum);
          // Serial.print(" to ");
          // Serial.println(newPos);
        }
      }
    } else if (input.startsWith("SPD")) {
      // Runtime set steps-per-degree: SPD <float>
      String v = input.substring(3);
      v.trim();
      if (v.length() > 0) {
        float spd = v.toFloat();
        if (spd > 0.01f && spd < 2000.0f) {
          stepsPerDeg = spd;
          updateFullTurnFromSPD();
          Serial.print("[SPD] steps/deg=");
          Serial.print(stepsPerDeg, 6);
          Serial.print(" fullTurnSteps=");
          Serial.println(FULL_TURN_STEPS);
        } else {
          Serial.println("[SPD] ignored (out of range)");
        }
      }
    }
  }

  // SIMULTANEOUS MODE: Run both steppers and drive motors at the same time
  loopSteppers();  // Always run steppers
  
  if (!speedsReached) {
    setDriveSpeeds();  // Run drive motors if they need to move
  }
}