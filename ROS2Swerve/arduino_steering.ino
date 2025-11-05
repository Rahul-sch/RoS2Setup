// ARDUINO #1: STEERING CONTROL ONLY
// Controls 4 stepper motors for wheel angles
// Receives commands: S <angle1> <angle2> <angle3> <angle4>
//                    C <module> <angle> (calibrate)
//                    SPD <value> (set steps per degree)

#include <AccelStepper.h>

const int numSteppers = 4;
const int dirPins[4] = {46, 47, 52, 53};
const int stepPins[4] = {44, 45, 50, 51};
const int sleepPins[4] = {42, 43, 48, 49};

AccelStepper stepper0(AccelStepper::DRIVER, stepPins[0], dirPins[0]);
AccelStepper stepper1(AccelStepper::DRIVER, stepPins[1], dirPins[1]);
AccelStepper stepper2(AccelStepper::DRIVER, stepPins[2], dirPins[2]);
AccelStepper stepper3(AccelStepper::DRIVER, stepPins[3], dirPins[3]);
AccelStepper* steppers[numSteppers] = {&stepper0, &stepper1, &stepper2, &stepper3};

long targetSteps[4] = {0, 0, 0, 0};
bool anglesReached = true;
unsigned long lastMotionTime[4] = {0, 0, 0, 0};
const unsigned long SLEEP_AFTER = 600;
bool awake[4] = {false, false, false, false};

// Calibratable steps-per-degree
float stepsPerDeg = 1.6;
long FULL_TURN_STEPS = lround(360.0f * stepsPerDeg);

void updateFullTurnFromSPD() {
  if (stepsPerDeg < 0.0001f) {
    stepsPerDeg = 0.0001f;
  }
  FULL_TURN_STEPS = lround(360.0f * stepsPerDeg);
}

void wake(int i) {
  if (!awake[i]) {
    digitalWrite(sleepPins[i], HIGH);
    awake[i] = true;
  }
}

void maybeSleep(int i) {
  if (awake[i] && steppers[i]->distanceToGo() == 0) {
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
}

void setupSteppers() {
  for (int i = 0; i < numSteppers; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    pinMode(sleepPins[i], OUTPUT);
    digitalWrite(stepPins[i], LOW);
    digitalWrite(dirPins[i], LOW);
    digitalWrite(sleepPins[i], HIGH);
    awake[i] = true;
    steppers[i]->setMaxSpeed(800.0f);
    steppers[i]->setAcceleration(300.0f);
    steppers[i]->setMinPulseWidth(3);
    steppers[i]->setCurrentPosition(0);
  }
}

void setup() {
  Serial.begin(115200);
  setupSteppers();
  Serial.println("[STEERING] Ready - 4 stepper motors");
}

void loopSteppers() {
  bool allReached = true;
  for (int i = 0; i < numSteppers; i++) {
    long d2g = steppers[i]->distanceToGo();
    if (d2g != 0) {
      steppers[i]->run();
      lastMotionTime[i] = millis();
      allReached = false;
    } else {
      maybeSleep(i);
    }
  }
  anglesReached = allReached;
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("S")) {
      // S a b c d  -> per-wheel absolute angles (degrees)
      // S a        -> applies a to all 4
      long t1, t2, t3, t4;
      int matched = sscanf(input.c_str(), "S %ld %ld %ld %ld", &t1, &t2, &t3, &t4);
      if (matched == 1 || matched == 4) {
        if (matched == 1) { t2 = t3 = t4 = t1; }
        setStepperTarget(0, (int)t1);
        setStepperTarget(1, (int)t2);
        setStepperTarget(2, (int)t3);
        setStepperTarget(3, (int)t4);
        Serial.println("[STEERING] Command received");
      }
    } else if (input.startsWith("C")) {
      // C <module> <angle> - Calibrate stepper position
      int modNum, amount;
      if (sscanf(input.c_str(), "C %d %d", &modNum, &amount) == 2) {
        if (modNum >= 0 && modNum < numSteppers) {
          long newPos = lround(amount * stepsPerDeg);
          steppers[modNum]->setCurrentPosition(newPos);
          Serial.print("[STEERING] Calibrated stepper ");
          Serial.print(modNum);
          Serial.print(" to ");
          Serial.println(newPos);
        }
      }
    } else if (input.startsWith("SPD")) {
      // SPD <float> - Set steps per degree
      String v = input.substring(3);
      v.trim();
      if (v.length() > 0) {
        float spd = v.toFloat();
        if (spd > 0.01f && spd < 2000.0f) {
          stepsPerDeg = spd;
          updateFullTurnFromSPD();
          Serial.print("[STEERING] steps/deg=");
          Serial.print(stepsPerDeg, 6);
          Serial.print(" fullTurnSteps=");
          Serial.println(FULL_TURN_STEPS);
        } else {
          Serial.println("[STEERING] SPD ignored (out of range)");
        }
      }
    }
  }

  loopSteppers();
}
