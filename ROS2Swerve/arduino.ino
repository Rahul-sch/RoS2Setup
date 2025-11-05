#include <Servo.h>
#include <AccelStepper.h>

#define DIR1 2
#define STEP1 3
#define SLEEP1 4
#define DIR2 5
#define STEP2 6
#define SLEEP2 7
#define DIR3 8
#define STEP3 9
#define SLEEP3 10
#define DIR4 A0
#define STEP4 A1
#define SLEEP4 A2

Servo driveMotors[4];
int drivePins[4] = {22, 23, 24, 25};
int driveSpeeds[4] = {1500, 1500, 1500, 1500};
long targetSpeed[4] = {1500, 1500, 1500, 1500};
bool speedsReached = true;

const int numSteppers = 4;
const int dirPins[4] = {DIR1, DIR2, DIR3, DIR4};
const int stepPins[4] = {STEP1, STEP2, STEP3, STEP4};
const int sleepPins[4] = {SLEEP1, SLEEP2, SLEEP3, SLEEP4};

AccelStepper stepper0(AccelStepper::DRIVER, stepPins[0], dirPins[0]);
AccelStepper stepper1(AccelStepper::DRIVER, stepPins[1], dirPins[1]);
AccelStepper stepper2(AccelStepper::DRIVER, stepPins[2], dirPins[2]);
AccelStepper stepper3(AccelStepper::DRIVER, stepPins[3], dirPins[3]);
AccelStepper* steppers[numSteppers] = {&stepper0, &stepper1, &stepper2, &stepper3};

long targetSteps[4]  = {0, 0, 0, 0};
bool anglesReached = true;
unsigned long lastMotionTime[4] = {0, 0, 0, 0};
const unsigned long SLEEP_AFTER = 600;
bool awake[4] = {false, false, false, false};

const int stepsPerRevolution = 200;
const float gearRatio = 4.0f;
const float gearboxRatio = 10.0f;

float stepsPerDeg = 1.6;
long FULL_TURN_STEPS = lround(360.0f * stepsPerDeg);

void updateFullTurnFromSPD() {
  if (stepsPerDeg < 0.0001f) stepsPerDeg = 0.0001f;
  FULL_TURN_STEPS = lround(360.0f * stepsPerDeg);
}

unsigned long lastDebugPrint = 0;
const unsigned long DEBUG_INTERVAL = 200;

void wake(int i) {
  if (!awake[i]) {
    digitalWrite(sleepPins[i], HIGH);
    awake[i] = true;
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
  long deadband = lround(0.5f * stepsPerDeg);
  if (labs(goal - current) < deadband) return;
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
  for (int i = 0; i < 4; i++) {
    driveMotors[i].attach(drivePins[i]);
    driveMotors[i].writeMicroseconds(1500);
    delay(50);
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
}

void setDriveSpeeds() {
  bool allEqual = true;
  for (int i = 0; i < 4; i++) {
    if (driveSpeeds[i] != targetSpeed[i]) {
      allEqual = false;
      long delta = targetSpeed[i] - driveSpeeds[i];
      driveSpeeds[i] += lround(delta * 0.65f);
      if (abs(targetSpeed[i] - driveSpeeds[i]) < 30) driveSpeeds[i] = targetSpeed[i];
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
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("D")) {
      const long MIN_DRIVE_DELTA = 20;
      long vals[4] = {0,0,0,0};
      int count = 0;
      {
        char buf[64];
        input.toCharArray(buf, sizeof(buf));
        char* saveptr = nullptr;
        char* tok = strtok_r(buf, " ", &saveptr);
        while (tok && count <= 4) {
          if (count > 0) {
            long v = atol(tok);
            if (count-1 < 4) vals[count-1] = v;
          }
          tok = strtok_r(nullptr, " ", &saveptr);
          count++;
        }
      }
      int numVals = (count-1);
      if (numVals == 1 || numVals >= 4) {
        if (numVals == 1) { vals[1] = vals[0]; vals[2] = vals[0]; vals[3] = vals[0]; }
        auto to_pwm = [MIN_DRIVE_DELTA](long v) -> long {
          if (v >= 1000 && v <= 2000) {
            long pwm = v;
            long delta = pwm - 1500L;
            if (delta != 0 && labs(delta) < MIN_DRIVE_DELTA) pwm = 1500L + (delta > 0 ? MIN_DRIVE_DELTA : -MIN_DRIVE_DELTA);
            if (pwm < 1000L) pwm = 1000L;
            if (pwm > 2000L) pwm = 2000L;
            return pwm;
          }
          if (v > -500 && v < 500) {
            long pwm = 1500L + v;
            long delta = pwm - 1500L;
            if (delta != 0 && labs(delta) < MIN_DRIVE_DELTA) pwm = 1500L + (delta > 0 ? MIN_DRIVE_DELTA : -MIN_DRIVE_DELTA);
            if (pwm < 1000L) pwm = 1000L;
            if (pwm > 2000L) pwm = 2000L;
            return pwm;
          }
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
      long t1, t2, t3, t4;
      int matched = sscanf(input.c_str(), "S %ld %ld %ld %ld", &t1, &t2, &t3, &t4);
      if (matched == 1 || matched == 4) {
        if (matched == 1) { t2 = t3 = t4 = t1; }
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
        }
      }
    } else if (input.startsWith("SPD")) {
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
  loopSteppers();
  if (!speedsReached) setDriveSpeeds();
}