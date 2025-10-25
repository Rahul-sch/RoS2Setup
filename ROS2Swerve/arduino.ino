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
const unsigned long SLEEP_AFTER = 200;
bool awake[4] = {false, false, false, false};

const int stepsPerRevolution = 200;
const float gearRatio = 4.0f;
const float gearboxRatio = 10.0f;
const float STEPS_PER_DEG = (stepsPerRevolution * gearRatio * gearboxRatio) / 360.0f;
const long FULL_TURN = lround(360.0f * STEPS_PER_DEG);

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
      // Serial.print("Sleeping stepper ");  // DISABLED for speed
      // Serial.println(i);
    }
  }
}

long wrapNearest(long current, long desired) {
  long cur = current % FULL_TURN;
  if (cur < 0) cur += FULL_TURN;
  long des = desired % FULL_TURN;
  if (des < 0) des += FULL_TURN;
  long diff = des - cur;
  if (diff > FULL_TURN / 2) diff -= FULL_TURN;
  if (diff < -FULL_TURN / 2) diff += FULL_TURN;
  return current + diff;
}

void setStepperTarget(int idx, int angleDeg) {
  int desired = ((angleDeg % 360) + 360) % 360;
  long desiredSteps = lround(desired * STEPS_PER_DEG);
  long current = steppers[idx]->currentPosition();
  long goal = wrapNearest(current, desiredSteps);
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
    digitalWrite(sleepPins[i], LOW);
    awake[i] = false;
    steppers[i]->setMaxSpeed(1200.0f);
    steppers[i]->setAcceleration(1000.0f);  // Lowered to reduce jolting
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
      int s1, s2, s3, s4;
      if (sscanf(input.c_str(), "D %d %d %d %d", &s1, &s2, &s3, &s4) == 4) {
        targetSpeed[0] = s1;
        targetSpeed[1] = s2;
        targetSpeed[2] = s3;
        targetSpeed[3] = s4;
        speedsReached = false;
        // Wake steppers to hold position during drive
        for (int i = 0; i < numSteppers; i++) {
          wake(i);
        }
        // Serial.println("D command received - speedsReached set to false");  // DISABLED for speed
      }
    } else if (input.startsWith("S")) {
      int t1, t2, t3, t4;
      if (sscanf(input.c_str(), "S %d %d %d %d", &t1, &t2, &t3, &t4) == 4) {
        // Serial.println("S command received - steering (simultaneous mode)");  // DISABLED for speed
        
        // SIMULTANEOUS MODE: Just steer, don't stop drive motors
        // Drive motors will keep running at their current speed
        setStepperTarget(0, t1);
        setStepperTarget(1, t2);
        setStepperTarget(2, t3);
        setStepperTarget(3, t4);
      }
    } else if (input.startsWith("C")) {
      int modNum, amount;
      if (sscanf(input.c_str(), "C %d %d", &modNum, &amount) == 2) {
        if (modNum >= 0 && modNum < numSteppers) {
          long newPos = lround(amount * STEPS_PER_DEG);
          steppers[modNum]->setCurrentPosition(newPos);
          // Serial.print("Calibrated stepper ");  // DISABLED for speed
          // Serial.print(modNum);
          // Serial.print(" to ");
          // Serial.println(newPos);
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