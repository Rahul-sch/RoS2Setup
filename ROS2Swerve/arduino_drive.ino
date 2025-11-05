// ARDUINO #2: DRIVE CONTROL ONLY
// Controls 4 ESCs for the wheel drive motors.
// Command format over serial (115200 baud, newline terminated):
//   S <pwm1> <pwm2> <pwm3> <pwm4>   -- smoothly ramp to per wheel values
//   T <motor> <pwm>                 -- test individual motor (motor 1-4)
//   STATUS                          -- print current PWM values
//   STOP                            -- stop all motors (neutral)

#include <Servo.h>

static const uint8_t DRIVE_PINS[4] = {2, 3, 4, 5};
bool speedsReached = true;
int driveSpeeds[4] = {1500,1500,1500,1500};
int targetSpeed[4] = {1500, 1500, 1500, 1500};

Servo driveMotors[4];
int currentPwm[4] = {1500, 1500, 1500, 1500};

void printStatus() {
  Serial.print("[STATUS] Motors: ");
  for (int i = 0; i < 4; i++) {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(currentPwm[i]);
    if (i < 3) Serial.print(", ");
  }
  Serial.println();
}

// Clamp PWM to safe range
int clampPwm(int pwm) {
  if (pwm < 1000) return 1000;
  if (pwm > 2000) return 2000;
  return pwm;
}

// Smoothly ramp motors to target speeds to prevent jolting
void setDriveSpeeds(){
  int delayTime = 10;
  speedsReached = true;
  for (int i = 0; i < 4; i++) {
    if (driveSpeeds[i] != targetSpeed[i]) {
      speedsReached = false;
      driveSpeeds[i] = driveSpeeds[i] + (targetSpeed[i] - driveSpeeds[i])*.65;
      if (abs(targetSpeed[i] - driveSpeeds[i]) < 30){
        driveSpeeds[i] = targetSpeed[i];
        speedsReached = true;
      }
      driveMotors[i].writeMicroseconds(driveSpeeds[i]);
    }

  }
    delay(delayTime);

}


void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    driveMotors[i].attach(DRIVE_PINS[i]);
    driveMotors[i].writeMicroseconds(1500);
  }

  targetSpeed[0] = 1500;
  targetSpeed[1] = 1500;
  targetSpeed[2] = 1500;
  targetSpeed[3] = 1500;



  Serial.println("[DRIVE] Ready - 4 drive motors");
}

void loop() {

  if (!speedsReached){
      setDriveSpeeds();
    }

  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if (s.startsWith("D")) {
      //Serial.println("received");

      int s1, s2, s3, s4;
      int n = sscanf(s.c_str(), "D %d %d %d %d", &s1, &s2, &s3, &s4);
      if (n == 4) {
        targetSpeed[0] = s1;
        targetSpeed[1] = s2;
        targetSpeed[2] = s3;
        targetSpeed[3] = s4;

        speedsReached = false;
        //Serial.println("moved");
      }
  }
  else if (s.equalsIgnoreCase("STOP")) {
    targetSpeed[0] = 1500;
    targetSpeed[1] = 1500;
    targetSpeed[2] = 1500;
    targetSpeed[3] = 1500;
    speedsReached = false;

    Serial.println("[STOP] All motors neutral");
  }
}


