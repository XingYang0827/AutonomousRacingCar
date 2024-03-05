#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7
uint16_t prevSensorValues[8];
int intSensorValues[8];
float floatSensorValues[8];

const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const int minimumCal[8] = {565, 588, 611, 472, 518, 495, 658, 754};
const int maximumCal[8] = {1394, 850, 967, 564, 589, 565, 1301, 1746};

const int LED_RF = 41;

const float Kp = 0.015; // subject to change
const float Kd = 0.1; // subject to change

int numStopped = 0;
bool isStarted = false;

int previous;
int current;
int baseSpd = 50;
int leftSpd = baseSpd;
int rightSpd = baseSpd;

int distance = 100;
int wheelSpd = baseSpd;


int numVarsToPrint = 10000;
//int Variable_Store[1500][numVarsToPrint];
/*
void StopMotors() {
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
  //digitalWrite(LED_RF, HIGH); // turn on yellow right front LED
  while(true) {
    if (!digitalRead(PUSH1)) {
      //digitalWrite(LED_RR, HIGH);
      for(int i = 0; i < 1500; i++) {
        for (int k = 0; k < numVarsToPrint - 1; k++) {// figure out
numVarsToPrint
          Serial.print(Variable_Store[i][k]);Serial.print('\t');
        }
      }
      Serial.println(Variable_Store[i][numVarsToPrint-1]);
    }
  }
}*/

bool reachedEnd(uint16_t* previousValues, uint16_t* currentValues) {
  for (int i = 0; i < 8; i++) {
    if (previousValues[i] < 1200 || currentValues[i] < 1200) {
      return false;
    }
  }
  return true;

}

int average() { // average pulse count
  int getL = getEncoderCount_left();
  int getR = getEncoderCount_right();
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}

void ChangeWheelSpeeds(int initialLeftSpd, int finalLeftSpd, int
initialRightSpd, int
                       finalRightSpd) {
  /*
    This function changes the car speed gradually from initial
    speed to final speed. This non-instantaneous speed change reduces the load
    on the plastic geartrain, and reduces the failure rate of the motors.
  */
  int diffLeft = finalLeftSpd - initialLeftSpd;
  int diffRight = finalRightSpd - initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft = abs(diffLeft) / stepIncrement;
  int numStepsRight = abs(diffRight) / stepIncrement;
  int numSteps = max(numStepsLeft, numStepsRight);
  int pwmLeftVal = initialLeftSpd; // initialize left wheel speed
  int pwmRightVal = initialRightSpd; // initialize right wheel speed
  int deltaLeft = (diffLeft) / numSteps; // left in(de)crement
  int deltaRight = (diffRight) / numSteps; // right in(de)crement
  for (int k = 0; k < numSteps; k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin, pwmLeftVal);
    analogWrite(right_pwm_pin, pwmRightVal);
    delay(60);
  } // end for int k
  if (finalLeftSpd == 0) analogWrite(left_pwm_pin, 0); ;
  if (finalRightSpd == 0) analogWrite(right_pwm_pin, 0);
} // end void ChangeWheelSpeeds


int sensorFusion() {
  for (int i = 0; i < 8; i++) {
    intSensorValues[i] = sensorValues[i];
    intSensorValues[i] -= minimumCal[i];// minimum removed
    intSensorValues[i] = (intSensorValues[i] * 1000) / maximumCal[i];
// scaling to 1000
  }

  //float ffinalSensorValue; // get this as a float
  int finalSensorValue = (-15 * intSensorValues[0] - 14 * intSensorValues[1] -
                       12 * intSensorValues[2] - 8*intSensorValues[3]
+ 8*intSensorValues[4]
                       + 12 * intSensorValues[5] + 14 *
intSensorValues[6] + 15 * intSensorValues[7]) / 4;
  return finalSensorValue;
}

void PID() {
  for (int i = 0; i < 8; i++) {
    prevSensorValues[i] = sensorValues[i];
  }
  ECE3_read_IR(sensorValues);

  previous = current;
  current = sensorFusion();

  //Serial.println(current);

  int PID_out = Kp * current + Kd * (current - previous);

  leftSpd = baseSpd - PID_out;
  rightSpd = baseSpd + PID_out;
  if (leftSpd < 25) {
    leftSpd = 25;
  }
  if (rightSpd > 75) {
    rightSpd = 75;
  }

  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(right_pwm_pin, rightSpd);
}

/*
int PID(int prev, int cur) {
  return cur * Kp + (cur - prev) * Kd;
}*/

///////////////////////////////////
void setup() {
  // put your setup code here, to run once:

  // initiate previous error to 0
  previous = 0;
  current = 0;

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);


  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);

  pinMode(LED_RF, OUTPUT);

  ECE3_Init();

  // set the data rate in bits/second for serial data transmission
  //Serial.begin(9600);

  resetEncoderCount_left();
  resetEncoderCount_right();

  digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);

  delay(2000); //Wait 2 seconds before starting

}

void loop() {
  // put your main code here, to run repeatedly:

  if (numStopped == 0) {
      // might delete later
      if (isStarted == false) {
        ChangeWheelSpeeds(0, baseSpd, 0, baseSpd); // start the car //
used to be wheelspd
        isStarted = true;
        PID();
        return;
      }
      if (!reachedEnd(prevSensorValues, sensorValues)) {
        PID();
      }
      else {
        ChangeWheelSpeeds(baseSpd, 0, baseSpd, 0); //Stop car // used
to be wheelspd
        numStopped +=1;
        resetEncoderCount_left();
        resetEncoderCount_right();
        digitalWrite(left_dir_pin,HIGH); // Set car direction to  backward
        while (getEncoderCount_left() < 350 && getEncoderCount_right() < 350) {
          analogWrite(left_pwm_pin,baseSpd);
          analogWrite(right_pwm_pin, baseSpd);
        }
        digitalWrite(left_dir_pin, LOW); // set left direction
        isStarted = false;
      }
  }

  else if (numStopped == 1) {
    if (isStarted == false) {
      //reset encoder count
        resetEncoderCount_left();
        resetEncoderCount_right();
        ChangeWheelSpeeds(0, baseSpd, 0, baseSpd); // start the car //
used to be wheelspd
        isStarted = true;
        PID();
        return;
      }
    else if (!reachedEnd(prevSensorValues, sensorValues)) {
       PID();
     } else {
      numStopped += 1;
     }
    }
  else{
    ChangeWheelSpeeds(baseSpd, 0, baseSpd, 0);
  }
}
