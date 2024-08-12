#include <Arduino.h>
#include <math.h>

float pid3pwm, pid2pwm, pid1pwm;
int xxx = 0;
float Kp1 = 3.2, Ki1 = 2.2 / 100, Kd1 = 2.2 / 1000;
float Kp2 = 3.05, Ki2 = 2.3 / 100, Kd2 = 2.3 / 1000;
float Kp3 = 3.5, Ki3 = 2.5 / 100, Kd3 = 2.5 / 1000;

float pid1, pid2, pid3;
int dirM1, dirM2, dirM3;

float integral1 = 0, integral2 = 0, integral3 = 0;
float lastError1 = 0, lastError2 = 0, lastError3 = 0;
float lastMicros1, lastMicros2, lastMicros3, dt1, dt2, dt3;
float maxVelocity = 18.0;//rad/sec

const int encoderPin1A = 25;  
const int encoderPin2A = 33;  
const int encoderPin3A = 26;  

const int motor1PWM = 12;  
const int motor2PWM = 14;  
const int motor3PWM = 27;  

const int motor1Input1 = 2;   
const int motor1Input2 = 15;  

const int motor2Input1 = 5;  
const int motor2Input2 = 4;  

const int motor3Input1 = 18;  
const int motor3Input2 = 19;  

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount3 = 0;

long lastEncoderCount1 = 0;
long lastEncoderCount2 = 0;
long lastEncoderCount3 = 0;

const int pulsesPerRevolution = 830;

float currentX = 0.0, currentY = 0.0, currentTheta = PI / 2;
float targetX = -10.0, targetY = 10;

float distanceX, distanceY, distanceTheta;
float velocityX, velocityY, velocityTheta;
float timeX, timeY, timeTheta;

float v1, v2, v3;
float w1, w2, w3;
float U1, U2, U3;
float L = 13, r = 3.5;

void IRAM_ATTR encoderISR1() {
  encoderCount1++;
}

void IRAM_ATTR encoderISR2() {
  encoderCount2++;
}

void IRAM_ATTR encoderISR3() {
  encoderCount3++;
}

void setup() {
  Serial.begin(115200);
 pinMode(encoderPin1A, INPUT_PULLUP);
 pinMode(encoderPin2A, INPUT_PULLUP);
 pinMode(encoderPin3A, INPUT_PULLUP);
  // Set up motor pins as output
  pinMode(motor1Input1, OUTPUT);
  pinMode(motor1Input2, OUTPUT);

  pinMode(motor2Input1, OUTPUT);
  pinMode(motor2Input2, OUTPUT);

  pinMode(motor3Input1, OUTPUT);
  pinMode(motor3Input2, OUTPUT);

  // Set up motor direction (adjust as needed)



  // Set up motor PWM pins as outputs
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor3PWM, OUTPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encoderPin1A), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), encoderISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin3A), encoderISR3, RISING);

analogWrite(motor1PWM, 255);
digitalWrite(motor1Input1, LOW);
digitalWrite(motor1Input2, HIGH);
analogWrite(motor2PWM, 255);
digitalWrite(motor2Input1, LOW);
digitalWrite(motor2Input2, HIGH);
analogWrite(motor3PWM, 255);
digitalWrite(motor3Input1, LOW);
digitalWrite(motor3Input2, HIGH);


      lastMicros1 = micros();
      lastMicros2 = micros();
      lastMicros3 = micros();

        // Calculate distances to move on each axis
  
}
void loop() {


  unsigned long currentMicros = micros();

  // Calculate time intervals in seconds
  float timeElapsed1 = (currentMicros - lastMicros1) / 1000000.0;
  float timeElapsed2 = (currentMicros - lastMicros2) / 1000000.0;
  float timeElapsed3 = (currentMicros - lastMicros3) / 1000000.0;

  // Calculate RPM for each motor
  float motorRPM1 = (encoderCount1 - lastEncoderCount1) * 60.0 / (pulsesPerRevolution * timeElapsed1);
  float motorRPM2 = (encoderCount2 - lastEncoderCount2) * 60.0 / (pulsesPerRevolution * timeElapsed2);
  float motorRPM3 = (encoderCount3 - lastEncoderCount3) * 60.0 / (pulsesPerRevolution * timeElapsed3);

  // Save the current encoder count and time for the next calculation
  lastEncoderCount1 = encoderCount1;
  lastEncoderCount2 = encoderCount2;
  lastEncoderCount3 = encoderCount3;

  lastMicros1 = currentMicros;
  lastMicros2 = currentMicros;
  lastMicros3 = currentMicros;




distanceX = targetX - currentX;
  distanceY = targetY - currentY;
  float targetTheta = atan2(distanceY, distanceX);  ////////HERE IN RAD
  distanceTheta = targetTheta - currentTheta;

  // Calculate time required (5 cm per second)
  timeX = abs(distanceX) / 2.5;          // Time in seconds
  timeY = abs(distanceY) / 2.5;          // Time in seconds
  timeTheta = abs(distanceTheta) / 2.5;  // Time in seconds (assuming the same speed for theta)

  // Calculate velocities (distance/time)
  if (distanceX == 0) {
    velocityX = 0;
  } else {
    velocityX = distanceX / timeX;
  }  // Velocity in cm/s

  if (distanceY == 0) {
    velocityY = 0;
  } else {
    velocityY = distanceY / timeY;
  }  // Velocity in cm/s }

  if (timeTheta == 0) {
    velocityTheta = 0;
  } else {
    velocityTheta = distanceTheta / timeTheta;
  }  // Velocity in degrees/s




  /////////////////Step 1//////////////////
  /////////// Correct Rotation ////////////
  /////////////////////////////////////////


  v1 = L * velocityTheta;
  v2 = L * velocityTheta;
  v3 = L * velocityTheta;

  w1 = v1 / r;
  w2 = v2 / r;
  w3 = v3 / r;

  U1 = w1 * 60 / (2 * PI);  //RPM
  U2 = w2 * 60 / (2 * PI);  //RPM
  U3 = w2 * 60 / (2 * PI);  //RPM

  /////////////////Step 1///////////////////
  //////////////////////////////////////////
  ///////////////// End ////////////////////
  // PID control for Motor 1
  if (micros() - lastMicros1 >= 800000)
   {
    dt1 = (micros() - lastMicros1) * 1.0e-6;
    lastMicros1 = micros();
    float L_w_encoder1 = dirM1 * ((2.0 * PI) * float(encoderCount1) / pulsesPerRevolution) / dt1;
    encoderCount1 = 0;
    float error1 = w1 - L_w_encoder1;
    integral1 += error1;
    float derivative1 = (error1 - lastError1) / dt1;
    lastError1 = error1;
    pid1 += (error1 * Kp1 + Ki1 * integral1 * dt1 + derivative1 * Kd1);
    
    if (pid1 < -maxVelocity) {
      pid1 = -maxVelocity;

    } else if (pid1 > maxVelocity) {
      pid1 = maxVelocity;
    }
    pid1pwm = (255.0 * pid1 / maxVelocity);
    if (pid1pwm < 0) {
      dirM1 = -1;
      digitalWrite(motor1Input1, LOW);
      digitalWrite(motor1Input2, HIGH);
    } else {
      dirM2 = 1;
      digitalWrite(motor1Input1, HIGH);
      digitalWrite(motor1Input2, LOW);
    }
    analogWrite(motor1PWM, abs(pid1pwm));
  }

  // PID control for Motor 2
  if (micros() - lastMicros2 >= 800000) {
    dt2 = (micros() - lastMicros2) * 1.0e-6;
    lastMicros2 = micros();
    float L_w_encoder2 = dirM2 * ((2.0 * PI) * float(encoderCount2) / pulsesPerRevolution) / dt2;
    encoderCount2 = 0;
    float error2 =( w2 - L_w_encoder2);
    integral2 += error2;
    float derivative2 = (error2 - lastError2) / dt2;
    lastError2 = error2;
     pid2 += (error2 * Kp2 + Ki2 * integral2 * dt2 + derivative2 * Kd3);

    if (pid2 < -maxVelocity) {
      pid2 = -maxVelocity;

    } else if (pid2 > maxVelocity) {
      pid2 = maxVelocity;
    }
     pid2pwm = (255.0 * pid2 / maxVelocity);

    if (pid2pwm < 0) {
      dirM2 = -1;
      digitalWrite(motor2Input1, LOW);
      digitalWrite(motor2Input2, HIGH);
    } else {
      dirM2 = 1;
      digitalWrite(motor2Input1, HIGH);
      digitalWrite(motor2Input2, LOW);
    }
    analogWrite(motor2PWM, abs(pid2pwm));
  }

  // PID control for Motor 3
  if (micros() - lastMicros3 >= 800000) {
    dt3 = (micros() - lastMicros3) * 1.0e-6;
    lastMicros3 = micros();
    float L_w_encoder3 = dirM3 * ((2.0 * PI) * float(encoderCount3) / pulsesPerRevolution) / dt3;
    encoderCount3 = 0;
    float error3 = w3 - L_w_encoder3;
    integral3 += error3;
    float derivative3 = (error3 - lastError3) / dt3;
    lastError3 = error3;
    pid3 += (error3 * Kp3 + Ki3 * integral3 * dt3 + derivative3 * Kd3);

     
    if (pid3 < -maxVelocity) {
      pid3 = -maxVelocity;

    } else if (pid3 > maxVelocity) {
      pid3 = maxVelocity;
    }
    pid3pwm = (255.0 * pid3 / maxVelocity);

    if (pid3pwm < 0) {
      dirM3 = -1;
      digitalWrite(motor3Input1, LOW);
      digitalWrite(motor3Input2, HIGH);
    } else {
      dirM3 = 1;
      digitalWrite(motor3Input1, HIGH);
      digitalWrite(motor3Input2, LOW);
    }
    analogWrite(motor3PWM, abs(pid3pwm));
  }



  // Print a header for the table
  Serial.println(F("-----------------------------------------------------"));
  Serial.println(F("|   Parameter      |   Value                        |"));
  Serial.println(F("-----------------------------------------------------"));

Serial.print("timeElapsed1: ");
  Serial.println(timeElapsed1);

  Serial.print("pid2pwm: ");
  Serial.println(pid2pwm);

  Serial.print("pid3pwm: ");
  Serial.println(pid3pwm);

  Serial.print(F("| Velocity U1      | "));
  Serial.print(U1);
  Serial.println(F(" RPM               |"));

  Serial.print(F("| Motor 1 RPM:     | "));
  Serial.println(motorRPM1);

  Serial.print(F("| Velocity U2      | "));
  Serial.print(U2);
  Serial.println(F(" RPM               |"));

  Serial.print(F("| Motor 2 RPM:     | "));
  Serial.println(motorRPM2);
  

  Serial.print(F("| Velocity U3      | "));
  Serial.print(U3);
  Serial.println(F(" RPM               |"));

  Serial.print(F("| Motor 3 RPM:     | "));
  Serial.println(motorRPM3);

  Serial.print(F("| distanceX        | "));
  Serial.print(distanceX);
  Serial.println(F(" cm               |"));

  Serial.print(F("| distanceY        | "));
  Serial.print(distanceY);
  Serial.println(F(" cm               |"));

  Serial.print(F("| targetTheta      | "));
  Serial.print(targetTheta);
  Serial.println(F(" rad              |"));
  
  Serial.print(F("| distanceTheta    | "));
  Serial.print(distanceTheta);
  Serial.println(F(" rad              |"));

  Serial.print(F("| velocityX        | "));
  Serial.print(velocityX);
  Serial.println(F(" cm/s             |"));

  Serial.print(F("| velocityY        | "));
  Serial.print(velocityY);
  Serial.println(F(" cm/s             |"));

  Serial.print(F("| velocityTheta    | "));
  Serial.print(velocityTheta);
  Serial.println(F(" rad/s            |"));


  
  
}