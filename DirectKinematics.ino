#include <Arduino.h>
#include <math.h>

float pid3pwm, pid2pwm, pid1pwm, deltaTime, theta_dot, maxdistanceTheta = 0;
int xxx = 0;
float Kp1 = 3.2, Ki1 = 2.2 / 100, Kd1 = 2.2 / 1000;
float Kp2 = 3.05, Ki2 = 2.3 / 100, Kd2 = 2.3 / 1000;
float Kp3 = 3.5, Ki3 = 2.5 / 100, Kd3 = 2.5 / 1000;

float pid1, pid2, pid3;
int dirM1, dirM2, dirM3;

float integral1 = 0, integral2 = 0, integral3 = 0;
float lastError1 = 0, lastError2 = 0, lastError3 = 0;
float lastMicros1, lastMicros2, lastMicros3, dt1, dt2, dt3;
float maxVelocity = 18.0;  //rad/sec

const int encoderPin1A = 25;
const int encoderPin2A = 26;
const int encoderPin3A = 33;

const int motor1PWM = 12;
const int motor2PWM = 27;
const int motor3PWM = 14;

const int motor1Input2 = 15;
const int motor1Input1 = 2;

const int motor3Input2 = 4;
const int motor3Input1 = 5;

const int motor2Input2 = 19;
const int motor2Input1 = 18;

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount3 = 0;

long lastEncoderCount1 = 0;
long lastEncoderCount2 = 0;
long lastEncoderCount3 = 0;

const int pulsesPerRevolution = 830;

float currentX = 0.0, currentY = 0.0, currentTheta = 1.57;
float targetX = 60, targetY = 20;  // Default values

float distanceX, distanceY, distanceTheta = 0;
float velocityX, velocityY, velocityTheta = 0;
float timeX, timeY, timeTheta;

float v1, v2, v3;
float w1, w2, w3;
float U1, U2, U3;
float L = 13, r = 3.5;
float DeadTime = 0;
float timeInterval, LastRunTime = 0;

float Encoder_w1 = 0;
float Encoder_w2 = 0;
float Encoder_w3 = 0;
float Encoder_v1 = 0;
float Encoder_v2 = 0;
float Encoder_v3 = 0;
float EncodervelocityTheta = 0;

float const1 = 0.2925 * 2.5;
float const2 = 0.325 * 2.5;
float const3 = 0.3 * 2.5;

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

  pinMode(motor1Input1, OUTPUT);
  pinMode(motor1Input2, OUTPUT);
  pinMode(motor2Input1, OUTPUT);
  pinMode(motor2Input2, OUTPUT);
  pinMode(motor3Input1, OUTPUT);
  pinMode(motor3Input2, OUTPUT);

  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor3PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPin1A), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), encoderISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin3A), encoderISR3, RISING);



  DeadTime = millis();
}

void loop() {






distanceX = targetX - currentX;
distanceY = targetY - currentY;
float targetTheta = atan2(distanceY, distanceX);

distanceTheta = targetTheta - currentTheta;

timeX = abs(distanceX) / 2.5;
timeY = abs(distanceY) / 2.5;
timeTheta = abs(distanceTheta) / 2.5;

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

  if (distanceTheta < 0.01) {
    velocityTheta = 0;
  } else {
    velocityTheta = distanceTheta / timeTheta;
  }  // Velocity in degrees/s

  v1 = -sin(currentTheta) * velocityX + cos(currentTheta) * velocityY + L * velocityTheta;
  v2 = -sin((PI / 3) - currentTheta) * velocityX - cos((PI / 3) - currentTheta) * velocityY + L * velocityTheta;
  v3 = +sin((PI / 3) + currentTheta) * velocityX - cos((PI / 3) + currentTheta) * velocityY + L * velocityTheta;

  w1 = v1 / r;
  w2 = v2 / r;
  w3 = v3 / r;



  U1 = w1 * 60 / (2 * PI);
  U2 = w2 * 60 / (2 * PI);
  U3 = w3 * 60 / (2 * PI);

  float PWM1 = map(U1, 0, 7, 0, 255);
  float PWM2 = map(U2, 0, 7, 0, 255);
  float PWM3 = map(U3, 0, 7, 0, 255);

  if (PWM1 > 255) {
    PWM1 = 255;
  }
  if (PWM2 > 255) {
    PWM2 = 255;
  }
  if (PWM3 > 255) {
    PWM3 = 255;
  }

  digitalWrite(motor1Input1, (U1 < 0) ? HIGH : LOW);
  digitalWrite(motor1Input2, (U1 < 0) ? LOW : HIGH);
  analogWrite(motor1PWM, abs(PWM1*const1));

  digitalWrite(motor2Input1, (U2 < 0) ? HIGH : LOW);
  digitalWrite(motor2Input2, (U2 < 0) ? LOW : HIGH);
  analogWrite(motor2PWM, abs(PWM2*const2));

  digitalWrite(motor3Input1, (U3 < 0) ? HIGH : LOW);
  digitalWrite(motor3Input2, (U3 < 0) ? LOW : HIGH);
  analogWrite(motor3PWM, abs(PWM3*const3));



 




  Serial.println(F("-----------------------------------------------------"));
  Serial.println(F("|   Parameter      |   Value                        |"));
  Serial.println(F("-----------------------------------------------------"));


  Serial.print(F("| maxdistanceTheta         | "));
  Serial.println(maxdistanceTheta);

  Serial.print(F("| Target X         | "));
  Serial.println(targetX);


  Serial.print(F("| Target Y         | "));
  Serial.println(targetY);
  Serial.println(F("---------------------------"));


  Serial.print(F("| targetTheta      | "));
  Serial.print(targetTheta);
  Serial.println(F(" rad              |"));
  Serial.println(F("---------------------------"));


  Serial.print(F("| Velocity U1      | "));
  Serial.print(U1);
  Serial.println(F(" RPM               |"));

  Serial.print(F("| Motor 1 PWM1:     | "));
  Serial.println(PWM1);

  Serial.print(F("| Velocity U2      | "));
  Serial.print(U2);
  Serial.println(F(" RPM               |"));

  Serial.print(F("| Motor 2 PWM2:     | "));
  Serial.println(PWM2);

  Serial.print(F("| Velocity U3      | "));
  Serial.print(U3);
  Serial.println(F(" RPM               |"));

  Serial.print(F("| Motor 3 PWM3:     | "));
  Serial.println(PWM3);
  Serial.println(F(" RPM              |"));



  Serial.println(F("---------------------------"));


  Serial.print(F("| distanceX        | "));
  Serial.print(distanceX);
  Serial.println(F(" cm               |"));

  Serial.print(F("| distanceY        | "));
  Serial.print(distanceY);
  Serial.println(F(" cm               |"));

  Serial.print(F("| distanceTheta    | "));
  Serial.print(distanceTheta);
  Serial.println(F(" rad              |"));
  Serial.println(F("---------------------------"));

  Serial.print(F("| velocityX        | "));
  Serial.print(velocityX);
  Serial.println(F(" cm/s             |"));

  Serial.print(F("| velocityY        | "));
  Serial.print(velocityY);
  Serial.println(F(" cm/s             |"));

  Serial.print(F("| velocityTheta    | "));
  Serial.print(velocityTheta);
  Serial.println(F(" rad/s            |"));

  Serial.print(F("| distanceTheta    | "));
  Serial.print(distanceTheta);
  Serial.println(F(" rad            |"));

  Serial.println(F("-----------------------------------------------------"));
}
