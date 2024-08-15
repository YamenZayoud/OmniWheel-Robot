#include <Arduino.h>
#include <math.h>

float pid3pwm, pid2pwm, pid1pwm, deltaTime, theta_dot, maxdistanceX,maxdistanceY = 0;
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
float targetX = 0, targetY = 0;  // Default values

float distanceX, distanceY, distanceTheta = 0;
float velocityX, velocityY, velocityTheta = 0;
float E_velocityX, E_velocityY, E_velocityTheta = 0;
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
float const2 = 0.325 * 1.8;
float const3 = 0.3 * 2.5;

float rampFactor = 0.0;  // Initialize the ramp factor
float rampIncrement = 0.01;  // Adjust this value to control ramp speed (e.g., 0.01 for slow ramp)


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

  /*analogWrite(motor1PWM, 255);
  digitalWrite(motor1Input1, LOW);
  digitalWrite(motor1Input2, HIGH);
  analogWrite(motor2PWM, 255);
  digitalWrite(motor2Input1, LOW);
  digitalWrite(motor2Input2, HIGH);
  analogWrite(motor3PWM, 255);
  digitalWrite(motor3Input1, LOW);
  digitalWrite(motor3Input2, HIGH);*/


   Serial.println("Enter targetX and targetY:");
      while(true){
      if (Serial.available() > 0) {
        
        targetX = Serial.parseFloat();
        targetY = Serial.parseFloat();
        break;
      }}

  DeadTime = millis();  // Initialize the previous time
}

void loop() {


 float currentMicros = 0;

  float motorRPM1 = (encoderCount1 - lastEncoderCount1) * 60.0 / (pulsesPerRevolution * 0.07);
  float motorRPM2 = (encoderCount2 - lastEncoderCount2) * 60.0 / (pulsesPerRevolution * 0.07);
  float motorRPM3 = (encoderCount3 - lastEncoderCount3) * 60.0 / (pulsesPerRevolution * 0.07);

  lastEncoderCount1 = encoderCount1;
  lastEncoderCount2 = encoderCount2;
  lastEncoderCount3 = encoderCount3;




  distanceX = targetX - currentX;
  distanceY = targetY - currentY;
  float distance = abs(sqrt(pow(distanceX, 2) + pow(distanceY, 2)));
  float Positive_distanceX = abs(distanceX);
  float Positive_distanceY = abs(distanceY);

  if(Positive_distanceX > maxdistanceX){
     maxdistanceX = Positive_distanceX;
  }
  if(Positive_distanceY > maxdistanceY){
     maxdistanceY = Positive_distanceY;
  }

  float MaxDistance = sqrt(pow(maxdistanceX, 2) + pow(maxdistanceY, 2)); 



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

  float PWM_constant = (float)(distance - 0) / (MaxDistance - 0);


  U1 = w1 * 60 / (2 * PI);
  U2 = w2 * 60 / (2 * PI);
  U3 = w3 * 60 / (2 * PI);

  float PWM1 = map(U1, 0, 7, 0, 255);
  float PWM2 = map(U2, 0, 7, 0, 255);
  float PWM3 = map(U3, 0, 7, 0, 255);

   // Ensure PWM values don't exceed max PWM (255)
    PWM1 = min(PWM1, (float)255.0);
    PWM2 = min(PWM2, (float)255.0);
    PWM3 = min(PWM3, (float)255.0);

    // Ramp the PWM values for smooth start
    if (rampFactor < 1.0) {
        rampFactor += rampIncrement;  // Gradually increase the ramp factor
        rampFactor = min(rampFactor, (float)1.0);  // Cap ramp factor at 1.0
    }

    // Apply ramp factor to PWM values
    PWM1 *= rampFactor;
    PWM2 *= rampFactor;
    PWM3 *= rampFactor;

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
  analogWrite(motor1PWM, abs(PWM1*const1*PWM_constant));

  digitalWrite(motor2Input1, (U2 < 0) ? HIGH : LOW);
  digitalWrite(motor2Input2, (U2 < 0) ? LOW : HIGH);
  analogWrite(motor2PWM, abs(PWM2*const2*PWM_constant));

  digitalWrite(motor3Input1, (U3 < 0) ? HIGH : LOW);
  digitalWrite(motor3Input2, (U3 < 0) ? LOW : HIGH);
  analogWrite(motor3PWM, abs(PWM3*const3*PWM_constant));



  ////////////////////////////////////////////
  ////////////////////////////////////////////
  ////////////////////////////////////////////

 Encoder_w1 = motorRPM1 * 2 * PI / 60;  // rad/s
 Encoder_w2 = motorRPM2 * 2 * PI / 60;  // rad/s
 Encoder_w3 = motorRPM3 * 2 * PI / 60;  // rad/s

  if (velocityTheta<0){

    Encoder_w1 = -1*motorRPM1 * 2 * PI / 60;  // rad/s
    Encoder_w2 = -1*motorRPM2 * 2 * PI / 60;  // rad/s
    Encoder_w3 = -1*motorRPM3 * 2 * PI / 60;  // rad/s

    motorRPM1 = -1*motorRPM1;
    motorRPM2 = -1*motorRPM2;
    motorRPM3 = -1*motorRPM3;

  }

 Encoder_v1 = Encoder_w1 *r; // con
 Encoder_v2 = Encoder_w2 *r; // con
 Encoder_v3 = Encoder_w3 *r; // con

      // Calculate linear velocity in the x direction
    E_velocityX = (2.0 / (3.0 * L)) * (cos(currentTheta + M_PI / 3.0) * Encoder_v1 + cos(M_PI / 3.0 - currentTheta) * Encoder_v2 - cos(currentTheta) * Encoder_v3);

    // Calculate linear velocity in the y direction
    E_velocityY = (2.0 / (3.0 * L)) * (sin(currentTheta + M_PI / 3.0) * Encoder_v1 - sin(M_PI / 3.0 - currentTheta) * Encoder_v2 - sin(currentTheta) * Encoder_v3);

    // Calculate angular velocity
    E_velocityTheta = (1.0 / (3.0 * L)) * (Encoder_v1 + Encoder_v2 + Encoder_v3);

     unsigned long currentTime = millis();
    
    // Calculate the time elapsed since the last loop iteration
    unsigned long timeInterval = currentTime - LastRunTime;
    
    // Update LastRunTime for the next iteration
    LastRunTime = currentTime;
    // Integrate theta_dot to find theta

    float angle = (E_velocityTheta * timeInterval)/1000;
    float distance_X = (E_velocityX * timeInterval)/1000;
    float distance_Y = (E_velocityY * timeInterval)/1000;
    currentTheta = angle + currentTheta ;
    currentX = distance_X + currentX ;
    currentY = distance_Y + currentY ;




  Serial.println(F("-------------------------------------------------------------"));
  Serial.println(F("|   Parameter            |   Value                         |"));
  Serial.println(F("-------------------------------------------------------------"));

  // PWM Constant
  Serial.print(F("| PWM_constant           | "));
  Serial.print(PWM_constant);
  Serial.println(F("                            |"));

  // Target X
  Serial.print(F("| Target X               | "));
  Serial.print(targetX);
  Serial.println(F(" cm                        |"));

  // Target Y
  Serial.print(F("| Target Y               | "));
  Serial.print(targetY);
  Serial.println(F(" cm                        |"));

  // Current X
  Serial.print(F("| currentX               | "));
  Serial.print(currentX);
  Serial.println(F(" cm                        |"));

  // Current Y
  Serial.print(F("| currentY               | "));
  Serial.print(currentY);
  Serial.println(F(" cm                        |"));

  // Current Theta
  Serial.print(F("| currentTheta           | "));
  Serial.print(currentTheta);
  Serial.println(F(" rad                      |"));

  // Distance
  Serial.print(F("| Distance               | "));
  Serial.print(distance);
  Serial.println(F(" cm                        |"));

  // Max Distance
  Serial.print(F("| MaxDistance            | "));
  Serial.print(MaxDistance);
  Serial.println(F(" cm                        |"));

  // Velocity U1
  Serial.print(F("| Velocity U1            | "));
  Serial.print(U1);
  Serial.println(F(" RPM                       |"));

  // Motor 1 RPM
  Serial.print(F("| Motor 1 RPM            | "));
  Serial.print(motorRPM1);
  Serial.println(F(" RPM                       |"));

  // Velocity U2
  Serial.print(F("| Velocity U2            | "));
  Serial.print(U2);
  Serial.println(F(" RPM                       |"));

  // Motor 2 RPM
  Serial.print(F("| Motor 2 RPM            | "));
  Serial.print(motorRPM2);
  Serial.println(F(" RPM                       |"));

  // Velocity U3
  Serial.print(F("| Velocity U3            | "));
  Serial.print(U3);
  Serial.println(F(" RPM                       |"));

  // Motor 3 RPM
  Serial.print(F("| Motor 3 RPM            | "));
  Serial.print(motorRPM3);
  Serial.println(F(" RPM                       |"));

  // E_velocityTheta
  Serial.print(F("| E_velocityTheta        | "));
  Serial.print(E_velocityTheta);
  Serial.println(F(" rad/s                    |"));

  // velocityTheta
  Serial.print(F("| velocityTheta          | "));
  Serial.print(velocityTheta);
  Serial.println(F(" rad/s                    |"));

  // E_velocityX
  Serial.print(F("| E_velocityX            | "));
  Serial.print(E_velocityX);
  Serial.println(F(" cm/s                     |"));

  // velocityX
  Serial.print(F("| velocityX              | "));
  Serial.print(velocityX);
  Serial.println(F(" cm/s                     |"));

  // E_velocityY
  Serial.print(F("| E_velocityY            | "));
  Serial.print(E_velocityY);
  Serial.println(F(" cm/s                     |"));

  // velocityY
  Serial.print(F("| velocityY              | "));
  Serial.print(velocityY);
  Serial.println(F(" cm/s                     |"));

  Serial.println(F("-------------------------------------------------------------"));

}