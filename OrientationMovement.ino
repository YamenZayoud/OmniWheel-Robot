#include <Arduino.h>
#include <math.h>

float pid3pwm, pid2pwm, pid1pwm,deltaTime,theta_dot,maxdistanceTheta =0;
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

float currentX = 0.0, currentY = 0.0, currentTheta = 1.57;
float targetX =10 , targetY=0 ;  // Default values

float distanceX, distanceY, distanceTheta =0;
float velocityX, velocityY, velocityTheta;
float timeX, timeY, timeTheta;

float v1, v2, v3;
float w1, w2, w3;
float U1, U2, U3;
float L = 13, r = 3.5;
float DeadTime=0;
float timeInterval,LastRunTime=0;

float Encoder_w1 = 0;
float Encoder_w2 = 0;
float Encoder_w3 = 0;
float Encoder_v1 = 0;
float Encoder_v2 = 0;
float Encoder_v3 = 0;
float EncodervelocityTheta = 0;

float const1 = 0.2925 *1.2  ;
float const2 = 0.325  *1.2  ;
float const3 = 0.3    *1.2  ;

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


 
 
  float currentMicros = 0;

  float motorRPM1 = (encoderCount1 - lastEncoderCount1) * 60.0 / (pulsesPerRevolution * 0.07);
  float motorRPM2 = (encoderCount2 - lastEncoderCount2) * 60.0 / (pulsesPerRevolution * 0.07);
  float motorRPM3 = (encoderCount3 - lastEncoderCount3) * 60.0 / (pulsesPerRevolution * 0.07);

  lastEncoderCount1 = encoderCount1;
  lastEncoderCount2 = encoderCount2;
  lastEncoderCount3 = encoderCount3;

  distanceX = targetX - currentX;
  distanceY = targetY - currentY;
  float targetTheta = atan2(distanceY, distanceX);  

  distanceTheta = targetTheta - currentTheta;
  float P_distanceTheta = abs(distanceTheta);

  if(P_distanceTheta > maxdistanceTheta){
     maxdistanceTheta = P_distanceTheta;
  }
  timeX = abs(distanceX) / 2.5;
  timeY = abs(distanceY) / 2.5;
  timeTheta = abs(distanceTheta) / 2.5;

  velocityX = (distanceX == 0) ? 0 : distanceX / timeX;
  velocityY = (distanceY == 0) ? 0 : distanceY / timeY;
  velocityTheta = (timeTheta == 0) ? 0 : distanceTheta / timeTheta;

  v1 = L * velocityTheta;
  v2 = L * velocityTheta;
  v3 = L * velocityTheta;

  w1 = v1 / r;
  w2 = v2 / r;
  w3 = v3 / r;


  float PWM_R = (float)(P_distanceTheta - 0) / (maxdistanceTheta - 0);

  U1 = w1 * 60/ (2 * PI);
  U2 = w2 * 60/ (2 * PI);
  U3 = w3 * 60/ (2 * PI);

  float PWM1 = map(U1, 0, 88, 0, 255) ;
  float PWM2 = map(U2, 0, 88, 0, 255) ;
  float PWM3 = map(U3, 0, 88, 0, 255) ;

  digitalWrite(motor1Input1, (U1 < 0) ? HIGH : LOW);
  digitalWrite(motor1Input2, (U1 < 0) ? LOW : HIGH);
  analogWrite(motor1PWM, abs((PWM1*PWM_R*const1)));

  digitalWrite(motor2Input1, (U2 < 0) ? HIGH : LOW);
  digitalWrite(motor2Input2, (U2 < 0) ? LOW : HIGH);
  analogWrite(motor2PWM, abs((PWM2*PWM_R*const2)));

  digitalWrite(motor3Input1, (U3 < 0) ? HIGH : LOW);
  digitalWrite(motor3Input2, (U3 < 0) ? LOW : HIGH);
  analogWrite(motor3PWM, abs((PWM3*PWM_R*const3)));



////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////

 Encoder_w1 = motorRPM1 * 2 * PI / 60;  
 Encoder_w2 = motorRPM2 * 2 * PI / 60;  
 Encoder_w3 = motorRPM3 * 2 * PI / 60;  

  if (velocityTheta<0){

    Encoder_w1 = -1*motorRPM1 * 2 * PI / 60;
    Encoder_w2 = -1*motorRPM2 * 2 * PI / 60;
    Encoder_w3 = -1*motorRPM3 * 2 * PI / 60;

    motorRPM1 = -1*motorRPM1;
    motorRPM2 = -1*motorRPM2;
    motorRPM3 = -1*motorRPM3;

  }

 Encoder_v1 = Encoder_w1 *r; 
 Encoder_v2 = Encoder_w2 *r; 
 Encoder_v3 = Encoder_w3 *r; 

    EncodervelocityTheta = (1.0 / (3.0 * L)) * (Encoder_v1 + Encoder_v2 + Encoder_v3);

     unsigned long currentTime = millis();
    
    unsigned long timeInterval = currentTime - LastRunTime;
    
    LastRunTime = currentTime;

    float current = (EncodervelocityTheta * timeInterval)/1000;
     currentTheta = current + currentTheta ;




  Serial.println(F("-----------------------------------------------------"));
  Serial.println(F("|   Parameter      |   Value                        |"));
  Serial.println(F("-----------------------------------------------------"));
  

  Serial.print(F("| maxdistanceTheta         | "));
  Serial.println(maxdistanceTheta);

  Serial.print(F("| current         | "));
  Serial.println(current);

  Serial.print(F("| PWM_R         | "));
  Serial.println(PWM_R);

  Serial.print(F("| Target X         | "));
  Serial.println(targetX);


  Serial.print(F("| Target Y         | "));
  Serial.println(targetY);
  Serial.println(F("---------------------------"));

  Serial.print(F("| currentTheta      | "));
  Serial.print(currentTheta);
  Serial.println(F(" rad              |"));

  Serial.print(F("| targetTheta      | "));
  Serial.print(targetTheta);
  Serial.println(F(" rad              |"));
  Serial.println(F("---------------------------"));


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
  Serial.println(F(" RPM               |"));

  Serial.print(F("| velocityTheta      | "));
  Serial.print(EncodervelocityTheta);
  Serial.println(F(" rad/s              |"));

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

  Serial.println(F("-----------------------------------------------------"));








}