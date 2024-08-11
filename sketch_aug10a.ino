#include <Arduino.h>

// Encoder pins for each motor
const int encoderPin1A = 25;  // Motor 1 encoder A
const int encoderPin2A = 33;  // Motor 2 encoder A
const int encoderPin3A = 26;  // Motor 3 encoder A

// Motor PWM pins
const int motor1PWM = 12; // Motor 1 PWM pin
const int motor2PWM = 14; // Motor 2 PWM pin
const int motor3PWM = 27;  // Motor 3 PWM pin

// Motor En pins
const int motor1Input1 = 2; // Motor 1 input pin
const int motor1Input2 = 15; // Motor 1 input pin

const int motor2Input1 = 5; // Motor 2 input pin
const int motor2Input2 = 4; // Motor 2 input pin

const int motor3Input1 = 18; // Motor 3 input pin
const int motor3Input2 = 19; // Motor 3 input pin

// Variables to store the encoder counts
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount3 = 0;

// Variables to store the last encoder counts for speed calculation
long lastEncoderCount1 = 0;
long lastEncoderCount2 = 0;
long lastEncoderCount3 = 0;

// Time interval for speed calculation (in milliseconds)
const long interval = 100;

// Variables to store the motor speeds in RPM
float motorSpeed1 = 0.0;
float motorSpeed2 = 0.0;
float motorSpeed3 = 0.0;

// Motor parameters
const int pulsesPerRevolution = 830;  // Number of encoder pulses per revolution

// Current and target coordinates
float currentX = 0.0;
float currentY = 0.0;
float currentTheta = 0.0;

float targetX = 0;  // Target X-coordinate in cm
float targetY = 10;  // Target Y-coordinate in cm
float targetTheta = 0; // Target theta in degrees

// Velocity calculation based on distance and time
float distanceX, distanceY, distanceTheta;
float velocityX, velocityY, velocityTheta;
float timeX, timeY, timeTheta;

// Assuming the robot has three wheels (replace x_i and y_i with actual coordinates)
  float x1 = 0, wy1 = 13;  // Wheel 1 coordinates
  float x2 = 11.2, wy2 = -6.5; // Wheel 2 coordinates
  float x3 = -11.2, wy3 = -6.5; // Wheel 3 coordinates



  float phi1 = 0.0, phi2 = 120.0, phi3 = 240.0; // Replace with actual values in radians
  float r = 3.5 ;

// Interrupt service routines for each encoder
void IRAM_ATTR handleEncoder1() {
  encoderCount1++;
}

void IRAM_ATTR handleEncoder2() {
  encoderCount2++;
}

void IRAM_ATTR handleEncoder3() {
  encoderCount3++;
}

void setup() {
  Serial.begin(115200);
  
  // Set up motor pins as output
  pinMode(motor1Input1, OUTPUT);
  pinMode(motor1Input2, OUTPUT);

  pinMode(motor2Input1, OUTPUT);
  pinMode(motor2Input2, OUTPUT);

  pinMode(motor3Input1, OUTPUT);
  pinMode(motor3Input2, OUTPUT);

  // Set up motor direction (adjust as needed)
  digitalWrite(motor1Input1, LOW);
  digitalWrite(motor1Input2, HIGH);

  digitalWrite(motor2Input1, LOW);
  digitalWrite(motor2Input2, HIGH);

  digitalWrite(motor3Input1, LOW);
  digitalWrite(motor3Input2, HIGH);

  // Set up encoder pins as inputs
  pinMode(encoderPin1A, INPUT_PULLUP);
  pinMode(encoderPin2A, INPUT_PULLUP);
  pinMode(encoderPin3A, INPUT_PULLUP);

  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPin1A), handleEncoder1, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), handleEncoder2, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderPin3A), handleEncoder3, FALLING);

  // Set up motor PWM pins as outputs
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor3PWM, OUTPUT);

  // Set motors to maximum speed (adjust as needed)
  analogWrite(motor1PWM, 255);
  analogWrite(motor2PWM, 255);
  analogWrite(motor3PWM, 255);
}

void loop() {
  // Wait for the defined interval
  delay(interval);

  // Calculate motor speeds in RPM
  motorSpeed1 = ((encoderCount1 - lastEncoderCount1) * 60.0 * 1000.0) / (pulsesPerRevolution * interval);
  motorSpeed2 = ((encoderCount2 - lastEncoderCount2) * 60.0 * 1000.0) / (pulsesPerRevolution * interval);
  motorSpeed3 = ((encoderCount3 - lastEncoderCount3) * 60.0 * 1000.0) / (pulsesPerRevolution * interval);

  // Update last encoder counts
  lastEncoderCount1 = encoderCount1;
  lastEncoderCount2 = encoderCount2;
  lastEncoderCount3 = encoderCount3;

   // Calculate distances to move on each axis
  distanceX = targetX - currentX;
  distanceY = targetY - currentY;
  distanceTheta = targetTheta - currentTheta;

  // Calculate time required (5 cm per second)
  timeX = abs(distanceX) / 5.0;   // Time in seconds
  timeY = abs(distanceY) / 5.0;   // Time in seconds
  timeTheta = abs(distanceTheta) / 22.5;  // Time in seconds (assuming the same speed for theta)

  // Calculate velocities (distance/time)
  if(distanceX == 0){
    velocityX = 0;
  }
    else{
      velocityX = distanceX / timeX; 
    } // Velocity in cm/s
  
  if(distanceY == 0){
    velocityY = 0;
  }
    else{
    velocityY = distanceY / timeY;}  // Velocity in cm/s }

  if(timeTheta == 0){
    velocityTheta = 0;
  }
    else{
    velocityTheta = distanceTheta / timeTheta;}  // Velocity in degrees/s
  
  


  // Convert velocities from the space frame {S} to the robot frame {B}
  float v_x_B = cos(targetTheta) * velocityX + sin(targetTheta) * velocityY;
  float v_y_B = -sin(targetTheta) * velocityX + cos(targetTheta) * velocityY;

  // Calculate Beta_i for each wheel
  float beta1 = atan2(v_y_B - wy1 * velocityTheta, v_x_B - x1 * velocityTheta);
  float beta2 = atan2(v_y_B - wy2 * velocityTheta, v_x_B - x2 * velocityTheta);
  float beta3 = atan2(v_y_B - wy3 * velocityTheta, v_x_B - x3 * velocityTheta);

  float u1 = (1 / r) * ((cos(beta1) * wy1 + sin(beta1) * x1) * velocityTheta + (cos(beta1) * cos(phi1) - sin(beta1) * sin(phi1)) * velocityX + (cos(beta1) * sin(phi1) + sin(beta1) * cos(phi1)) * velocityY);
  float u2 = (1 / r) * ((cos(beta2) * wy2 + sin(beta2) * x2) * velocityTheta + (cos(beta2) * cos(phi2) - sin(beta2) * sin(phi2)) * velocityX + (cos(beta2) * sin(phi2) + sin(beta2) * cos(phi2)) * velocityY);
  float u3 = (1 / r) * ((cos(beta3) * wy3 + sin(beta3) * x3) * velocityTheta + (cos(beta3) * cos(phi3) - sin(beta3) * sin(phi3)) * velocityX + (cos(beta3) * sin(phi3) + sin(beta3) * cos(phi3)) * velocityY);

  // Print a header for the table
  Serial.println(F("-----------------------------------------------------"));
  Serial.println(F("|   Parameter      |   Value                        |"));
  Serial.println(F("-----------------------------------------------------"));

  // Print velocities U1, U2, U3
  Serial.print(F("| Velocity U1      | "));
  Serial.print(u1);
  Serial.println(F(" cm/s              |"));
  
  Serial.print(F("| Velocity U2      | "));
  Serial.print(u2);
  Serial.println(F(" cm/s              |"));
  
  Serial.print(F("| Velocity U3      | "));
  Serial.print(u3);
  Serial.println(F(" cm/s              |"));
  
  // Print a separator line
  Serial.println(F("-----------------------------------------------------"));
  
  // Print target and current positions
  Serial.print(F("| Target X         | "));
  Serial.print(targetX);
  Serial.println(F(" cm                |"));
  
  Serial.print(F("| Current X        | "));
  Serial.print(currentX);
  Serial.println(F(" cm                |"));
  
  Serial.print(F("| Target Y         | "));
  Serial.print(targetY);
  Serial.println(F(" cm                |"));
  
  Serial.print(F("| Current Y        | "));
  Serial.print(currentY);
  Serial.println(F(" cm                |"));
  
  // Print a separator line
  Serial.println(F("-----------------------------------------------------"));
  
  // Print distances
  Serial.print(F("| Distance X       | "));
  Serial.print(distanceX);
  Serial.println(F(" cm                |"));
  
  Serial.print(F("| Distance Y       | "));
  Serial.print(distanceY);
  Serial.println(F(" cm                |"));
  
  Serial.print(F("| Distance Theta   | "));
  Serial.print(distanceTheta);
  Serial.println(F(" degrees           |"));
  
  // Print a separator line
  Serial.println(F("-----------------------------------------------------"));
  
  // Print velocities and times for X, Y, and Theta
  Serial.print(F("| Velocity X       | "));
  Serial.print(velocityX);
  Serial.print(F(" cm/s              | Time X        | "));
  Serial.print(timeX);
  Serial.println(F(" s                |"));
  
  Serial.print(F("| Velocity Y       | "));
  Serial.print(velocityY);
  Serial.print(F(" cm/s              | Time Y        | "));
  Serial.print(timeY);
  Serial.println(F(" s                |"));
  
  Serial.print(F("| Velocity Theta   | "));
  Serial.print(velocityTheta);
  Serial.print(F(" degrees/s        | Time Theta    | "));
  Serial.print(timeTheta);
  Serial.println(F(" s                |"));
  
  // Print a footer line
  Serial.println(F("-----------------------------------------------------"));

    // Print velocities and times for X, Y, and Theta
  Serial.print(F("| Beta 1           | "));
  Serial.print(beta1);
  Serial.println(F(" rad               |               | "));
  
  Serial.print(F("| Beta 2           | "));
  Serial.print(beta2);
  Serial.println(F(" rad               |               | "));
  
  Serial.print(F("| Beta 3           | "));
  Serial.print(beta3);
  Serial.println(F(" rad               |               | "));
  
  // Print a footer line
  Serial.println(F("-----------------------------------------------------"));
  



  // Update current coordinates (for next iteration, if needed)
  /*currentX = targetX;
  currentY = targetY;
  currentTheta = targetTheta;*/

  // Wait for a bit before next calculation

  // Print motor speeds
  Serial.print("Motor 1 Speed (RPM): ");
  Serial.println(motorSpeed1);
  Serial.print("Motor 2 Speed (RPM): ");
  Serial.println(motorSpeed2);
  Serial.print("Motor 3 Speed (RPM): ");
  Serial.println(motorSpeed3);
}

