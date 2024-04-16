#include <Arduino.h>

// Motor control pins
const int leftWheelBack = 8;
const int leftWheelFront = 4;
const int rightWheelBack = 5;
const int rightWheelFront = 6;

// Ultrasonic sensor pins
const int TRIGGER_PIN = 7;
const int ECHO_PIN = 13;

// Gripper control pin
const int GRIPPER_PIN = 12;

// Distance threshold for object detection (in centimeters)
const float DISTANCE_THRESHOLD = 5.0;

// Sensor pins for line tracking
const int numSensors = 8;
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Threshold value to differentiate between black and white
const int threshold = 716;

// Base speed for the motors
const int baseSpeed = 150;
const int maxSpeed = 255;

// For averaging the sensor values
const int numReadings = 5;
int readings[numSensors][numReadings];
int readIndex = 0; 
int total[numSensors] = {0}; 
int average[numSensors] = {0}; 

void setup() {
  // Initialize motor control pins as outputs
  pinMode(leftWheelBack, OUTPUT);
  pinMode(leftWheelFront, OUTPUT);
  pinMode(rightWheelBack, OUTPUT);
  pinMode(rightWheelFront, OUTPUT);

  // Initialize sensor pins as inputs for line tracking
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
    for (int j = 0; j < numReadings; j++) {
      readings[i][j] = 0;
    }
  }

  // Initialize ultrasonic sensor pins
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize the gripper control pin
  pinMode(GRIPPER_PIN, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Line tracking 
  int position = readLinePosition();
  controlMotors(position);

  // Measure the distance to the nearest object
  float distance = measureDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  // Check if the object is within the distance threshold
  if (distance <= DISTANCE_THRESHOLD) {
    // Object detected within threshold, close the gripper
    digitalWrite(GRIPPER_PIN, HIGH);
    Serial.println("Gripper closed");
  } else {
    // No object within threshold, open the gripper
    digitalWrite(GRIPPER_PIN, LOW);
    Serial.println("Gripper opened");
  }

  // Delay for stability and sensor processing
  delay(100);
}

int readLinePosition() {
  int sumValue = 0;
  int numActive = 0;

  // Subtract the last reading:
  for (int i = 0; i < numSensors; i++) {
    total[i] -= readings[i][readIndex];
    readings[i][readIndex] = analogRead(sensorPins[i]);
    total[i] += readings[i][readIndex];
    average[i] = total[i] / numReadings;

    if (average[i] > threshold) {
      sumValue += (i - numSensors / 2) * 1000; // Weighted for position
      numActive++;
    }
  }
  readIndex = (readIndex + 1) % numReadings; // advance to the next index

  if (numActive > 0) {
    return sumValue / numActive;
  } else {
    return 0; // No line detected
  }
}

void controlMotors(int position) {
  int error = position;
  int leftSpeed = baseSpeed + error;
  int rightSpeed = baseSpeed - error;

  // Constrain the speeds to allowable PWM range
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  analogWrite(leftWheelFront, leftSpeed);
  analogWrite(rightWheelFront, rightSpeed);
  digitalWrite(leftWheelBack, LOW);
  digitalWrite(rightWheelBack, LOW);
}

float measureDistance() {
  // Send a 10 UltraSonic pulse to trigger the measurement
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Read the echo, which returns the time (in microseconds) taken for the pulse to return
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance (in cm) based on the speed of sound
  float distance = duration * 0.034 / 2;
  return distance;
}
