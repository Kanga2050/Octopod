#include <ESP32Servo.h>

// Create a servo object
Servo myServo;

// Define the servo pin
const int servoPin = 18;  // GPIO 18 on ESP32

// Variables for servo control
int pos = 0;              // Variable to store the servo position
int minAngle = 0;         // Minimum servo angle
int maxAngle = 180;       // Maximum servo angle
int delayTime = 15;       // Delay between movements (milliseconds)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Allow allocation of all timers for servo library
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Attach the servo to the pin with min/max pulse widths
  myServo.setPeriodHertz(50);    // Standard 50hz servo
  myServo.attach(servoPin, 500, 2400);  // Attach servo with pulse width range
  
  Serial.println("ESP32 Servo Control Initialized");
  Serial.println("Servo will sweep from 0 to 180 degrees");
}

void loop() {
  // Sweep from 0 to 180 degrees
  for (pos = minAngle; pos <= maxAngle; pos += 1) {
    myServo.write(pos);              // Tell servo to go to position in variable 'pos'
    delay(delayTime);                // Wait for the servo to reach the position
  }
  
  // Sweep from 180 to 0 degrees
  for (pos = maxAngle; pos >= minAngle; pos -= 1) {
    myServo.write(pos);              // Tell servo to go to position in variable 'pos'
    delay(delayTime);                // Wait for the servo to reach the position
  }
}

// Additional functions for manual servo control

// Function to move servo to a specific angle
void moveServoToAngle(int angle) {
  if (angle >= minAngle && angle <= maxAngle) {
    myServo.write(angle);
    Serial.print("Servo moved to: ");
    Serial.print(angle);
    Serial.println(" degrees");
  } else {
    Serial.println("Angle out of range (0-180)");
  }
}

// Function to move servo smoothly to a target angle
void smoothMoveServo(int targetAngle, int stepDelay = 15) {
  int currentPos = myServo.read();
  
  if (targetAngle > currentPos) {
    // Move clockwise
    for (int pos = currentPos; pos <= targetAngle; pos++) {
      myServo.write(pos);
      delay(stepDelay);
    }
  } else {
    // Move counter-clockwise
    for (int pos = currentPos; pos >= targetAngle; pos--) {
      myServo.write(pos);
      delay(stepDelay);
    }
  }
}

// Function to calibrate servo (find actual min/max positions)
void calibrateServo() {
  Serial.println("Calibrating servo...");
  
  // Test minimum position
  myServo.write(0);
  delay(1000);
  Serial.println("Servo at 0 degrees");
  
  // Test maximum position
  myServo.write(180);
  delay(1000);
  Serial.println("Servo at 180 degrees");
  
  // Return to center
  myServo.write(90);
  delay(1000);
  Serial.println("Servo at 90 degrees (center)");
}
