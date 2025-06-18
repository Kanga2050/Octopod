#include <ESP32Servo.h>
#include <math.h>

// Include servo control functions from servo_control.ino
// (You can copy the helper functions from servo_control.ino or include it)

// Create servo objects for the 3-DOF leg
Servo baseServo;    // Servo 1: Base rotation (Z-axis rotation)
Servo shoulderServo; // Servo 2: Shoulder joint (X-axis rotation)
Servo elbowServo;   // Servo 3: Elbow joint (X-axis rotation)

// Define servo pins
const int basePin = 18;     // GPIO 18 for base servo
const int shoulderPin = 19; // GPIO 19 for shoulder servo
const int elbowPin = 21;    // GPIO 21 for elbow servo

// Leg segment lengths (in mm)
const float L1 = 60.0;    // Distance from base to shoulder joint
const float L2 = 90.0;    // Distance from shoulder to elbow joint
const float L3 = 154.6;   // Distance from elbow to endpoint
// Servo angle limits (in degrees)
const int baseMin = 0, baseMax = 180;
const int shoulderMin = 0, shoulderMax = 180;
const int elbowMin = 0, elbowMax = 180;

// Current leg position
float currentX = 0, currentY = 0, currentZ = 0;

// Current servo angles
float currentBaseAngle = 90;
float currentShoulderAngle = 90;
float currentElbowAngle = 90;

// Structure to hold joint angles
struct JointAngles {
  float base;     // Base rotation angle (degrees)
  float shoulder; // Shoulder angle (degrees)
  float elbow;    // Elbow angle (degrees)
  bool valid;     // Whether the solution is valid
};

// Structure for interpolated movement
struct InterpolationData {
  float startBase, targetBase;
  float startShoulder, targetShoulder;
  float startElbow, targetElbow;
  unsigned long startTime;
  unsigned long duration;
  bool active;
};

void setup() {
  Serial.begin(115200);
  
  // Initialize PWM timers for servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Attach servos with appropriate pulse width ranges
  baseServo.setPeriodHertz(50);
  baseServo.attach(basePin, 500, 2400);
  
  shoulderServo.setPeriodHertz(50);
  shoulderServo.attach(shoulderPin, 500, 2400);
  
  elbowServo.setPeriodHertz(50);
  elbowServo.attach(elbowPin, 500, 2400);
    Serial.println("3-DOF Leg Inverse Kinematics Initialized");
  Serial.println("Leg segments: L1=60mm, L2=90mm, L3=154.6mm");
  
  // Initialize current servo positions
  currentBaseAngle = 90;
  currentShoulderAngle = 90;
  currentElbowAngle = 90;
  
  // Set initial servo positions
  baseServo.write(currentBaseAngle);
  shoulderServo.write(currentShoulderAngle);
  elbowServo.write(currentElbowAngle);
  
  delay(1000);
  
  // Move to home position
  moveToPosition(0, 100, -200, 2000);
  delay(3000);
  
  // Demo movement
  demonstrateLegMovement();
}

void loop() {
  // Update servo interpolation
  updateServoInterpolation();
  
  // Continuous leg movement pattern
  walkingPattern();
  delay(10); // Reduced delay for smoother interpolation
}

// Main inverse kinematics function
JointAngles inverseKinematics(float x, float y, float z) {
  JointAngles angles;
  angles.valid = false;
  
  // Calculate base rotation angle (rotation around Z-axis)
  angles.base = atan2(y, x) * 180.0 / PI;
  
  // Calculate distance from base to target in XY plane
  float rho = sqrt(x*x + y*y);
  
  // Adjust for the offset from base to shoulder joint
  float adjustedRho = rho - L1;
  
  // Calculate distance from shoulder to target
  float distance = sqrt(adjustedRho*adjustedRho + z*z);
  
  // Check if target is reachable
  if (distance > (L2 + L3) || distance < abs(L2 - L3)) {
    Serial.println("Target unreachable!");
    return angles;
  }
  
  // Calculate elbow angle using law of cosines
  float cosElbow = (L2*L2 + L3*L3 - distance*distance) / (2*L2*L3);
  
  // Clamp cosine value to valid range to avoid numerical errors
  cosElbow = constrain(cosElbow, -1.0, 1.0);
  
  // Elbow angle (measured from straight line)
  float elbowRad = acos(cosElbow);
  angles.elbow = 180.0 - (elbowRad * 180.0 / PI);
  
  // Calculate shoulder angle
  float alpha = atan2(z, adjustedRho);
  float beta = acos((L2*L2 + distance*distance - L3*L3) / (2*L2*distance));
  
  // Shoulder angle (measured from horizontal)
  angles.shoulder = (alpha + beta) * 180.0 / PI;
  
  // Adjust angles to servo coordinate system (0-180 degrees)
  angles.base = constrain(angles.base + 90, baseMin, baseMax);
  angles.shoulder = constrain(90 - angles.shoulder, shoulderMin, shoulderMax);
  angles.elbow = constrain(angles.elbow, elbowMin, elbowMax);
  
  angles.valid = true;
  return angles;
}

// Move leg to specified position with interpolation
bool moveToPosition(float x, float y, float z, unsigned long duration = 1000) {
  JointAngles angles = inverseKinematics(x, y, z);
  
  if (!angles.valid) {
    Serial.println("Invalid position requested");
    return false;
  }
  
  // Start interpolated movement to new angles
  startServoInterpolation(angles.base, angles.shoulder, angles.elbow, duration);
  
  // Update current position
  currentX = x;
  currentY = y;
  currentZ = z;
  
  Serial.print("Moving to: X=");
  Serial.print(x);
  Serial.print(", Y=");
  Serial.print(y);
  Serial.print(", Z=");
  Serial.println(z);
  Serial.print("Target Angles: Base=");
  Serial.print(angles.base);
  Serial.print("°, Shoulder=");
  Serial.print(angles.shoulder);
  Serial.print("°, Elbow=");
  Serial.print(angles.elbow);
  Serial.println("°");
  
  return true;
}

// Interpolation control
InterpolationData interpData = {0, 0, 0, 0, 0, 0, 0, 0, false};

// Linear interpolation function
float lerp(float start, float end, float t) {
  return start + t * (end - start);
}

// Easing function for smooth acceleration/deceleration
float easeInOutCubic(float t) {
  if (t < 0.5) {
    return 4 * t * t * t;
  } else {
    float p = 2 * t - 2;
    return 1 + p * p * p / 2;
  }
}

// Start servo interpolation
void startServoInterpolation(float targetBase, float targetShoulder, float targetElbow, unsigned long duration) {
  interpData.startBase = currentBaseAngle;
  interpData.startShoulder = currentShoulderAngle;
  interpData.startElbow = currentElbowAngle;
  
  interpData.targetBase = targetBase;
  interpData.targetShoulder = targetShoulder;
  interpData.targetElbow = targetElbow;
  
  interpData.startTime = millis();
  interpData.duration = duration;
  interpData.active = true;
}

// Update servo positions based on interpolation
void updateServoInterpolation() {
  if (!interpData.active) {
    return;
  }
  
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - interpData.startTime;
  
  if (elapsed >= interpData.duration) {
    // Movement complete
    currentBaseAngle = interpData.targetBase;
    currentShoulderAngle = interpData.targetShoulder;
    currentElbowAngle = interpData.targetElbow;
    
    baseServo.write(currentBaseAngle);
    shoulderServo.write(currentShoulderAngle);
    elbowServo.write(currentElbowAngle);
    
    interpData.active = false;
    Serial.println("Movement complete");
    return;
  }
  
  // Calculate interpolation progress (0.0 to 1.0)
  float progress = (float)elapsed / (float)interpData.duration;
  
  // Apply easing for smooth movement
  float easedProgress = easeInOutCubic(progress);
  
  // Calculate current angles using interpolation
  currentBaseAngle = lerp(interpData.startBase, interpData.targetBase, easedProgress);
  currentShoulderAngle = lerp(interpData.startShoulder, interpData.targetShoulder, easedProgress);
  currentElbowAngle = lerp(interpData.startElbow, interpData.targetElbow, easedProgress);
  
  // Update all servos simultaneously
  baseServo.write(currentBaseAngle);
  shoulderServo.write(currentShoulderAngle);
  elbowServo.write(currentElbowAngle);
}

// Check if movement is in progress
bool isMoving() {
  return interpData.active;
}

// Wait for current movement to complete
void waitForMovementComplete() {
  while (interpData.active) {
    updateServoInterpolation();
    delay(10);
  }
}

// Smooth servo movement function (kept for compatibility, but now uses interpolation)
void smoothMoveServo(Servo &servo, int targetAngle, int stepDelay = 15) {
  // This function is now deprecated in favor of the interpolation system
  // Kept for backward compatibility
  servo.write(targetAngle);
}

// Demonstrate leg movement capabilities
void demonstrateLegMovement() {
  Serial.println("Demonstrating leg movement...");
  
  // Move to several test positions with different durations
  moveToPosition(150, 0, -100, 1500);   // Forward
  waitForMovementComplete();
  delay(500);
  
  moveToPosition(100, 100, -150, 1200); // Forward-right
  waitForMovementComplete();
  delay(500);
  
  moveToPosition(0, 150, -100, 1000);   // Right
  waitForMovementComplete();
  delay(500);
  
  moveToPosition(-100, 100, -150, 1800); // Back-right
  waitForMovementComplete();
  delay(500);
  
  moveToPosition(0, 0, -200, 1000);     // Down
  waitForMovementComplete();
  delay(500);
  
  moveToPosition(0, 0, -100, 800);      // Up
  waitForMovementComplete();
  delay(500);
}

// Simple walking pattern
void walkingPattern() {
  static unsigned long lastUpdate = 0;
  static int phase = 0;
  
  // Only start new movement if not currently moving
  if (!isMoving() && millis() - lastUpdate > 500) {
    switch (phase) {
      case 0:
        moveToPosition(120, 0, -150, 800);  // Lift and forward
        break;
      case 1:
        moveToPosition(120, 0, -200, 600);  // Place down
        break;
      case 2:
        moveToPosition(50, 0, -200, 1000);   // Pull back
        break;
      case 3:
        moveToPosition(50, 0, -150, 600);   // Lift
        break;
    }
    
    phase = (phase + 1) % 4;
    lastUpdate = millis();
  }
}

// Calculate forward kinematics (joint angles to position)
void forwardKinematics(float baseAngle, float shoulderAngle, float elbowAngle) {
  // Convert angles to radians and adjust for coordinate system
  float base_rad = (baseAngle - 90) * PI / 180.0;
  float shoulder_rad = (90 - shoulderAngle) * PI / 180.0;
  float elbow_rad = (180 - elbowAngle) * PI / 180.0;
  
  // Calculate position of shoulder joint
  float shoulder_x = L1 * cos(base_rad);
  float shoulder_y = L1 * sin(base_rad);
  float shoulder_z = 0;
  
  // Calculate position of elbow joint
  float elbow_x = shoulder_x + L2 * cos(base_rad) * cos(shoulder_rad);
  float elbow_y = shoulder_y + L2 * sin(base_rad) * cos(shoulder_rad);
  float elbow_z = shoulder_z + L2 * sin(shoulder_rad);
  
  // Calculate end effector position
  float end_x = elbow_x + L3 * cos(base_rad) * cos(shoulder_rad + elbow_rad);
  float end_y = elbow_y + L3 * sin(base_rad) * cos(shoulder_rad + elbow_rad);
  float end_z = elbow_z + L3 * sin(shoulder_rad + elbow_rad);
  
  Serial.print("Forward Kinematics - End position: X=");
  Serial.print(end_x);
  Serial.print(", Y=");
  Serial.print(end_y);
  Serial.print(", Z=");
  Serial.println(end_z);
}

// Move leg in a circular pattern with smooth interpolation
void circularMovement(float centerX, float centerY, float centerZ, float radius, float speed) {
  static float angle = 0;
  static unsigned long lastUpdate = 0;
  
  // Only update position if not currently moving
  if (!isMoving() && millis() - lastUpdate > 100) {
    float x = centerX + radius * cos(angle);
    float y = centerY + radius * sin(angle);
    float z = centerZ;
    
    moveToPosition(x, y, z, 200); // Short duration for smooth circular motion
    
    angle += speed;
    if (angle >= 2 * PI) {
      angle = 0;
    }
    
    lastUpdate = millis();
  }
}

// Home position function with interpolation
void goHome() {
  moveToPosition(0, 100, -200, 1500);
  Serial.println("Moving to home position");
}

// Move to position and wait for completion
void moveToPositionAndWait(float x, float y, float z, unsigned long duration = 1000) {
  moveToPosition(x, y, z, duration);
  waitForMovementComplete();
}
