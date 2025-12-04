/*
  DMP Gimbal Stabilization with PID Control
  
  This sketch uses the MPU6050's Digital Motion Processor (DMP) for stable 
  3D orientation (Quaternion filtering) and applies three independent PID 
  controllers to stabilize the Yaw, Pitch, and Roll axes using three servos.
*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// hardware declarations
MPU6050 mpu;
Servo servoYaw;
Servo servoPitch;
Servo servoRoll;

#define INTERRUPT_PIN 2

// --- MPU State Variables ---
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];             // [yaw, pitch, roll] in radians

// --- Loop Timing and dt ---
unsigned long lastLoopTime;
const int LOOP_TIME_MS = 10; // Target 100 Hz update rate
float dt = 0.01;             // Will be calculated, default for safety

// --- PID State Variables and Setpoints ---
// Setpoints are the target angles (0.0 = level)
float setpointYaw = 0.0;
float setpointPitch = 0.0;
float setpointRoll = 0.0;

// PID variables
float lastErrorYaw = 0;
float lastDerivativeYaw = 0;

float lastErrorPitch = 0;
float lastDerivativePitch = 0;

float lastErrorRoll = 0;
float lastDerivativeRoll = 0;

// ----- PID Constants -----
float Kp_Yaw = 1;
float Kd_Yaw = 0;

float Kp_Pitch = 1.5;
float Kd_Pitch = 0.3;

float Kp_Roll = 1.5;
float Kd_Roll = 0.3;

// --- Yaw Offset Correction ---
float yawZeroOffset = 0.0;
int calibrationCounter = 0;
const int CAL_READINGS = 300; // Calibrate over first 300 loops

// === INTERRUPT DETECTION ROUTINE ===
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

// PID controller function
// Calculates and returns the correction command for a single axis
float computePID(float currentAngle, float setpoint, float Kp, float Kd, float& lastError, float& lastDerivative) {
    float error = setpoint - currentAngle;
    float derivative = (error - lastError) / dt;

    // low-pass filter
    derivative = 0.7 * lastDerivative + 0.3 * derivative;
    lastDerivative = derivative;

    float output = (Kp * error) + (Kd * derivative);
    // if(abs(output) < 2.0) output = 0;
    output = constrain(output, -45.0, 45.0);
    lastError = error;

    return output;
}


void setup() {
  // --- I2C Initialization ---
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock
  #endif

  Serial.begin(115200);
  while (!Serial);
  Serial.println("MPU6050 DMP PID Control Starting...");

  // --- MPU Initialization ---
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // --- DMP Initialization ---
  devStatus = mpu.dmpInitialize();

  // Load offsets
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(0); // <- TUNE THIS FOR DRIFT
  mpu.setZAccelOffset(1551);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Init failed (code "));
    Serial.println(devStatus);
  }

  // Servo initialization
  servoYaw.attach(10);
  servoPitch.attach(8);
  servoRoll.attach(9);
  
  // Center servos on startup
  servoYaw.write(90);
  servoPitch.write(90);
  servoRoll.write(90);

  lastLoopTime = millis();
}


void loop() {
  // 1. MPU DMP Data Retrieval
  if (!dmpReady) return;

  // Check if MPU interrupt happened (new data is ready)
  // If no new data, we EXIT immediately. We do NOT update time yet.
  if (!mpuInterrupt && fifoCount < packetSize) {
    return;
  }
  
  // --- 2. Timing: Calculate dt ONLY when we actually have data ---
  unsigned long currentTime = millis();
  dt = (currentTime - lastLoopTime) / 1000.0;
  
  // Safety check: If dt is too small (prevent divide by zero), ignore this run
  if (dt < 0.001 || dt > 0.1) { // Less than 5ms is essentially noise/double-trigger
     return; 
  }
  lastLoopTime = currentTime; // Reset timer for next actual data packet

  // Reset interrupt flag
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // Check for overflow (common issue)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    // Serial.println(F("FIFO overflow!")); 
    return;
  } 
  
  // Check for DMP data ready
  if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // Wait for correct available data length
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
    // Read packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // --- 3. Extract Angles (World Frame) ---
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Convert Radians to Degrees
    float currentYaw = ypr[0] * 180 / M_PI;
    float currentPitch = ypr[1] * 180 / M_PI;
    float currentRoll = ypr[2] * 180 / M_PI;

    // --- 4. Yaw Drift Correction (Custom Zeroing) ---
    if (calibrationCounter < CAL_READINGS) {
      yawZeroOffset += currentYaw;  // Accumulate
      calibrationCounter++;
      return;
    } else if (calibrationCounter == CAL_READINGS) {
      yawZeroOffset /= CAL_READINGS;  // Average once
      calibrationCounter++;  // Increment so we don't repeat this
    } else {
      currentYaw = currentYaw - yawZeroOffset;
      if (currentYaw > 180) currentYaw -= 360;
      if (currentYaw < -180) currentYaw += 360;
    }

    // --- 5. Compute PID Outputs (World Frame Corrections) ---
    // float outputYaw = computePID(...); // <-- YAW REMOVED
    float outputYaw = computePID(currentYaw, setpointYaw, Kp_Yaw, Kd_Yaw, lastErrorYaw, lastDerivativeYaw);
    float outputPitch = computePID(currentPitch, setpointPitch, Kp_Pitch, Kd_Pitch, lastErrorPitch, lastDerivativePitch);
    float outputRoll = computePID(currentRoll, setpointRoll, Kp_Roll, Kd_Roll, lastErrorRoll, lastDerivativeRoll);
    
    // --- 6. **** AXIS DECOUPLING / ROTATION MATRIX **** ---
    
    // Convert currentYaw to radians for trig functions
    float yaw_rad = currentYaw * M_PI / 180.0;
    float cos_yaw = cos(yaw_rad);
    float sin_yaw = sin(yaw_rad);

    // This 2D rotation matrix "decouples" the axes
    float motor_cmd_for_roll_axis = (outputRoll * cos_yaw) + (outputPitch * sin_yaw);
    float motor_cmd_for_pitch_axis = (outputPitch * cos_yaw) - (outputRoll * sin_yaw);

    // --- 7. Map PID Output to Servos ---
    
    int servoCmdYaw = 90 - (int)outputYaw;
    servoCmdYaw = constrain(servoCmdYaw, 0, 180);
    servoYaw.write(servoCmdYaw);
    
    // PITCH (Use the new, decoupled command)
    int servoCmdPitch = 90 + (int)motor_cmd_for_pitch_axis;
    servoCmdPitch = constrain(servoCmdPitch, 0, 180);
    servoPitch.write(servoCmdPitch);

    // ROLL (Use the new, decoupled command)
    int servoCmdRoll = 90 - (int)motor_cmd_for_roll_axis; 
    servoCmdRoll = constrain(servoCmdRoll, 0, 180);
    servoRoll.write(servoCmdRoll);
  }
}
