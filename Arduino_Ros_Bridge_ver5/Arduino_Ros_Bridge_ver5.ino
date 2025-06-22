#include <Wire.h>
#include <Adafruit_MCP4725.h>

// Encoder pin definitions
const int encoder1PinA = 2;
const int encoder1PinB = 3;
const int encoder2PinA = 6;
const int encoder2PinB = 7;

// Ultrasonic echo/trigger pin
const int trigPinLeft = 8;
const int echoPinLeft = 9;
const int trigPinRight = 12;
const int echoPinRight = 13;

// MCP4725 DAC instances for controlling motor voltages
Adafruit_MCP4725 dac1;  // X-axis (vertical) motor
Adafruit_MCP4725 dac2;  // Y-axis (horizontal) motor

// Encoder variables
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
volatile int encoder1State = 0;
volatile int encoder2State = 0;

// Define the maximum velocity of the motors (m/s)
#define MAX_VELOCITY 0.2
#define MIN_VOLTAGE 0.5
#define MAX_VOLTAGE 4.5
#define NEUTRAL_LOW 1.9
#define NEUTRAL_HIGH 3.1


// Bus voltage
float busVoltage = 5.0;

// Wheel parameters
const float wheelDiameter = 0.4;  // meters
const float wheel_radius = 0.20;
const float wheel_separation = 0.606;
const float pulsesPerRevolution = 7200.0;  // Encoder pulses per revolution

// PID variables
//double kp = 0.3, ki = 0.0, kd = 0.05;
double kp = 0.1, ki = 0.0, kd = 0.0; // kp = 0.05
double kp_w = 0.1, ki_w = 0.0, kd_w = 0.0;
double bias = 0.5;

// PID inputs, outputs, and setpoints
double angularVelocity1 = 0.0, angularVelocity2 = 0.0;
double controlSignal1 = 0.0, controlSignal2 = 0.0;
double targetLinearVelocity = 0.0, targetAngularVelocity = 0.0;
double linearVelocity = 0.0, angularVelocity = 0.0;
double maxControlSignal = 20.0;


// Timing variables
unsigned long lastControlTime = 0;
const int controlInterval = 33;  // 30 Hz (33 ms)

// Function prototypes
void updateAngularVelocities();
double calculateAngularVelocity(long encoderCount, long &lastEncoderCount, unsigned long &lastTime);
void applyPIDControl();
void setJoystickVoltage(double x, double y);
void processSerialCommand(const String &command);

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);

  // Initialize I2C for DAC
  Wire.begin();
  dac1.begin(0x61);  // Address for DAC1
  dac2.begin(0x60);  // Address for DAC2

  // Initialize encoder pins
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  // Initialize ultrasonic sensor pins
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);

  // Attach interrupts for Encoder 1 and 2
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), encoder1_ISR, CHANGE);

  PCICR |= (1 << PCIE2);  // Enable PCINT2 (pins 6 and 7)
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);  // Enable interrupts for pins 6 and 7

}

void loop() {
  // Process serial communication commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processSerialCommand(command);
  }


  // Run control loop
  unsigned long currentTime = millis();
  if (currentTime - lastControlTime >= controlInterval) {
  
    // Measure bus voltage
    busVoltage = analogRead(A0) * (5.0 / 1023.0);  // Read analog value
//    Serial.println(busVoltage);
    
    lastControlTime = currentTime;

    // Update angular velocities
    updateAngularVelocities();

    // Apply PID control
    applyPIDControl();
  }
}

// Process incoming serial commands
void processSerialCommand(const String &command) {
  if (command.startsWith("e")) {
    // Read encoder values
    Serial.print("encoder");
    Serial.print(encoder1Count);
    Serial.print(" ");
    Serial.println(encoder2Count);
  } else if (command.startsWith("m")) {
    // Set target angular velocities
    int spaceIndex = command.indexOf(' ');
    String value1 = command.substring(1, spaceIndex);
    String value2 = command.substring(spaceIndex + 1);
    double m1 = value1.toDouble();
    double m2 = value2.toDouble();
    targetLinearVelocity = (m1 + m2)*wheel_radius/2;
    targetAngularVelocity = (m1 - m2)*wheel_radius/wheel_separation;
    
  } else if (command.startsWith("u")) {
    // Set PID values
    int firstColon = command.indexOf(':');
    int secondColon = command.indexOf(':', firstColon + 1);
    int thirdColon = command.indexOf(':', secondColon + 1);

    kp = command.substring(1, firstColon).toDouble();
    kd = command.substring(firstColon + 1, secondColon).toDouble();
    ki = command.substring(secondColon + 1, thirdColon).toDouble();

  } else if (command.startsWith("s")) {
    float ultrasonicLeft = readUltrasonic(trigPinLeft, echoPinLeft);
    float ultrasonicRight = readUltrasonic(trigPinRight, echoPinRight);
    Serial.print("sonic");
    Serial.print(ultrasonicLeft);
    Serial.print(" ");
    Serial.println(ultrasonicRight);
  }
}

// Update ultrasonic reading
float readUltrasonic(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000);
  float distance = (duration * 0.034 / 2)/100;
  if (distance > 2.00 || distance == 0) distance = 2.00;
  delay(5);
  return distance;
}

// Update angular velocities for both wheels
void updateAngularVelocities() {
  static long lastCount1 = 0, lastCount2 = 0;
  static unsigned long lastTime1 = 0, lastTime2 = 0;

  angularVelocity1 = calculateAngularVelocity(encoder1Count, lastCount1, lastTime1);
  angularVelocity2 = calculateAngularVelocity(encoder2Count, lastCount2, lastTime2);

  linearVelocity = (angularVelocity1 + angularVelocity2)*wheel_radius/2;
  angularVelocity = (angularVelocity1 - angularVelocity2)*wheel_radius/wheel_separation;
}

// Calculate angular velocity
double calculateAngularVelocity(long encoderCount, long &lastEncoderCount, unsigned long &lastTime) {
  unsigned long currentTime = millis();
  double deltaTime = (currentTime - lastTime) / 1000.0; // Time in seconds
  if (deltaTime > 0) {
    long deltaCount = encoderCount - lastEncoderCount;
    lastTime = currentTime;
    lastEncoderCount = encoderCount; // Update last encoder count
    return (deltaCount / pulsesPerRevolution) * (2.0 * PI) / deltaTime; // Angular velocity in rad/s
  }
  return 0.0;
}

// Apply PID control to motors
void applyPIDControl() {
  // Time variables for control loop
  static double prevError1 = 0.0, prevError2 = 0.0;
  static double integral1 = 0.0, integral2 = 0.0;
  static unsigned long prevTime1 = millis(), prevTime2 = millis();

  unsigned long currentTime = millis();
  double deltaTime1 = (currentTime - prevTime1) / 1000.0 + 1e-6; // seconds
  double deltaTime2 = (currentTime - prevTime2) / 1000.0 + 1e-6;

  // Calculate errors for both wheels
  double error1 = targetLinearVelocity - linearVelocity;
  double error2 = targetAngularVelocity - angularVelocity;

  // Update integrals
  integral1 += error1 * deltaTime1;
  integral2 += error2 * deltaTime2;

  // Update control signals using PID formula
  controlSignal1 += kp * error1 + ki * integral1   + kd * ((error1 - prevError1) / deltaTime1);
  controlSignal2 += kp_w * error2 + ki_w * integral2 + kd_w * ((error2 - prevError2) / deltaTime2);

  // Save previous values for next iteration
  prevError1 = error1;
  prevError2 = error2;
  prevTime1 = currentTime;
  prevTime2 = currentTime;



  
  double verticalVoltage   = 2.5 - controlSignal1;
  double horizontalVoltage = 2.5 + controlSignal2;

  constrain(verticalVoltage, 0.6, 4.4);
  constrain(horizontalVoltage, 0.6, 4.4);  
  
  // Output voltages to DACs
  dac1.setVoltage((verticalVoltage * 4095.0 / busVoltage), false);
  dac2.setVoltage((horizontalVoltage * 4095.0 / busVoltage), false);
//
//  // Debugging output
///  Serial.print("TargetLV: ");
///  Serial.print(targetLinearVelocity);
//  Serial.print(" TargetAV: ");
//  Serial.print(targetAngularVelocity);
///  Serial.print(" CurrentLV: ");
///  Serial.print(linearVelocity);
//  Serial.print(" CurrentAV: ");
//  Serial.print(angularVelocity);
///  Serial.print(" controlSignal1: ");
///  Serial.print(controlSignal1);
//  Serial.print(" controlSignal2: ");
//  Serial.print(controlSignal2);
///  Serial.print(" VerticalVoltage: ");
///  Serial.println(verticalVoltage);
//  Serial.print(" HorizontalVoltage: ");
//  Serial.println(horizontalVoltage);


}



// Interrupt Service Routine for Encoder 1
void encoder1_ISR() {
  bool currentA = digitalRead(encoder1PinA);
  bool currentB = digitalRead(encoder1PinB);
  int newState = (currentA << 1) | currentB;

  if ((encoder1State == 0b00 && newState == 0b01) ||
      (encoder1State == 0b01 && newState == 0b11) ||
      (encoder1State == 0b11 && newState == 0b10) ||
      (encoder1State == 0b10 && newState == 0b00)) {
    encoder1Count++;
  } else if ((encoder1State == 0b00 && newState == 0b10) ||
             (encoder1State == 0b10 && newState == 0b11) ||
             (encoder1State == 0b11 && newState == 0b01) ||
             (encoder1State == 0b01 && newState == 0b00)) {
    encoder1Count--;
  }

  encoder1State = newState;
}

// Interrupt Service Routine for Encoder 2
ISR(PCINT2_vect) {
  bool currentA = digitalRead(encoder2PinA);
  bool currentB = digitalRead(encoder2PinB);
  int newState = (currentA << 1) | currentB;

  if ((encoder2State == 0b00 && newState == 0b01) ||
      (encoder2State == 0b01 && newState == 0b11) ||
      (encoder2State == 0b11 && newState == 0b10) ||
      (encoder2State == 0b10 && newState == 0b00)) {
    encoder2Count++;
  } else if ((encoder2State == 0b00 && newState == 0b10) ||
             (encoder2State == 0b10 && newState == 0b11) ||
             (encoder2State == 0b11 && newState == 0b01) ||
             (encoder2State == 0b01 && newState == 0b00)) {
    encoder2Count--;
  }

  encoder2State = newState;
}
