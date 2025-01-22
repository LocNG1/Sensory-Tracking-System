#include <Servo.h>
#include <Stepper.h> // Ensure Stepper library is included

// Servo setup
Servo myServo;
const int servoPin = 10;

// Ultrasonic sensor setup
const int trigPin = 7;
const int echoPin = 8;

// Stepper motor setup
#define STEPS_PER_REV 2048 // For 28BYJ-48
const int IN1 = 2, IN2 = 3, IN3 = 4, IN4 = 5;
Stepper stepperMotor(STEPS_PER_REV, IN1, IN3, IN2, IN4); // Create a Stepper object

// Variables for integration
int detectedAngle = -1; // To store the angle where the object is detected
const float stepperAnglePerStep = 360.0 / STEPS_PER_REV;

// Initial servo position (0 degrees)
int servoHeading = 0;

void setup() {
  // Initialize servo
  myServo.attach(servoPin);
  myServo.write(servoHeading); // Start at 0 degrees

  // Initialize ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize stepper motor
  stepperMotor.setSpeed(10); // Adjust speed as needed
  stepperMotor.step(0); // This ensures the motor is at the 0-step position

  // Start serial communication
  Serial.begin(9600);
  Serial.println("System Initialized");
}

void loop() {
  // Servo sweeping loop
  for (servoHeading = 0; servoHeading <= 180; servoHeading++) {
    myServo.write(servoHeading);
    delay(15); // Allow servo to reach position

    // Display the compass heading based on servo position
    Serial.print("Compass Heading: ");
    Serial.println(servoHeading); // Simulating compass with the servo position

    float distance = measureDistance(); // Measure distance using ultrasonic sensor

    if (distance > 0 && distance <= 30) { // Object detected within 30 cm
      detectedAngle = servoHeading; // Record the angle
      Serial.print("Object detected at angle: ");
      Serial.println(detectedAngle);

      // Rotate stepper motor to detected angle
      moveToAngle(detectedAngle);
    }
  }

  for (servoHeading = 180; servoHeading >= 0; servoHeading--) {
    myServo.write(servoHeading);
    delay(15);

    // Display the compass heading based on servo position
    Serial.print("Compass Heading: ");
    Serial.println(servoHeading);

    float distance = measureDistance();

    if (distance > 0 && distance <= 30) {
      detectedAngle = servoHeading;
      Serial.print("Object detected at angle: ");
      Serial.println(detectedAngle);

      moveToAngle(detectedAngle);
    }
  }
}

// Function to measure distance using the ultrasonic sensor
float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.034) / 2; // Convert to cm
  return distance;
}

// Function to rotate the stepper motor to a specific angle (adjusted for gear ratio)
void moveToAngle(int targetAngle) {
  static int currentStepperAngle = 0; // Track the current angle of the stepper motor

  int angleDifference = targetAngle - currentStepperAngle;
  int stepsToMove = (angleDifference / stepperAnglePerStep) * 1.88; // Adjust for gear ratio

  Serial.print("Rotating stepper to angle (adjusted): ");
  Serial.println(targetAngle);

  stepperMotor.step(stepsToMove); // Move the stepper motor
  currentStepperAngle = targetAngle; // Update the current angle
}
