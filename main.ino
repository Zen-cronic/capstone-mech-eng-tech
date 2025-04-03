#include <Servo.h>  // Include Servo library

// motor 1 pins
const int M1_RPWM = 11;  // Right PWM (Forward)
const int M1_LPWM = 12;  // Left PWM (Reverse)
const int M1_R_EN = 24;  // Right Enable
const int M1_L_EN = 25;  // Left Enable

// motor 2 pins
const int M2_RPWM = 3;  // Right PWM (Forward)
const int M2_LPWM = 4;  // Left PWM (Reverse)
const int M2_R_EN = 34;  // Right Enable
const int M2_L_EN = 35;  // Left Enable

const int ultrasonicPin = 40;  // SIG connected to Pin 10

const int rocketSwitchPin = 44;  // Rocket switch connected to Pin 4

// IR sensor
const int ballDetectPin = 42;  // Ball detection sensor connected to Pin 3

// Set distance threshold (adjust as needed)
const int triggerDistance = 35;  // If distance is more than 35 cm, proceed to IR sensor

// Define hood (angle adjustment) servo motor pin
const int hoodServoPin = 10;  // Servo connected to pin 5
Servo hoodServo;  // Create Servo object

const int hopperServoPin = 12;
Servo hopperServo;

void setup() {
  Serial.begin(9600);  // Start serial communication

  // Set motor driver 1 pins as outputs
  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M1_R_EN, OUTPUT);
  pinMode(M1_L_EN, OUTPUT);

  // Set motor driver 2 pins as outputs
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  pinMode(M2_R_EN, OUTPUT);
  pinMode(M2_L_EN, OUTPUT);

  // Enable motor driver 1
  digitalWrite(M1_R_EN, HIGH);
  digitalWrite(M1_L_EN, HIGH);

  // Enable motor driver 2
  digitalWrite(M2_R_EN, HIGH);
  digitalWrite(M2_L_EN, HIGH);

  // Set rocket switch pin as input
  pinMode(rocketSwitchPin, INPUT_PULLUP);  // Use internal pull-up resistor

  // Set ball detection pin as input
  pinMode(ballDetectPin, INPUT);  // Ball detection sensor as input

  // Attach the servo motors
  hoodServo.attach(hoodServoPin);
  hopperServo.attach(hopperServoPin);

  // Set initial servo position (adjust angle if necessary)
  hoodServo.write(90);  // 90 degrees (horizontal position)
  hopperServo.write(90);  // block the ball initially

  Serial.println("System Ready...");
}

void stopMotors() {
 // Stop motor 1
        analogWrite(M1_RPWM, 0); 
        analogWrite(M1_LPWM, 0);

        // Stop motor 2
        analogWrite(M2_RPWM, 0); 
        analogWrite(M2_LPWM, 0);

}

void runMotors() {
    // Run motor 1 for 3 seconds
    analogWrite(M1_RPWM, 180);  // Motor forward (adjust speed if needed)
    analogWrite(M1_LPWM, 0);  // Keep the left motor off

    // Run motor 2 for 3 seconds - reverse of motor 1
    analogWrite(M2_RPWM, 0);
    analogWrite(M2_LPWM, 180);
        
}

void loop() {
  long duration, cm;

  // Send ultrasonic pulse
  pinMode(ultrasonicPin, OUTPUT);
  digitalWrite(ultrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(ultrasonicPin, LOW);

  // Read the pulse return
  pinMode(ultrasonicPin, INPUT);
  duration = pulseIn(ultrasonicPin, HIGH);

  // Convert time to distance (cm)
  cm = duration / 29 / 2;
  Serial.println(cm);

  // Read the rocket switch state
  bool rocketSwitchState = digitalRead(rocketSwitchPin) == LOW;  // Active low

  // Read the ball detection sensor state (IR sensor)
  bool ballDetected = digitalRead(ballDetectPin) == HIGH;  // Ball detected when HIGH

  // If the rocket switch is pressed, proceed with the next steps
  if (rocketSwitchState) {
    if (cm > triggerDistance) {  // Only check for ball if no dog detected in front of ultrasonic sensor
      Serial.println("No dog detected, checking for ball...");

      if (ballDetected) {
        Serial.println("Ball detected! Launching...");

        runMotors();

        // Start the servo to launch the ball - hopper/feeder servo 
        hoodServo.write(45);  // Adjust launch angle (45 degrees)

        delay(3000);  // Motor runs for 3 sec

        stopMotors();
       
        delay(2000);  // Wait for 2 seconds (adjust as necessary)

        // Reset servo to original position (horizontal)
        hoodServo.write(90);  // Reset servo to horizontal position
      } else {
        Serial.println("No ball detected. Returning servo to original position.");
        
        // No ball detected, return servo to horizontal position
        hoodServo.write(90);  // Reset servo to horizontal position
      }
    } else {
      Serial.println("Dog detected, not launching.");
      stopMotors();

      hoodServo.write(90);  // Ensure servo is at initial position
    }
  } else {
    // If rocket switch is not pressed, ensure everything stays off
    Serial.println("Rocket switch not pressed. Motor and servo off.");

    stopMotors();

    hoodServo.write(90);  // Reset servo to horizontal position
  }

  delay(500);  // delay before next reading
}