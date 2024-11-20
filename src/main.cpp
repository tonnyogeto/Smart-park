#include <Arduino.h>
#include <ESP32Servo.h>

// Define pins
#define IR_SENSOR_PIN 13  // GPIO pin for the IR sensor
#define LED_PIN 25        // GPIO pin for the LED
#define SERVO_PIN 15      // GPIO pin for the servo signal wire

// Constants
const int INITIAL_POSITION = 150; // Initial position angle for the servo

Servo myServo; // Create a Servo object

void setup() {
  pinMode(IR_SENSOR_PIN, INPUT);   // Set the IR sensor pin as input
  pinMode(LED_PIN, OUTPUT);        // Set the LED pin as output
  Serial.begin(115200);            // Start serial communication for debugging

  myServo.attach(SERVO_PIN);       // Attach servo to the defined pin
  myServo.write(INITIAL_POSITION); // Set servo to the initial position
  delay(1000);                     // Delay to allow the servo to move to position
}

void loop() {
  int irState = digitalRead(IR_SENSOR_PIN); // Read the IR sensor state

  if (irState == HIGH) {
    // No obstacle detected
    digitalWrite(LED_PIN, LOW); // Turn off LED
    Serial.println("No obstacle detected.");
  } else {
    // Obstacle detected
    digitalWrite(LED_PIN, HIGH); // Turn on LED
    Serial.println("Obstacle detected!");

    // Rotate servo from initial position to 0 degrees
    for (int angle = INITIAL_POSITION; angle >= 0; angle--) {
      myServo.write(angle);
      delay(30); // Delay to control the servo speed
    }

    delay(5000); // Wait for 5 seconds

    // Return servo to the initial position
    for (int angle = 0; angle <= INITIAL_POSITION; angle++) {
      myServo.write(angle);
      delay(30); // Delay to control the servo speed
    }
  }

  delay(100); // Small delay for debouncing
}



