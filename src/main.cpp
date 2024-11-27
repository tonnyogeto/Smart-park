#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Define pins
#define IR_SENSOR_PIN_1 13 // GPIO pin for the first IR sensor (entrance)
#define IR_SENSOR_PIN_2 5  // GPIO pin for the second IR sensor (slot 1)
#define IR_SENSOR_PIN_3 14 // GPIO pin for the third IR sensor (slot 2)
#define BUZZER_PIN 25      // GPIO pin for the buzzer
#define SERVO_PIN 15       // GPIO pin for the servo signal wire

// Constants
const int INITIAL_POSITION = 170; // Initial position angle for the servo

Servo myServo; // Create a Servo object

// Wi-Fi credentials
const char* ssid = "Magic in the Air";
const char* password = "Kenya@2.5!";

// MQTT Broker settings - Using test.mosquitto.org as the broker
const char* mqtt_server = "test.mosquitto.org"; // Public MQTT broker
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  pinMode(IR_SENSOR_PIN_1, INPUT);  // Set the first IR sensor pin as input
  pinMode(IR_SENSOR_PIN_2, INPUT);  // Set the second IR sensor pin as input
  pinMode(IR_SENSOR_PIN_3, INPUT);  // Set the third IR sensor pin as input
  pinMode(BUZZER_PIN, OUTPUT);      // Set the buzzer pin as output
  Serial.begin(115200);             // Start serial communication for debugging

  myServo.attach(SERVO_PIN);        // Attach servo to the defined pin
  myServo.write(INITIAL_POSITION);  // Set servo to the initial position
  delay(1000);                      // Delay to allow the servo to move to position

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("Wi-Fi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT broker
  client.setServer(mqtt_server, 1883); // Set MQTT broker server address and port
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void loop() {
  int irState1 = digitalRead(IR_SENSOR_PIN_1); // Read the first IR sensor state
  int irState2 = digitalRead(IR_SENSOR_PIN_2); // Read the second IR sensor state
  int irState3 = digitalRead(IR_SENSOR_PIN_3); // Read the third IR sensor state

  // Check if Slot 1 sensor (IR2) detects an obstacle
  if (irState2 == LOW) {
    digitalWrite(BUZZER_PIN, HIGH); // Turn on buzzer
    Serial.println("Slot 1 Full");
    client.publish("parking/slot1", "occupied"); // Send MQTT message
    delay(500); // Prevent excessive printing
    return;     // Skip the rest of the loop
  }

  // Check if Slot 2 sensor (IR3) detects an obstacle
  if (irState3 == LOW) {
    digitalWrite(BUZZER_PIN, HIGH); // Turn on buzzer
    Serial.println("Slot 2 Full");
    client.publish("parking/slot2", "occupied"); // Send MQTT message
    delay(500); // Prevent excessive printing
    return;     // Skip the rest of the loop
  }

  // Entrance sensor (IR1) check
  if (irState1 == HIGH) {
    // No obstacle detected at the entrance
    digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
    Serial.println("No obstacle detected at entrance.");
  } else {
    // Obstacle detected at the entrance
    digitalWrite(BUZZER_PIN, HIGH); // Turn on buzzer
    Serial.println("Obstacle detected at entrance!");

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

  // MQTT client loop to maintain the connection
  client.loop();

  delay(100); // Small delay for debouncing
}
