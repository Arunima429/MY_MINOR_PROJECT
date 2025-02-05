 #include "DHT.h"

// Define the DHT type and the pin it's connected to
#define DHTPIN 2       // Pin connected to the DATA pin of DHT11
#define DHTTYPE DHT11  // DHT 11 sensor

// Initialize the DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Define motor control pins
int IN1 = 5;  // Motor driver input pin 1
int IN2 = 6;  // Motor driver input pin 2
int EN = 9;   // Motor driver enable pin for PWM (optional for speed control)

// Define the rain sensor pin
#define RAIN_SENSOR_PIN A0 // Analog pin connected to the rain sensor

// Variable to store rain detection state
bool isRaining = false;

void setup() {
  Serial.begin(9600); // Start serial communication
  Serial.println("DHT11 and Rain Sensor Test!");

  dht.begin(); // Initialize the DHT sensor

  // Set motor control pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);  // If using PWM for speed control
}

void loop() {
  // Wait a few seconds between readings
  delay(2000);

  // Read temperature and humidity from DHT11
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Check if the readings from the DHT sensor failed
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    // Print DHT sensor results to the Serial Monitor
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
  }

  // Read rain sensor value
  int rainValue = analogRead(RAIN_SENSOR_PIN);

  // Convert the rain sensor value to a more meaningful range (0-100% wetness)
  float wetnessPercentage = map(rainValue, 0, 1023, 100, 0);

  // Print rain sensor results to the Serial Monitor
  Serial.print("Rain Sensor Value: ");
  Serial.print(rainValue);
  Serial.print("\tWetness: ");
  Serial.print(wetnessPercentage);
  Serial.println(" %");

  // Check rain status and act accordingly
  if (wetnessPercentage > 60) {
    if (!isRaining) {  // If it's raining and motor is not already stopped
      Serial.println("It's raining! Covering...");
      moveMotorForward();  // Move motor to cover
      isRaining = true;  // Set the rain status to true
    }
  } else {
    if (isRaining) {  // If rain stops and motor is not already uncovering
      Serial.println("No rain! Uncovering...");
      moveMotorBackward();  // Move motor to uncover
      isRaining = false;  // Set the rain status to false
    }
  }

  Serial.println(); // Blank line for readability
}

// Function to move the motor forward (covering action)
void moveMotorForward() {
  // Run motor forward by setting IN1 HIGH and IN2 LOW
  digitalWrite(IN1, HIGH);   // Motor runs forward
  digitalWrite(IN2, LOW);    // Motor runs forward
  analogWrite(EN, 200);      // Set motor speed (0 to 255, 255 = full speed)
  
  // Run motor for a set time (e.g., 3 seconds for covering)
  delay(9000);  // Motor runs forward for 3 seconds
  
  // Stop motor after running forward
  digitalWrite(IN1, LOW);    // Stop motor
  digitalWrite(IN2, LOW);    // Stop motor
}

// Function to move the motor backward (uncovering action)
void moveMotorBackward() {
  // Run motor backward by setting IN1 LOW and IN2 HIGH
  digitalWrite(IN1, LOW);   // Motor runs backward
  digitalWrite(IN2, HIGH);  // Motor runs backward
  analogWrite(EN, 200);     // Set motor speed (0 to 255, 255 = full speed)

  // Run motor for a set time (e.g., 3 seconds for uncovering)
  delay(9000);  // Motor runs backward for 3 seconds
  
  // Stop motor after running backward
  digitalWrite(IN1, LOW);    // Stop motor
  digitalWrite(IN2, LOW);    // Stop motor
}