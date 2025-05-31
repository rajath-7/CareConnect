#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME "CareConnect"
#define BLYNK_AUTH_TOKEN ""
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"  // SparkFun MAX3010x library for MAX30102

// WiFi credentials
char ssid[] = "";
char pass[] = "";

// Sensor objects
Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

// Button pin
const int buttonPin = 23;  // GPIO 23 for tactile button
int lastButtonState = HIGH; // Button is HIGH when not pressed (pull-up)

// Timer for debouncing fall detection and button
unsigned long lastFallTime = 0;
unsigned long lastButtonTime = 0;
const unsigned long fallDebounce = 5000;  // 5 seconds debounce for fall
const unsigned long buttonDebounce = 200; // 200 ms debounce for button

// Buffer for MAX30102 data
#define BUFFER_SIZE 100
uint32_t irBuffer[BUFFER_SIZE];  // IR data buffer
uint32_t redBuffer[BUFFER_SIZE]; // Red data buffer

// BlynkTimer for scheduling
BlynkTimer timer;

// Function to calculate heart rate (simplified)
int calculateHeartRate(uint32_t *irBuffer, int bufferSize) {
  int peaks = 0;
  uint32_t threshold = 0;
  
  // Calculate a simple threshold (average of buffer)
  uint32_t sum = 0;
  for (int i = 0; i < bufferSize; i++) {
    sum += irBuffer[i];
  }
  threshold = sum / bufferSize;

  // Count peaks above threshold
  for (int i = 1; i < bufferSize - 1; i++) {
    if (irBuffer[i] > threshold && irBuffer[i] > irBuffer[i - 1] && irBuffer[i] > irBuffer[i + 1]) {
      peaks++;
    }
  }

  // Estimate BPM (peaks over time, assuming 100 samples = 4 seconds at 25Hz)
  int bpm = (peaks * 60) / 4; // Simplified: 60 sec / 4 sec window
  return bpm;
}

// Function to send MPU-6050 data (called 2 times per second)
void sendMPU6050Data() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Send acceleration data (m/s²) to Blynk and print to Serial
  Blynk.virtualWrite(V3, a.acceleration.x);
  Blynk.virtualWrite(V4, a.acceleration.y);
  Blynk.virtualWrite(V5, a.acceleration.z);
  Serial.print("Accel X: ");
  Serial.print(a.acceleration.x);
  Serial.print(" m/s², Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(" m/s², Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s²");

  // Send gyroscope data (degrees/s) to Blynk and print to Serial
  Blynk.virtualWrite(V6, g.gyro.x);
  Blynk.virtualWrite(V7, g.gyro.y);
  Blynk.virtualWrite(V8, g.gyro.z);
  Serial.print("Gyro X: ");
  Serial.print(g.gyro.x);
  Serial.print(" deg/s, Y: ");
  Serial.print(g.gyro.y);
  Serial.print(" deg/s, Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" deg/s");

  // Fall detection logic
  float accelMag = sqrt(a.acceleration.x * a.acceleration.x +
                        a.acceleration.y * a.acceleration.y +
                        a.acceleration.z * a.acceleration.z);
  float gyroMag = sqrt(g.gyro.x * g.gyro.x +
                       g.gyro.y * g.gyro.y +
                       g.gyro.z * g.gyro.z);

  if (accelMag > 15 || gyroMag > 200) {
    unsigned long currentTime = millis();
    if (currentTime - lastFallTime >= fallDebounce) {
      Blynk.virtualWrite(V2, 1); // Trigger fall alert (LED)
      Blynk.logEvent("potential_fall", "Potential Fall Detected!");
      Serial.println("Potential Fall Detected!");
      lastFallTime = currentTime;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configure button pin with internal pull-up
  pinMode(buttonPin, INPUT_PULLUP);

  // Connect to WiFi and Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Initialize MPU-6050
  if (!mpu.begin()) {
    Serial.println("MPU-6050 not found!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.println("MPU-6050 initialized successfully");

  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    while (1) delay(10);
  }
  particleSensor.setup(); // Default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Adjust red LED brightness
  particleSensor.setPulseAmplitudeIR(0x0A);  // Adjust IR LED brightness
  Serial.println("MAX30102 initialized successfully");

  // Set timer for MPU-6050 updates (every 500 ms = 2 times per second)
  timer.setInterval(500L, sendMPU6050Data);
}

void loop() {
  Blynk.run();
  timer.run();

  // Button press to cancel fall alert
  int buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {
    unsigned long currentTime = millis();
    if (currentTime - lastButtonTime >= buttonDebounce) {
      if (buttonState == LOW) { // Button pressed (LOW with pull-up)
        Serial.println("Button Clicked");
        Blynk.virtualWrite(V2, 0); // Reset fall alert (LED off)
        Blynk.logEvent("fall_cancelled", "Fall Alert Cancelled by User");
        Serial.println("Fall Alert Cancelled by User");
      }
      lastButtonTime = currentTime;
    }
  }
  lastButtonState = buttonState;

  // Read MAX30102 data (less frequent to avoid blocking)
  static unsigned long lastMAX30102Time = 0;
  if (millis() - lastMAX30102Time >= 4000) { // Update every 4 seconds
    for (int i = 0; i < BUFFER_SIZE; i++) {
      while (particleSensor.available() == false) // Wait for sample
        particleSensor.check(); // Check sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); // Move to next sample
      delay(10); // Adjust sampling rate (e.g., 10ms = 100Hz)
    }

    // Calculate heart rate and send to Blynk
    int heartRate = calculateHeartRate(irBuffer, BUFFER_SIZE);
    int spo2 = 90; // Placeholder for SpO2 (replace with real algorithm if available)

    if (heartRate > 0 && heartRate < 200) { // Basic validation
      Blynk.virtualWrite(V0, heartRate);
      Blynk.virtualWrite(V1, spo2); // Placeholder value
      Serial.print("Heart Rate: ");
      Serial.print(heartRate);
      Serial.print(" BPM, SpO2: ");
      Serial.println(spo2);
    }
    lastMAX30102Time = millis();
  }
}





















