#include <ezButton.h>  // Library for button handling
#include <WiFi.h>
#include <MQTT.h>
#include "ArduinoGraphics.h"  // Add this for LED matrix
#include "Arduino_LED_Matrix.h"  // Add this for LED matrix
#include <ArduinoJson.h>  // Add this for JSON parsing
#include <Arduino_FreeRTOS.h>

// --- LED Matrix ---
ArduinoLEDMatrix matrix;  // Create matrix object

// --- FreeRTOS Task Handles ---
TaskHandle_t sensorTaskHandle;
TaskHandle_t communicationTaskHandle;

// --- Rotary Encoder for Steering (ezButton approach with original steering logic) ---
#define CLK_PIN 2
#define DT_PIN 3
#define SW_PIN 4

#define DIRECTION_CW 0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction

volatile int encoderCounter = 0;
volatile int direction = DIRECTION_CW;
volatile unsigned long last_time;  // for debouncing
int prevEncoderCounter = 0;

// Original steering variables
long steeringValue = 0;
const int maxSteeringValue = 32767;
const int minSteeringValue = -32767;

ezButton encoderButton(SW_PIN);  // create ezButton object for encoder switch

// --- Pedals ---
int forwardPedalPin = A1;
int brakePedalPin   = A2;
int forwardPedalValue = 0;
int brakePedalValue   = 0;
int mappedThrottle = 0;
int mappedBrake = 0;

// --- Simulated RPM Output ---
const int rpmOutPin = 9;   // PWM output pin
float rpmSignal = 0;       // smoothed output (0–255)

// --- Gear system ---
#define GEAR_UP_PIN   6
#define GEAR_DOWN_PIN 7
int gear = 0; // 0 = Neutral, 1..5 = gears
int lastGear = -1; // Track previous gear for display updates
int lastUpState = HIGH;
int lastDownState = HIGH;
bool gearJustShifted = false;
unsigned long shiftTime = 0;
const int shiftDropDuration = 300;  // ms hold RPM drop
float shiftDropFactor = 0.7;        // keep 70% RPM when upshifting

// --- Gear limits (PWM values for max output) ---
const int gearMaxOutput[6] = {0, 60, 90, 110, 125, 140}; // ~3V max

// --- Parental Control Settings ---
struct ParentalControl {
  bool enabled = false;
  int maxGear = 5;      // Maximum allowed gear (1-5)
  int maxThrottle = 100; // Maximum allowed throttle percentage (0-100)
};

ParentalControl parentalControl;

// --- Shared Sensor Data Structure ---
struct SensorData {
  long steering;
  int throttle;
  int brake;
  int currentGear;
  float rpm;
  unsigned long timestamp;
  bool dataReady;
};

volatile SensorData sensorData = {0, 0, 0, 0, 0.0, 0, false};

// --- Simple message queue for MQTT ---
struct MQTTMessage {
  String topic;
  String payload;
};

const int MQTT_QUEUE_SIZE = 10;
MQTTMessage mqttMessageQueue[MQTT_QUEUE_SIZE];
volatile int mqttQueueHead = 0;
volatile int mqttQueueTail = 0;
volatile int mqttQueueCount = 0;

// --- Simplified Alert System ---
const int HIGH_GEAR_THRESHOLD = 4;        // Gear 4 or 5 considered high
const int HIGH_THROTTLE_THRESHOLD = 70;   // 70% throttle considered high
const unsigned long SUSTAINED_DURATION = 10000; // 3 seconds of sustained high gear+throttle
const unsigned long ALERT_COOLDOWN = 10000;    // 10 seconds between alerts

// Alert tracking
unsigned long highGearThrottleStartTime = 0;
bool inHighGearThrottleMode = false;
unsigned long lastSustainedAlert = 0;

// --- WiFi & MQTT ---
const char* WIFI_SSID = "whoami";
const char* WIFI_PASS = "n19101702092548";
const char* MQTT_SERVER = "10.23.166.129";
const int   MQTT_PORT   = 1883;
WiFiClient wifiClient;
MQTTClient mqttClient(256); // Buffer size for MQTT

// --- MQTT Message Callback ---
void messageReceived(String &topic, String &payload) {
  Serial.println("Incoming MQTT message:");
  Serial.println("Topic: " + topic);
  Serial.println("Payload: " + payload);
  
  // Handle parental control settings updates
  if (topic == "car/driver1/parental_control_settings") {
    handleParentalControlUpdate(payload);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { }
  
  pinMode(rpmOutPin, OUTPUT);
  pinMode(GEAR_UP_PIN, INPUT_PULLUP);
  pinMode(GEAR_DOWN_PIN, INPUT_PULLUP);
  
  // --- LED Matrix Setup ---
  matrix.begin();
  updateGearDisplay(); // Show initial gear (0/N)
  
  // --- Encoder Setup ---
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  encoderButton.setDebounceTime(50);  // set debounce time to 50 milliseconds
  
  // Use interrupt for CLK pin
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), ISR_encoderChange, RISING);
  
  randomSeed(analogRead(A0)); // Randomness for jitter

  // --- WiFi connection ---
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // --- MQTT connection ---
  mqttClient.begin(MQTT_SERVER, MQTT_PORT, wifiClient);
  mqttClient.onMessage(messageReceived); // Set callback for incoming messages
  
  Serial.print("Connecting to MQTT");
  while (!mqttClient.connect("Driver1")) {
    Serial.print("*");
    delay(300);
  }
  Serial.println("MQTT connected");
  
  // Subscribe to parental control settings topic
  mqttClient.subscribe("car/driver1/parental_control_settings");
  Serial.println("Subscribed to parental control settings");

  // --- Create FreeRTOS Tasks ---
  auto const rc_sensor = xTaskCreate(
    sensorTaskFunction,
    static_cast<const char*>("SensorTask"),
    1025 / 4,   /* usStackDepth in words */
    nullptr,   /* pvParameters */
    2,         /* uxPriority - higher priority */
    &sensorTaskHandle
  );

  if (rc_sensor != pdPASS) {
    Serial.println("Failed to create sensor task");
    return;
  }

  auto const rc_comm = xTaskCreate(
    communicationTaskFunction,
    static_cast<const char*>("CommTask"),
    2048 / 4,  /* usStackDepth in words - larger for WiFi/MQTT */
    nullptr,   /* pvParameters */
    1,         /* uxPriority - lower priority */
    &communicationTaskHandle
  );

  if (rc_comm != pdPASS) {
    Serial.println("Failed to create communication task");
    return;
  }

  Serial.println("Starting FreeRTOS scheduler...");
  
  // Start the scheduler
  vTaskStartScheduler();
  
  // We'll never get here
  for(;;);
}

void loop() {
  // Empty - everything handled by FreeRTOS tasks
}

// ================= FREERTOS TASK 1: SENSOR TASK =================
void sensorTaskFunction(void *pvParameters) {
  for(;;) {
    encoderButton.loop();  // Update button state
    
    // ------------------- Rotary Encoder (Steering) -------------------
    long deltaEncoderValue = encoderCounter - prevEncoderCounter;
    steeringValue += deltaEncoderValue;
    
    // Constrain steering
    if (steeringValue > maxSteeringValue) steeringValue = maxSteeringValue;
    else if (steeringValue < minSteeringValue) steeringValue = minSteeringValue;
    
    prevEncoderCounter = encoderCounter;
    
    // ------------------- Pedals -------------------
    forwardPedalValue = analogRead(forwardPedalPin);
    brakePedalValue   = analogRead(brakePedalPin);
    mappedThrottle = map(forwardPedalValue, 180, 900, 0, 100);
    mappedThrottle = constrain(mappedThrottle, 0, 100);
    mappedBrake = map(brakePedalValue, 180, 900, 0, 100);
    mappedBrake = constrain(mappedBrake, 0, 100);
    
    // Apply parental control throttle limit
    if (parentalControl.enabled && mappedThrottle > parentalControl.maxThrottle) {
      mappedThrottle = parentalControl.maxThrottle;
    }
    
    // ------------------- Gear logic -------------------
    int upState = digitalRead(GEAR_UP_PIN);
    int downState = digitalRead(GEAR_DOWN_PIN);
    
    if (lastUpState == HIGH && upState == LOW) { 
      int targetGear = gear + 1;
      
      // Apply parental control gear limit
      if (parentalControl.enabled && targetGear > parentalControl.maxGear) {
        targetGear = parentalControl.maxGear;
        Serial.println("Gear shift blocked by parental control");
      }
      
      if (targetGear <= 5 && targetGear != gear) {
        gear = targetGear;
        gearJustShifted = true;
        shiftTime = millis();
        rpmSignal *= shiftDropFactor;  // drop RPM immediately
      }
    }
    
    if (lastDownState == HIGH && downState == LOW) {
      if (gear > 0) {
        gear--;
      }
    }
    
    lastUpState = upState;
    lastDownState = downState;
    
    // Update display when gear changes
    if (gear != lastGear) {
      updateGearDisplay();
      lastGear = gear;
    }
    
    // ------------------- RPM model -------------------
    if (gearJustShifted && millis() - shiftTime < shiftDropDuration) {
      analogWrite(rpmOutPin, (int)rpmSignal);
    } else {
      gearJustShifted = false;
      
      float targetRPM = map(mappedThrottle, 0, 100, 0, gearMaxOutput[gear]);
      if (rpmSignal < targetRPM) {
        rpmSignal += 1.2;   // ramp up
        if (rpmSignal > targetRPM) rpmSignal = targetRPM;
      } else if (rpmSignal > targetRPM) {
        rpmSignal -= 0.7;   // ramp down naturally
        if (rpmSignal < targetRPM) rpmSignal = targetRPM;
      }
      
      // Apply brake force
      if (mappedBrake > 0) {
        float brakeEffect = mappedBrake * 0.025;
        rpmSignal -= brakeEffect;
        if (rpmSignal < 0) rpmSignal = 0;
      }
      
      // Add jitter near redline
      if (gear > 0) {
        int maxOut = gearMaxOutput[gear];
        if (rpmSignal > maxOut * 0.9) {  
          float jitter = random(-3, 4);  
          rpmSignal += jitter;
          if (rpmSignal > maxOut) rpmSignal = maxOut;
          if (rpmSignal < maxOut * 0.85) rpmSignal = maxOut * 0.85;
        }
      }
      
      analogWrite(rpmOutPin, (int)rpmSignal);
    }
    
    // Update shared sensor data
    sensorData.steering = steeringValue;
    sensorData.throttle = mappedThrottle;
    sensorData.brake = mappedBrake;
    sensorData.currentGear = gear;
    sensorData.rpm = rpmSignal;
    sensorData.timestamp = millis();
    sensorData.dataReady = true;
    
    // Check for alerts
    checkAndSendAlerts();
    
    // Debug print occasionally
    static int debugCounter = 0;
    if (++debugCounter >= 50) { // Every 50 cycles
      debugCounter = 0;
      debugPrint();
    }
    
    // Task delay - 50Hz (20ms)
    vTaskDelay(configTICK_RATE_HZ / 50);
    taskYIELD();
  }
}

// ================= FREERTOS TASK 2: COMMUNICATION TASK =================
void communicationTaskFunction(void *pvParameters) {
  for(;;) {
    // Process MQTT messages
    mqttClient.loop();
    
    // Process outgoing MQTT queue
    while (mqttQueueCount > 0) {
      mqttClient.publish(mqttMessageQueue[mqttQueueTail].topic, 
                        mqttMessageQueue[mqttQueueTail].payload);
      mqttQueueTail = (mqttQueueTail + 1) % MQTT_QUEUE_SIZE;
      mqttQueueCount--;
    }
    
    // Publish telemetry if sensor data is ready
    if (sensorData.dataReady) {
      publishTelemetry();
      sensorData.dataReady = false; // Mark as consumed
    }
    
    // Task delay - 20Hz (50ms)
    vTaskDelay(configTICK_RATE_HZ / 20);
    taskYIELD();
  }
}

// ------------------- Alert System -------------------
void checkAndSendAlerts() {
  unsigned long currentTime = millis();
  
  // Check if we're in high gear + high throttle mode
  bool currentlyInHighMode = (gear >= HIGH_GEAR_THRESHOLD && mappedThrottle >= HIGH_THROTTLE_THRESHOLD);
  
  if (currentlyInHighMode) {
    if (!inHighGearThrottleMode) {
      // Just entered high gear+throttle mode
      inHighGearThrottleMode = true;
      highGearThrottleStartTime = currentTime;
    } else {
      // Check if we've been in this mode long enough
      unsigned long durationInHighMode = currentTime - highGearThrottleStartTime;
      
      if (durationInHighMode >= SUSTAINED_DURATION && 
          (currentTime - lastSustainedAlert) >= ALERT_COOLDOWN) {
        
        // Send sustained driving alert
        String alertMessage = "Sustained aggressive driving detected! Gear: " + 
                            String(gear) + ", Throttle: " + String(mappedThrottle) + 
                            "% for " + String(durationInHighMode / 1000) + " seconds";
        
        sendAlert("SUSTAINED_AGGRESSIVE", alertMessage, 2);
        lastSustainedAlert = currentTime;
      }
    }
  } else {
    // No longer in high gear+throttle mode
    inHighGearThrottleMode = false;
    highGearThrottleStartTime = 0;
  }
}

void sendAlert(String alertType, String message, int severity) {
  StaticJsonDocument<300> alertDoc;
  alertDoc["alert_type"] = alertType;
  alertDoc["message"] = message;
  alertDoc["severity"] = severity;
  alertDoc["timestamp"] = millis();
  alertDoc["driver"] = "driver1";
  
  String alertPayload;
  serializeJson(alertDoc, alertPayload);
  
  enqueueMQTTMessage("car/driver1/alerts", alertPayload);
  
  Serial.println("ALERT SENT: " + alertType + " - " + message);
}

// ------------------- Queue Functions -------------------
bool enqueueMQTTMessage(String topic, String payload) {
  if (mqttQueueCount >= MQTT_QUEUE_SIZE) {
    return false; // Queue full
  }
  
  mqttMessageQueue[mqttQueueHead].topic = topic;
  mqttMessageQueue[mqttQueueHead].payload = payload;
  mqttQueueHead = (mqttQueueHead + 1) % MQTT_QUEUE_SIZE;
  mqttQueueCount++;
  return true;
}

// ------------------- MQTT Publishing Functions -------------------
void publishTelemetry() {
  // Queue all telemetry messages
  enqueueMQTTMessage("car/driver1/steering", String(sensorData.steering));
  enqueueMQTTMessage("car/driver1/throttle", String(sensorData.throttle));
  enqueueMQTTMessage("car/driver1/brake", String(sensorData.brake));
  enqueueMQTTMessage("car/driver1/gear", String(sensorData.currentGear));
  enqueueMQTTMessage("car/driver1/rpm", String(sensorData.rpm));
  
  // Publish parental control status
  StaticJsonDocument<200> pcDoc;
  pcDoc["enabled"] = parentalControl.enabled;
  pcDoc["max_gear"] = parentalControl.maxGear;
  pcDoc["max_throttle"] = parentalControl.maxThrottle;
  
  String pcStatus;
  serializeJson(pcDoc, pcStatus);
  enqueueMQTTMessage("car/driver1/parental_control_status", pcStatus);
}

// ------------------- Parental Control Handler -------------------
void handleParentalControlUpdate(String payload) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, payload);
  
  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Update parental control settings
  if (doc.containsKey("enabled")) {
    parentalControl.enabled = doc["enabled"];
  }
  if (doc.containsKey("max_gear")) {
    parentalControl.maxGear = doc["max_gear"];
  }
  if (doc.containsKey("max_throttle")) {
    parentalControl.maxThrottle = doc["max_throttle"];
  }
  
  // Force gear down if current gear exceeds new limit
  if (parentalControl.enabled && gear > parentalControl.maxGear) {
    gear = parentalControl.maxGear;
    updateGearDisplay();
    Serial.println("Gear forced down due to parental control");
  }
  
  Serial.println("Parental Control Settings Updated:");
  Serial.println("Enabled: " + String(parentalControl.enabled));
  Serial.println("Max Gear: " + String(parentalControl.maxGear));
  Serial.println("Max Throttle: " + String(parentalControl.maxThrottle));
  
  // Show parental control status on LED matrix briefly
  showParentalControlStatus();
}

// ------------------- LED Matrix Functions -------------------
void updateGearDisplay() {
  matrix.beginDraw();
  matrix.clear(); // Clear the display
  matrix.stroke(0xFFFFFFFF);
  matrix.textFont(Font_5x7);
  matrix.beginText(4, 1, 0xFFFFFF); // Centered position
  
  if (gear == 0) {
    matrix.println("N"); // Show "N" for Neutral
  } else {
    matrix.println(String(gear)); // Show gear number 1-5
  }
  
  matrix.endText();
  matrix.endDraw();
}

void showParentalControlStatus() {
  matrix.beginDraw();
  matrix.clear();
  matrix.stroke(0xFFFFFFFF);
  matrix.textFont(Font_4x6);
  matrix.beginText(1, 1, 0xFFFFFF);
  
  if (parentalControl.enabled) {
    matrix.println("PC"); // Show "PC" for Parental Control
  } else {
    matrix.println("FR"); // Show "FR" for Free (no restrictions)
  }
  
  matrix.endText();
  matrix.endDraw();
  
  vTaskDelay(configTICK_RATE_HZ * 2); // Show for 2 seconds using FreeRTOS delay
  updateGearDisplay(); // Return to gear display
}

// ------------------- Encoder Interrupt Service Routine -------------------
void ISR_encoderChange() {
  if ((millis() - last_time) < 10)  // debounce time is 10ms
    return;

  if (digitalRead(DT_PIN) == HIGH) {
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    encoderCounter--;
    direction = DIRECTION_CCW;
  } else {
    // the encoder is rotating in clockwise direction => increase the counter
    encoderCounter++;
    direction = DIRECTION_CW;
  }

  last_time = millis();
}

// ------------------- Debug Print -------------------
void debugPrint() {
  Serial.print("Steering: "); Serial.print(steeringValue);
  Serial.print(" | Throttle: "); Serial.print(mappedThrottle);
  Serial.print("% | Brake: "); Serial.print(mappedBrake);
  Serial.print("% | Gear: "); Serial.print(gear);
  Serial.print(" | RPM Signal: "); Serial.print(rpmSignal);
  
  if (parentalControl.enabled) {
    Serial.print(" | PC: ON (MaxGear:");
    Serial.print(parentalControl.maxGear);
    Serial.print(", MaxThrottle:");
    Serial.print(parentalControl.maxThrottle);
    Serial.print("%)");
  }

  // Show alert status
  if (inHighGearThrottleMode) {
    unsigned long duration = millis() - highGearThrottleStartTime;
    Serial.print(" | HIGH MODE: ");
    Serial.print(duration / 1000);
    Serial.print("s");
  }
  
  Serial.println();
}