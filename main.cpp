#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <math.h>

#define API_KEY "----"
#define DATABASE_URL "----" 

#define WIFI_SSID "----"
#define WIFI_PASSWORD "----"

#define USERNAME "----"
#define PASSWORD "----"

#define AIR780E_RX 4   
#define AIR780E_TX 2   

#define ZMPT_PIN 34
#define ACS_PIN 32
#define TEMP_PIN 26

#define BUZZER_PIN 5

#define SERVO1_PIN 12
#define SERVO2_PIN 13

#define SAFE_VOLTAGE 240.0
#define VOLT_CALIBRATION 312.0
#define CURRENT_CALIBRATION 0.066
#define CURRENT_THRESHOLD 0.1


String alertPhoneNumbers[] = {
  "",
  "",
  "",
  "",
  ""
};

const int numPhoneNumbers = 5;

LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
Servo servo1;
Servo servo2;
HardwareSerial SerialAT(2);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

float currentOffset = 1.65;
int displayMode = 0;
bool mcbTripped = false;
unsigned long tripTime = 0;
bool smsAlertSent = false;
bool modemReady = false;
unsigned long sendDataPrevMillis = 0;
int tripCount = 0;

void initializeModem();
void sendEmergencyAlert(float voltage, float current, float power, float temp);
void sendSMS(String phoneNumber, String message);
bool waitForResponse(String expected, int timeout);
void clearBuffer();
void printBuffer();
void tripMCB();
void resetMCB();
float calibrateCurrentOffset();
float readACVoltage();
float readACCurrent();
void connectToWiFi();
void sendToFirebase(float voltage, float current, float power, float temp, String status);
void logTripHistory(float voltage, float current, float power, float temp);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== Power Monitor with Firebase & SMS ===");

  connectToWiFi();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  
  auth.user.email = USERNAME;
  auth.user.password = PASSWORD;
  
  config.token_status_callback = tokenStatusCallback;
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  Serial.println("Waiting for Firebase to be ready...");
  
  unsigned long firebaseWait = millis();
  while (!Firebase.ready() && millis() - firebaseWait < 10000) {
    delay(100);
    Serial.print(".");
  }
  
  if (Firebase.ready()) {
    Serial.println("\n✅ Firebase ready!");
  } else {
    Serial.println("\n❌ Firebase connection failed!");
  }

  initializeModem();

  analogSetPinAttenuation(ZMPT_PIN, ADC_11db);
  analogSetPinAttenuation(ACS_PIN, ADC_11db);

  pinMode(ZMPT_PIN, INPUT);
  pinMode(ACS_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(0);
  servo2.write(0);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  sensors.begin();

  Serial.print("Temperature sensors found: ");
  Serial.println(sensors.getDeviceCount());

  if (sensors.getDeviceCount() == 0) {
    Serial.println("ERROR: No DS18B20 found!");
    lcd.setCursor(0, 1);
    lcd.print("Temp sensor ERR");
    delay(3000);
  }

  lcd.setCursor(0, 1);
  lcd.print("Calibrating...");
  delay(1500);

  currentOffset = calibrateCurrentOffset();

  lcd.clear();
  lcd.print("Ready!");
  Serial.println("System Ready!");
  delay(800);
}

void loop() {
  float voltage = readACVoltage();
  float current = readACCurrent();

  if (current < CURRENT_THRESHOLD) current = 0.0;
  float power = voltage * current;

  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);

  if (temperature == -127.0 || temperature < -50 || temperature > 125) {
    temperature = 0.0;
    Serial.println("⚠️ Temperature sensor error!");
  }

  String status = mcbTripped ? "OVERTRIPPED" : "NORMAL";

  Serial.print("V: ");
  Serial.print(voltage, 1);
  Serial.print(" V | I: ");
  Serial.print(current, 2);
  Serial.print(" A | P: ");
  Serial.print(power, 1);
  Serial.print(" W | T: ");
  Serial.print(temperature, 1);
  Serial.print(" C | Status: ");
  Serial.println(status);

  if (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0) {
    sendDataPrevMillis = millis();
    sendToFirebase(voltage, current, power, temperature, status);
  }

  if (voltage > SAFE_VOLTAGE && !mcbTripped) {
    Serial.println("OVERVOLTAGE DETECTED! TRIPPING MCB...");

    for (int i = 0; i < 5; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
    }

    tripMCB();
    mcbTripped = true;
    tripTime = millis();
    smsAlertSent = false;
    tripCount++;

    logTripHistory(voltage, current, power, temperature);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("!! OVERVOLTAGE !!");
    lcd.setCursor(0, 1);
    lcd.print("MCB TRIPPED");

    if (modemReady) {
      sendEmergencyAlert(voltage, current, power, temperature);
    }

    delay(3000);
  }

  if (mcbTripped && (millis() - tripTime >= 5000)) {
    Serial.println("Resetting MCB...");
    resetMCB();
    mcbTripped = false;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Resetting MCB...");
    delay(1500);
  }

  if (!mcbTripped) {
    lcd.clear();
    if (displayMode == 0) {
      lcd.setCursor(0, 0);
      lcd.print("V:");
      lcd.print(voltage, 1);
      lcd.print("V");
      lcd.setCursor(0, 1);
      lcd.print("I:");
      lcd.print(current, 2);
      lcd.print("A");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("P:");
      lcd.print(power, 1);
      lcd.print("W");
      lcd.setCursor(0, 1);
      lcd.print("T:");
      lcd.print(temperature, 1);
      lcd.print("C");
    }
  }

  static unsigned long lastSwitch = 0;
  if (millis() - lastSwitch > 3000) {
    displayMode = !displayMode;
    lastSwitch = millis();
  }

  delay(1000);
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    lcd.setCursor(0, 1);
    lcd.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");
    delay(1000);
  } else {
    Serial.println("\n❌ WiFi failed!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed!");
    delay(2000);
  }
}

void sendToFirebase(float voltage, float current, float power, float temp, String status) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    connectToWiFi();
    return;
  }

  if (Firebase.ready()) {
    String basePath = "/powerMonitor/current/";
    
    Firebase.RTDB.setFloat(&fbdo, basePath + "voltage", voltage);
    Firebase.RTDB.setFloat(&fbdo, basePath + "current", current);
    Firebase.RTDB.setFloat(&fbdo, basePath + "power", power);
    Firebase.RTDB.setFloat(&fbdo, basePath + "temperature", temp);
    Firebase.RTDB.setString(&fbdo, basePath + "status", status);
    Firebase.RTDB.setInt(&fbdo, basePath + "tripCount", tripCount);
    Firebase.RTDB.setTimestamp(&fbdo, basePath + "timestamp");

    Serial.println("📤 Data sent to Firebase");
    
    Serial.print("  V: "); Serial.print(voltage);
    Serial.print(" | I: "); Serial.print(current);
    Serial.print(" | P: "); Serial.println(power);
  } else {
    Serial.println("❌ Firebase not ready");
  }
}

void logTripHistory(float voltage, float current, float power, float temp) {
  if (Firebase.ready()) {
    String path = "/powerMonitor/tripHistory/" + String(millis());
    
    FirebaseJson json;
    json.set("voltage", voltage);
    json.set("current", current);
    json.set("power", power);
    json.set("temperature", temp);
    json.set("tripNumber", tripCount);
    json.set("timestamp/.sv", "timestamp");
    
    if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
      Serial.println("✅ Trip logged to history");
    } else {
      Serial.println("❌ Failed to log trip");
    }
  }
}


void initializeModem() {
  Serial.println("Initializing Air780E modem...");
  Serial.println("Using GPIO4(RX) and GPIO2(TX)...");

  SerialAT.begin(115200, SERIAL_8N1, AIR780E_RX, AIR780E_TX);
  delay(3000);

  for (int attempt = 0; attempt < 5; attempt++) {
    Serial.print("Attempt ");
    Serial.print(attempt + 1);
    Serial.print(": Sending AT...");

    SerialAT.println("AT");
    delay(1000);

    if (waitForResponse("OK", 2000)) {
      Serial.println(" ✅ Response received!");

      Serial.println("Setting SMS mode...");
      SerialAT.println("AT+CMGF=1");
      delay(500);
      clearBuffer();

      Serial.println("Checking SIM...");
      SerialAT.println("AT+CPIN?");
      delay(500);
      printBuffer();

      Serial.println("Checking signal...");
      SerialAT.println("AT+CSQ");
      delay(500);
      printBuffer();

      modemReady = true;
      Serial.println("✅ Modem ready for SMS alerts");
      return;
    } else {
      Serial.println(" ❌ No response");
      clearBuffer();
    }
  }

  Serial.println("⚠️ Modem not responding - SMS alerts disabled");
  modemReady = false;
}

void sendEmergencyAlert(float voltage, float current, float power, float temp) {
  if (!modemReady || smsAlertSent) return;

  Serial.println("\n📱 SENDING EMERGENCY SMS ALERTS...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sending SMS...");

  String message = "*** POWER ALERT ***\r\n";
  message += "OVERVOLTAGE DETECTED!\r\n";
  message += "Voltage: " + String(voltage, 1) + "V\r\n";
  message += "Current: " + String(current, 2) + "A\r\n";
  message += "Power: " + String(power, 1) + "W\r\n";
  message += "Temp: " + String(temp, 1) + "C\r\n";
  message += "Trip #" + String(tripCount) + "\r\n";
  message += "MCB TRIPPED!";

  for (int i = 0; i < numPhoneNumbers; i++) {
    if (alertPhoneNumbers[i].length() > 0) {
      Serial.print("Sending to: ");
      Serial.println(alertPhoneNumbers[i]);
      sendSMS(alertPhoneNumbers[i], message);
      delay(2000);
    }
  }

  smsAlertSent = true;
  Serial.println("✅ Emergency alerts sent!");

  lcd.setCursor(0, 1);
  lcd.print("SMS Sent!");
  delay(2000);
}

void sendSMS(String phoneNumber, String message) {
  SerialAT.println("AT+CMGF=1");
  delay(300);
  while (SerialAT.available()) SerialAT.read();

  SerialAT.print("AT+CMGS=\"");
  SerialAT.print(phoneNumber);
  SerialAT.println("\"");
  delay(1000);

  while (SerialAT.available()) {
    char c = SerialAT.read();
    Serial.write(c);
  }

  SerialAT.print(message);
  delay(500);
  SerialAT.write(26);

  unsigned long start = millis();
  while (millis() - start < 10000) {
    if (SerialAT.available()) {
      String resp = SerialAT.readString();
      Serial.print(resp);
      if (resp.indexOf("OK") >= 0 || resp.indexOf("+CMGS:") >= 0) {
        Serial.println(" ✅ Sent");
        return;
      }
      if (resp.indexOf("ERROR") >= 0) {
        Serial.println(" ❌ Failed");
        return;
      }
    }
  }
  Serial.println(" ⏱️ Timeout");
}

bool waitForResponse(String expected, int timeout) {
  unsigned long start = millis();
  String response = "";

  while (millis() - start < timeout) {
    if (SerialAT.available()) {
      response += (char)SerialAT.read();
      if (response.indexOf(expected) >= 0) {
        return true;
      }
    }
  }
  return false;
}

void clearBuffer() {
  while (SerialAT.available()) {
    SerialAT.read();
  }
}

void printBuffer() {
  while (SerialAT.available()) {
    Serial.write(SerialAT.read());
  }
  Serial.println();
}



void tripMCB() {
  Serial.println("Moving servos to TRIP position (180°)...");
  servo1.write(180);
  servo2.write(180);
  delay(1000);
}

void resetMCB() {
  Serial.println("Moving servos to RESET position (0°)...");
  servo1.write(0);
  servo2.write(0);
  delay(1000);
}

float calibrateCurrentOffset() {
  const int samples = 500;
  float sum = 0.0;
  for (int i = 0; i < samples; i++) {
    float sensor = (analogRead(ACS_PIN) / 4095.0) * 3.3;
    sum += sensor;
    delay(2);
  }
  return sum / samples;
}

float readACVoltage() {
  const int samples = 500;
  float sum = 0.0;
  float zmptOffset = 1.65;
  for (int i = 0; i < samples; i++) {
    float sensor = (analogRead(ZMPT_PIN) / 4095.0) * 3.3;
    float centered = sensor - zmptOffset;
    sum += centered * centered;
    delayMicroseconds(200);
  }
  float mean = sum / samples;
  float rms = sqrt(mean);
  return rms * VOLT_CALIBRATION;
}

float readACCurrent() {
  const int samples = 500;
  float sum = 0.0;
  for (int i = 0; i < samples; i++) {
    float sensor = (analogRead(ACS_PIN) / 4095.0) * 3.3;
    float centered = sensor - currentOffset;
    sum += centered * centered;
    delayMicroseconds(200);
  }
  float mean = sum / samples;
  float rms = sqrt(mean);
  return rms / CURRENT_CALIBRATION;
}
