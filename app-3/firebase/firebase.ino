
#include <Arduino.h>
#include <ArduinoJson.h>
#if defined(ESP32) 
#include <WiFi.h>
#endif
#include <Wire.h>
// #include <FirebaseESP32.h>
#include <Firebase_ESP_Client.h>
// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>


const char *ssid = "SLT-4G_163BEA";
const char *password = "751FCEED";

#define FIREBASE_HOST "https://elec-research-0-default-rtdb.asia-southeast1.firebasedatabase.app"// "<YOUR_FIREBASE_HOST>"
#define FIREBASE_AUTH "AIzaSyCvhDMEJ9gnl8lqyV282-8pdHOVmZqyVPs" //"<YOUR_FIREBASE_AUTH>"

#define USER_EMAIL "device@gmail.com"
#define USER_PASSWORD "123456"


FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

bool wifiConnected = false;

unsigned long sendDataPrevMillis = 0;

unsigned long count = 0;

// Constants for voltage measurement
// const int analogPin = 36; // Pin connected to the voltage divider
// const float referenceVoltage = 3.3; // Actual reference voltage of your board
// const float dividerRatio = 4/37; // Voltage divider ratio (100kΩ and 10kΩ resistors)

// // Battery and SOC parameters
// const float batteryCapacity = 300.0; // Capacity in milliamp-hours (mAh)
// // float soc = 100.0; // Initial SOC in percentage
// unsigned long lastTime = 0; // Last time the SOC was updated

// // Fixed current value (in mA)
// // Set this to the average current draw of your system or the charging current
// const float fixedCurrent = -50.0; // Example: discharging at 50mA

// const int analogPin = 34; // Change based on your ESP32's ADC pin
// const float R1 = 100000.0;
// const float R2 = 10000.0;
// const float Vref = 3.3; // Reference voltage for ESP32 ADC
// const int ADCresolution = 4095; // 12-bit ADC
// const float Rshunt = 0.1; // Shunt resistor value in ohms (change as needed)

#define RGB_R 14
#define RGB_G 12
#define RGB_B 13

#define SW3_PIN 32;

float soc = 100.0; // State of Charge percentage
float fullChargeCapacity = 300.0; // 300Ah, full charge capacity of your battery
float current; // Current in Amperes
unsigned long previousMillis = 0; // to store the last time update
unsigned long interval = 3600000; // interval at which to measure (1 hour)

int red = 0;
int blue = 0;
int green = 0;

int mode = 0;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  connectToWiFi();

  config.api_key = FIREBASE_AUTH;
  config.database_url = FIREBASE_HOST;
  fbdo.setBSSLBufferSize(2048 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);

    // Or use legacy authenticate method
    // config.database_url = DATABASE_URL;
    // config.signer.tokens.legacy_token = "<database secret>";
      /* Assign the api key (required) */
  config.api_key = FIREBASE_AUTH;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
  Firebase.setDoubleDigits(5);

  // analogReadResolution(12); // Set the resolution to 12 bits for ESP32
  initializeLEDs();
  pinMode(SW3_PIN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // delay(1000);
  // json.set("/data", sdata);
  // json.set("/voltage", sdata);
  int m = random(0, 2000);
  Serial.println(m);
  writeFirebase(m);
  delay(1000);

  // if(SW3_PIN == HIGH){
  //   mode++;
  //   if(mode > 6){
  //     mode = 0;
  //   }
  // }
  // rgbLighting(mode);
  // delay(1000);
  readFirebase();
  rgb(red, green, blue);
  // // Firebase.RTDB.updateNode(firebaseData, "/sensor", json);
  // Serial.printf("Update json... %s\n\n", Firebase.RTDB.updateNode(&fbdo, "/battery/", &json) ? "ok" : fbdo.errorReason().c_str());
}


void rgb(int r, int g, int b)
{
  analogWrite(RGB_R, r);
  analogWrite(RGB_G, g);
  analogWrite(RGB_B, b);
}

void initializeLEDs()
{
  Serial.println("Initializing LEDs");
  pinMode(RGB_R, OUTPUT);
  pinMode(RGB_G, OUTPUT);
  pinMode(RGB_B, OUTPUT);
  // pinMode(IL1_PIN, OUTPUT);
  // pinMode(IL2_PIN, OUTPUT);
  // pinMode(IL3_PIN, OUTPUT);
  // pinMode(IL4_PIN, OUTPUT);
}
void connectToWiFi()
{
  // functionState = true;
  // Check if already connected
  if (wifiConnected)
  {
    Serial.println("Already connected to Wi-Fi.");
    return;
  }

  Serial.println("Connecting to Wi-Fi...");

  WiFi.begin(ssid, password); // Connect to Wi-Fi network

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting...");
  }

  wifiConnected = true;
  Serial.println("Connected to Wi-Fi");
  // functionState = false;
}
//---------------------------------------------------------------------------------
// RGB LED
void rgbLighting(int mode)
{
  switch (mode)
  {
  case 0:
    rgb(255, 255, 255);
    break;
  case 1:
    rgb(255, 0, 0);
    break;
  case 2:
    rgb(0, 255, 0);
    break;
  case 3:
    rgb(0, 0, 255);
    break;
  case 4:
    sos(true);
    break;
  case 5:
    sos(false);
    rgb(255, 255, 255);
    break;
  case 6:
    rgb(0, 0, 0);
    break;

  default:
    rgb(255, 255, 255); break;
  }
}

void sos(bool state)
{
  while (state)
  {
    rgb(255, 255, 255);
    delay(300);
    rgb(255, 255, 255);
    delay(300);
    rgb(255, 255, 255);
    delay(1000);
    rgb(255, 255, 255);
    delay(300);
    rgb(255, 255, 255);
    delay(300);
    rgb(255, 255, 255);
    delay(300);
  }
}


void writeFirebase(int socs){
  
  FirebaseJson json;
  json.set("/step", socs);
  delay(1000);

  // Firebase.RTDB.updateNode(firebaseData, "/sensor", json);
  Serial.printf("Update json... %s\n\n", Firebase.RTDB.updateNode(&fbdo, "/count/", &json) ? "ok" : fbdo.errorReason().c_str());
}

void readFirebase(){
  
  if (Firebase.RTDB.getInt(&fbdo, "/led/red")) {
    if (fbdo.dataType() == "int") {
      red = fbdo.intData();
      Serial.println(red);
    }
  }
  else {
    Serial.println(fbdo.errorReason());
  }
  if (Firebase.RTDB.getInt(&fbdo, "/led/green")) {
    if (fbdo.dataType() == "int") {
      green = fbdo.intData();
      Serial.println(red);
    }
  }
  else {
    Serial.println(fbdo.errorReason());
  }
  if (Firebase.RTDB.getInt(&fbdo, "/led/blue")) {
    if (fbdo.dataType() == "int") {
      blue = fbdo.intData();
      Serial.println(red);
    }
  }
  else {
    Serial.println(fbdo.errorReason());
  }
    
    // if (Firebase.RTDB.getFloat(&fbdo, "/test/float")) {
    //   if (fbdo.dataType() == "float") {
    //     floatValue = fbdo.floatData();
    //     Serial.println(floatValue);
    //   }
    // }
    // else {
    //   Serial.println(fbdo.errorReason());
    // }
}
