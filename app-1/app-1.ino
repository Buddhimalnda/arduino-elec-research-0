#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <Arduino.h>
// #include <FirebaseESP32.h>
#include <Firebase_ESP_Client.h>
// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>
//---------------------------------------------------------------------------------
// Define GPIO pins
// Inputs
#define GPS_TX 16      // Neo 6M GPS Module TX pin
#define GPS_RX 17      // Neo 6M GPS Module RX pin
#define MPU_SDA 21     // MPU6050 SDA pin
#define MPU_SCL 22     // MPU6050 SCL pin
#define BATTERY_PIN 25//@me // GPIO pin for battery level indicator (replace XX with the actual pin number)
#define SW1_PIN 2 //2     // GPIO pin for the first switch
#define SW2_PIN 4 //4     // GPIO pin for the second switch
#define SW3_PIN 34 //5     // GPIO pin for the third switch

// Outputs
#define SD_CS 19// @me      // GPIO pin for MicroSD card module CS (replace XX with the actual pin number)
#define RGB_RED 12     // GPIO pin for RGB LED Red channel
#define RGB_GREEN 13   // GPIO pin for RGB LED Green channel
#define RGB_BLUE 14    // GPIO pin for RGB LED Blue channel
#define IL1_PIN 15 // GPIO pin for the first indicator LED
#define IL2_PIN 26 // GPIO pin for the second indicator LED
#define IL3_PIN 27 // GPIO pin for the third indicator LED
#define IL4_PIN 32 // GPIO pin for the fourth indicator LED
#define IL5_PIN 33 // GPIO pin for the fifth indicator LED
// avl - 2,4,5,18,23,35, (36, 39)-input
//---------------------------------
// #define BLI <BLI_GPIO_PIN>
//---------------------------------------------------------------------------------
// WiFi credentials
const char *ssid = "SLT-4G_163BEA";
const char *password = "751FCEED";
//---------------------------------------------------------------------------------
// Firebase credentials
#define FIREBASE_HOST "https://elec-research-0.firebaseapp.com"// "<YOUR_FIREBASE_HOST>"
#define FIREBASE_AUTH "AIzaSyCvhDMEJ9gnl8lqyV282-8pdHOVmZqyVPs"//"<YOUR_FIREBASE_AUTH>"

#define USER_EMAIL "device@gmail.com"
#define USER_PASSWORD "123456"


FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;
//---------------------------------------------------------------------------------
// MPU6050 and GPS objects
MPU6050 accelgyro;
//---------------------------------------------------------------------------------
// Initialize Neo 6M GPS module object here
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
String dataSetOFLastGPS = "";
//---------------------------------------------------------------------------------
// Initialize  MicroSD Card module object here
File myFile;

//---------------------------------------------------------------------------------
// Define other global variables
//---------------------------------
// for step counter
float lastMagnitude = 0;
float stepThreshold = 1.2; // Threshold for step detection, adjust as needed
unsigned int stepCount = 0;
unsigned long lastStepTime = 0;
String mpuDataSet = "";
//---------------------------------
// switch states variables
bool sw1State = false;
bool sw2State = false;
bool sw3State = false;
//---------------------------------
// main led mode
int mainLedMode = 0;
//---------------------------------
// wifi connection
bool wifiConnected = false;
//---------------------------------
// Function state
bool functionState = false;
bool storeStatus = false;
bool isSDCardInitialized = false;
//---------------------------------
// battery level
float batteryLevel = 0;
void setup()
{
  Serial.begin(115200);

  // Initialize switches as inputs
  pinMode(SW1_PIN, INPUT);
  pinMode(SW2_PIN, INPUT);
  pinMode(SW3_PIN, INPUT);
  // ... initialize other switches

  // Initialize LEDs as outputs
  pinMode(IL1_PIN, OUTPUT);
  pinMode(IL2_PIN, OUTPUT);
  pinMode(IL3_PIN, OUTPUT);
  pinMode(IL4_PIN, OUTPUT);
  pinMode(IL5_PIN, OUTPUT);
  // ... initialize other LEDs
  pinMode(BATTERY_PIN, INPUT);

  // Initialize MPU6050
  // try
  // {
  Wire.begin();
  accelgyro.initialize();
  // }
  // catch (Exception e)
  // {
  //   Serial.println("Error MPU6050: " + e);
  // }

  // Initialize Neo 6M GPS module
  // try
  // {
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  // }
  // catch (Exception e)
  // {
  //   Serial.println("Error GPS: " + e);
  // }
  // Initialize SD card
  if (!SD.begin(10))
  {
    Serial.println("Initialization failed!");
    return;
  }
  else
  {
    isSDCardInitialized = true;
    Serial.println("Initialization done.");
  }
  // Initialize WiFi and Firebase
  // Connect to Wi-Fi when the sketch starts
  connectToWiFi();
  // Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  connectFirebase();
}

void loop()
{
  //---------------------------------------------------------------------------------
  // Check switch states and perform actions
  // Read the state of SW2
  if (digitalRead(SW2_PIN) == HIGH)
  {
    sw2State = !sw2State;
  }

  // Read the state of SW3
  if (digitalRead(SW3_PIN) == HIGH)
  {
    mainLedMode++;
  }

  rgbLighting(mainLedMode);

  if (sw2State)
  { // SW2 is pressed
    Serial.println("SW2 pressed. Initiating Wi-Fi connection...");

    // Attempt to connect to Wi-Fi
    connectToWiFi();

    // Wait for a few seconds (optional)
    delay(5000);
  }
  //---------------------------------------------------------------------------------
  if (wifiConnected)
  {
    // connect to firebase
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    // store data to firebase
    // Firebase.set("/data", "Hello World!");
    // read data from firebase
    // Serial.println(Firebase.getString("/data"));
    // disconnect from firebase
    Firebase.end();
  }
  //---------------------------------------------------------------------------------
  // Check battery level
  //---------------------------------------------------------------------------------
  // Check step counter
  if (stepCounter())
  {
    Serial.println("Step detected");
    // store data to sd card
    DynamicJsonDocument doc();
    doc["time"] = millis(); // for now just use millis
    doc["gps"] = gpsData();
    doc["ble"] = batteryLevelFun();
    doc["mpu"] = mpuDataSet;
    String jsonData = getJsonString(doc);
    if (storeLocal(jsonData))
    {
      Serial.println("Data stored to SD card");
    }
    else
    {
      Serial.println("Error storing data to SD card");
    }
  }
  //---------------------------------------------------------------------------------
  // Check for BLE connection
  // Check for WiFi connection
  // Check for Firebase connection
  //---------------------------------------------------------------------------------
  // Update LEDs based on states
  if (sw1State)
  {
    indicatorLight(IL1_PIN, 's');
  }
  else if (functionState)
  {
    indicatorLight(IL1_PIN, 'b');
  }
  else
  {
    indicatorLight(IL1_PIN, 'o');
  }
  if (wifiConnected)
  {
    indicatorLight(IL2_PIN, 's');
  }
  else
  {
    indicatorLight(IL2_PIN, 'o');
  }
  if (isSDCardInitialized)
  {
    indicatorLight(IL3_PIN, 's');
  }
  else if (isSDCardInitialized && storeStatus)
  {
    indicatorLight(IL3_PIN, 'b');
  }
  else
  {
    indicatorLight(IL3_PIN, 'o');
  }

  //---------------------------------------------------------------------------------
  // Read sensors
  // Log data to SD card
  // Sync with Firebase if connected
}
//---------------------------------------------------------------------------------
// connect to wifi
void connectToWiFi()
{
  functionState = true;
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
  functionState = false;
}
//---------------------------------------------------------------------------------
// store data to sd card
bool storeLocal(String data)
{
  functionState = true;
  storeStatus = true;
  myFile = SD.open("test.txt", FILE_WRITE);

  if (myFile)
  {
    myFile.println(data);
    myFile.close();
    Serial.println("Done writing.");
    return true;
  }
  else
  {
    Serial.println("Error opening file.");
    return false;
  }
  functionState = false;
  storeStatus = false;
  // Serial.println("Error file: " + e);
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
    rgb(255, 255, 255) break;
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

void rgb(int r, int g, int b)
{
  analogWrite(RGB_RED, r);
  analogWrite(RGB_GREEN, g);
  analogWrite(RGB_BLUE, b);
}
//---------------------------------------------------------------------------------
String getJsonString(const JsonDocument &doc)
{
  String output;
  serializeJson(doc, output); // Serialize the JSON object to a String
  return output;
}
//---------------------------------------------------------------------------------
// indicator light
void indicatorLight(int light, char state)
{
  // state: b - blink, s - solid, o - off
  switch (state)
  {
  case 'b':
    digitalWrite(light, HIGH);
    delay(500);
    digitalWrite(light, LOW);
    delay(500);
    break;
  case 's':
    digitalWrite(light, HIGH);
    break;
  case 'o':
    digitalWrite(light, LOW);
    break;
  default:
    digitalWrite(light, LOW);
    break;
  }
}
//---------------------------------------------------------------------------------
// gps data
String gpsData()
{
  String jsonData = "";
  while (GPSSerial.available() > 0)
  {
    char c = GPSSerial.read();
    gps.encode(c);
  }
  DynamicJsonDocument doc(1024);
  if (gps.location.isUpdated())
  {
    // GPS data is available
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    doc["latitude"] = gps.location.lat();
    Serial.print("Longitude: ");
    doc["longitude"] = gps.location.lng();
    Serial.println(gps.location.lng(), 6);
    // You can also access other data like speed, altitude
    Serial.print("Speed MPH: ");
    doc["speed"] = gps.speed.mph();
    Serial.println(gps.speed.mph());
    Serial.print("Altitude Feet: ");
    doc["altitude"] = gps.altitude.feet();
    Serial.println(gps.altitude.feet());
    jsonData = getJsonString(doc);
    dataSetOFLastGPS = jsonData;
  }
  else
  {
    jsonData = dataSetOFLastGPS;
    Serial.println("Location not updated");
  }
  return jsonData;
}
//---------------------------------------------------------------------------------
// mpu data
String mpuData(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
  // accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Serial.println("Error MPU: " );

  Serial.print("a/g:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);
  DynamicJsonDocument doc(1024);
  doc["ax"] = ax;
  doc["ay"] = ay;
  doc["az"] = az;
  doc["gx"] = gx;
  doc["gy"] = gy;
  doc["gz"] = gz;
  String jsonData = getJsonString(doc);
  return jsonData;
}
// step counter
bool stepCounter()
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  accelgyro.getAcceleration(&ax, &ay, &az, &gx, &gy, &gz);
  float magnitude = sqrt(ax * ax + ay * ay + az * az);

  if (abs(magnitude - lastMagnitude) > stepThreshold)
  {
    unsigned long currentTime = millis();
    if (currentTime - lastStepTime > 250)
    { // Debounce: 250 milliseconds between steps
      stepCount++;
      Serial.print("Step Count: ");
      Serial.println(stepCount);
      lastStepTime = currentTime;
      lastMagnitude = magnitude;
      mpuDataSet = mpuData(ax, ay, az, gx, gy, gz);
      return true;
    }
  }
  lastMagnitude = magnitude;
  return false;
}
//---------------------------------------------------------------------------------
// calculate battery level
String batteryLevelFun()
{
  const float batteryMaxVoltage = 4.2; // maximum voltage of the LiPo battery
  const float batteryMinVoltage = 3.0; // minimum voltage of the LiPo battery
  const float voltageDividerRatio = 100000.0 / (100000.0 + 10000.0); // 100kΩ and 10kΩ voltage divider
  const int adcResolution = 4095; // 12-bit ADC resolution of ESP32
  const float adcReferenceVoltage = 3.3; // reference voltage of ESP32 ADC

  batteryLevel = analogRead(BATTERY_PIN);
  // int rawValue = analogRead(analogPin); // read the raw value from the ADC
  float voltage = (batteryLevel / (float)adcResolution) * adcReferenceVoltage / voltageDividerRatio; // calculate the battery voltage
  float soc = map(voltage, batteryMinVoltage, batteryMaxVoltage, 0, 100); // map the voltage to the SOC
  soc = constrain(soc, 0, 100); // constrain SOC to between 0% and 100%

  String str2 = String(soc, 2);
  return str2;
}
//---------------------------------------------------------------------------------
// log step count
String logstepcoutn(){
  DynamicJsonDocument doc(1024);
  doc["datetime"] = millis();
  doc["stepcount"] = stepCount;
  String jsonData = getJsonString(doc);
  return jsonData;
}
String storeUserData(){
  DynamicJsonDocument doc(1024);
  doc["userId"] = "";
  doc["name"] = "";
  doc["email"] = "";
  doc["name"] = "";
  doc["name"] = "";
  String jsonData = getJsonString(doc);
  return jsonData;
}

void connectFirebase(){
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
}

void updateFirebaseData(){
  
  FirebaseJson json;

  Serial.printf("Update json... %s\n\n", Firebase.RTDB.updateNode(&fbdo, "/test/push/", &json) ? "ok" : fbdo.errorReason().c_str());

}