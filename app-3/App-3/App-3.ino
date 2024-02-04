// liberys
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoJson.h>
// GPS
#include <TinyGPS++.h>
// MPU
#include <Adafruit_MPU6050.h>
#include <I2Cdev.h>
#include <MPU6050.h>
// RTC
#include <ThreeWire.h>
#include <RtcDS1302.h>
// #include <DS1302.h>

// SD Card
#include <SD.h>
#include <SPI.h>
// Firebase
#include <FirebaseESP32.h>
// #include <Firebase_ESP_Client.h>
// Provide the token generation process info.
#include <addons/TokenHelper.h>
// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>
// Bluetooth
#include <BluetoothSerial.h>

//-------------------------
// Port Definitions
// input pins
// GPS
#define GPS_RX 16
#define GPS_TX 17
// MPU
#define MPU_SDA 21
#define MPU_SCL 22;
// Switch
#define SW2_PIN 35;
#define SW3_PIN 32;
#define SW4_PIN_SOS 32;
// Battery Voltage Level
#define BVL_PIN 34;
// RTC
#define RTC_CLK 5
#define RTC_RST 2
#define RTC_IO 4
//-------------------------
// output pins
// RGB LED
#define RGB_R 14
#define RGB_G 12
#define RGB_B 13
// SD Card
#define SD_CS 
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK 18
// Iindication LED
#define IL1_PIN 33
#define IL1_PIN 26
#define IL1_PIN 27
#define IL1_PIN 21
//-------------------------
// Global variables
// GPS
TinyGPSPlus gps;
// MPU
Adafruit_MPU6050 mpu;

unsigned long lastStepTime = 0;
int stepCount = 0;
float stepThreshold = 1.2; // Experimentally determined threshold for step detection
float debounceTime = 250; // Minimum time between steps (milliseconds)

// RTC
ThreeWire myWire(RTC_CLK, RTC_IO, RTC_RST);
RtcDS1302<ThreeWire> Rtc(myWire);
// Bluetooth
BluetoothSerial SerialBT;
// Firebase
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// Network and Firebase credentials
const char *ssid = "SLT-4G_163BEA";
const char *password = "751FCEED";

#define FIREBASE_HOST "https://elec-research-0-default-rtdb.asia-southeast1.firebasedatabase.app" // "<YOUR_FIREBASE_HOST>"
#define FIREBASE_AUTH "AIzaSyCvhDMEJ9gnl8lqyV282-8pdHOVmZqyVPs"                                   //"<YOUR_FIREBASE_AUTH>"

#define USER_EMAIL "device@gmail.com"
#define USER_PASSWORD "123456"

// SD Card
File userFile;

// DS1302
ThreeWire myWire(RTC_IO, RTC_CLK, RTC_RST) // IO, SCLK, CE
    RtcDS1302<ThreeWire> Rtc(myWire);
// now time
RtcDateTime now;
// variables
bool stateOfWIFI = false;
int mode = 0;
//-------------------------
// setup
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  initializeGPS();
  initializeMPU6050();
  initializeLEDs();
  if (!stateOfWIFI)
  {
    connectToWiFi();
  }
  initializeFirebase();
  initializeSDCard();
  initializeRTC();
  mode = 0;
}

// loop
void loop()
{
  // put your main code here, to run repeatedly:
  if(SW3_PIN){
    mode++;
    if(mode > 6){
      mode = 0;
    }
  }
  rgbLighting(mode);
  delay(1000);
  
}

// functions
void initializeGPS()
{
  Serial.println("Initializing GPS");
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

void initializeMPU6050()
{
  Serial.println("Initializing MPU6050");
  Wire.begin(MPU_SDA, MPU_SCL);
  mpu.initialize();
  mpu.testConnection() ? Serial.println("MPU6050 connection successful") : Serial.println("MPU6050 connection failed");
}

void initializeLEDs()
{
  Serial.println("Initializing LEDs");
  pinMode(RGB_R, OUTPUT);
  pinMode(RGB_G, OUTPUT);
  pinMode(RGB_B, OUTPUT);
  pinMode(IL1_PIN, OUTPUT);
  pinMode(IL2_PIN, OUTPUT);
  pinMode(IL3_PIN, OUTPUT);
  pinMode(IL4_PIN, OUTPUT);
}

void connectToWiFi()
{
  Serial.println("Connecting to WiFi");
  WiFi.begin("SSID", "PASSWORD");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  stateOfWIFI = true;
}
// connect to Firebase
// initiate firebase
void initializeFirebase()
{
  Serial.println("Initializing Firebase");
  // Firebase.begin("FIREBASE_HOST", "FIREBASE_AUTH");
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

void initializeSDCard()
{
  Serial.println("Initializing SD Card");
  if (!SD.begin(SD_CS))
  {
    Serial.println("SD Card initialization failed");
    return;
  }
  Serial.println("SD Card initialization successful");
}

void initializeRTC()
{
  Serial.println("Initializing RTC");
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  if (!Rtc.IsDateTimeValid())
  {
    if (Rtc.LastError() != 0)
    {
      Serial.print("RTC communications error = ");
      Serial.println(Rtc.LastError());
    }
    else
    {
      Serial.println("RTC lost confidence in the DateTime!");
      Rtc.SetDateTime(compiled);
    }
  }
  if (!Rtc.GetIsRunning())
  {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }
  now = Rtc.GetDateTime();
  if (now < compiled)
  {
    Serial.println("RTC is older than compile time! Updating");
    Rtc.SetDateTime(compiled);
  }
  else if (now > compiled)
  {
    Serial.println("RTC is newer than compile time. Not updating");
  }
  else if (now == compiled)
  {
    Serial.println("RTC is the same as compile time! Not updating");
  }
}

String serializationJson(JsonDocument doc)
{
  String json;
  doc['test'] = "";
  JsonArray data = doc["data"].to<JsonArray>();
  data.add(48.756080);
  data.add(2.302038);
  serializeJson(doc, json);
  return json;
}

JsonDocument deserializationJson(char json[])
{
  JsonDocument doc;
  deserializeJson(doc, json);
  return doc;
}

// data manage to SD.
JsonDocument userData()
{
  JsonDocument doc;
  userFile = SD.open("user.json", FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  String jsonString = "";
  while (file.available())
  {
    jsonString += (char)file.read();
  }
  file.close();
  char jsonCharArray[jsonString.length() + 1];
  jsonString.toCharArray(jsonCharArray, sizeof(jsonCharArray));
  DynamicJsonDocument doc(1024); // Adjust size according to your JSON document
  deserializeJson(doc, jsonCharArray);
  /*
      {
          deviceID: "",
          email: "",
          password: "",
          isToken: false,
          token: ""
          user: {
              name: "",

          }
      }
  */
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

void updateFirebaseData(FirebaseJson json, String path)
{
  Serial.printf("Update json... %s\n\n", Firebase.RTDB.updateNode(&firebaseData, path, &json) ? "ok" : firebaseData.errorReason().c_str());
}

void readFirebaseData()
{
  Serial.printf("Read json... %s\n\n", Firebase.RTDB.getJSON(&fbdo, "/test/push") ? "ok" : fbdo.errorReason().c_str());
  if (fbdo.dataType() == "json")
  {
    FirebaseJson &json = fbdo.jsonObject();
    Serial.println(json);
  }
}

void stepCounter(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Calculate the magnitude of acceleration */
  float accMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z);

  /* Normalize to 1g */
  accMagnitude /= GRAVITY_EARTH;

  /* Detect step */
  unsigned long currentTime = millis();
  if (accMagnitude > stepThreshold && currentTime - lastStepTime > debounceTime) {
    stepCount++;
    lastStepTime = currentTime;
    Serial.print("Step Count: ");
    Serial.println(stepCount);
    //update firebase
    FirebaseJson json;
    json.add("step", stepCount);
    updateFirebaseData(json, "/");
  }

  delay(10);
}

