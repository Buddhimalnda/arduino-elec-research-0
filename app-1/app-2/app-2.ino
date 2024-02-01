
#include <Arduino.h>
#include <ArduinoJson.h>
#if defined(ESP32) 
#include <WiFi.h>
#endif
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
FirebaseJson json;

FirebaseAuth auth;
FirebaseConfig config;

bool wifiConnected = false;

unsigned long sendDataPrevMillis = 0;

unsigned long count = 0;

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
}

void loop() {
  // put your main code here, to run repeatedly:
  int sdata = random(0, 1024);
  Serial.println(sdata);
  delay(1000);
  json.set("/data", sdata);
  delay(1000);

  // Firebase.RTDB.updateNode(firebaseData, "/sensor", json);
  Serial.printf("Update json... %s\n\n", Firebase.RTDB.updateNode(&fbdo, "/test/push/", &json) ? "ok" : fbdo.errorReason().c_str());
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