#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>


#define FIREBASE_HOST "https://pdam-iot-65ebf-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "oIH91rpFqN47QJ4zjapYQlFLwZJOwgPkjAbGwQw8"
#define WIFI_SSID "adikost 2"
#define WIFI_PASSWORD "evanadi11"


//D6 = Rx & D5 = Tx
SoftwareSerial nodemcu(D6, D5);
FirebaseData firebaseData;
void setup() {
  // Initialize Serial port
  Serial.begin(9600);
  nodemcu.begin(9600);
  while (!Serial) continue;
  
Serial.println("Serial communication started\n\n");  
           
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);                                     //try to connect with wifi
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP Address is : ");
  Serial.println(WiFi.localIP());                                            
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);   

  Firebase.reconnectWiFi(true);
  delay(1000);
  yield();
  
}

void loop() {
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& data = jsonBuffer.parseObject(nodemcu);

  if (data == JsonObject::invalid()) {
    //Serial.println("Invalid Json Object");
    jsonBuffer.clear();
    return;
     yield();
  }
  Serial.println("JSON Object Recieved");
  
  Serial.print("Recieved turbidity:  ");
  float turbid = data["turbidity"];
  Serial.println(turbid,2);

  Serial.print("Recieved pH:  ");
  float phsen = data["ph"];
  Serial.println(phsen, 2);

  Serial.print("Recieved ORP:  ");
  float orps = data["orp"];
  Serial.println(orps, 2);

  Serial.print("Recieved Temperature:  ");
  float tempe = data["temperature"];
  Serial.println(tempe, 2);

  Serial.print("Recieved TDS:  ");
  float tdsen = data["tds"];
  Serial.println(tdsen, 2);
  Serial.println("-----------------------------------------");
  //yield();
  Firebase.setFloat(firebaseData, "MonitoringDebitQualityApp/MonitoringQuality/Kualitas1/turbidity", turbid);
  Firebase.setFloat(firebaseData, "MonitoringDebitQualityApp/MonitoringQuality/Kualitas1/ph", phsen);
  Firebase.setFloat(firebaseData, "MonitoringDebitQualityApp/MonitoringQuality/Kualitas1/orp", orps);
  Firebase.setFloat(firebaseData, "MonitoringDebitQualityApp/MonitoringQuality/Kualitas1/temperature", tempe);
  Firebase.setFloat(firebaseData, "MonitoringDebitQualityApp/MonitoringQuality/Kualitas1/tds", tdsen);
}
