#include <SoftwareSerial.h>
#include <ArduinoJson.h>
//D6 = Rx & D5 = Tx
SoftwareSerial nodemcu(D6, D5);

#include <common.h>
#include <Utils.h>
#include <ESP8266WiFi.h>
#include <Firebase.h>
#include <FirebaseFS.h>
#include <FirebaseESP8266.h>
String ssid = "adikost 2";
String pass = "evanadi11";
  
// Declare the Firebase Data object in the global scope
FirebaseData fbdo;
FirebaseJson json;
FirebaseAuth auth;
FirebaseConfig config;

void setup() {
  Serial.begin(9600);                                   
  //Serial.println("Serial communication started\n\n");  
  WiFi.begin(ssid, pass);    //try to connect with wifi
  //Serial.print("Connecting to ");
  Serial.print(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print(".");
    delay(500);
  }
  Serial.println();
  //Serial.print("Connected to ");
  Serial.println(ssid);
 // Serial.print("IP Address is : ");
  Serial.println(WiFi.localIP());                                             
   config.database_url = "https://pdam-iot-65ebf-default-rtdb.firebaseio.com/";
   config.signer.tokens.legacy_token = "oIH91rpFqN47QJ4zjapYQlFLwZJOwgPkjAbGwQw8";
   Firebase.reconnectWiFi(true);
   Firebase.begin(&config, &auth);
  delay(1000);
  nodemcu.begin(9600);
  while (!Serial) continue;
}

void jsondata(){
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& data = jsonBuffer.parseObject(nodemcu);

  if (data == JsonObject::invalid()) {
    //Serial.println("Invalid Json Object");
    jsonBuffer.clear();
    return;
  }
float turbi = data["turbidity"];
float orps = data["orp"];
float tmp = data["temperature"];
float tds = data["tdsen"];
float phsen = data["phsensor"];

json.set("turbidity", turbi);
Serial.println(turbi);
json.set ("orp", orps);
Serial.println(orps);
json.set ("temperature", tmp);
Serial.println(tmp);
json.set ("tds", tds);
Serial.println(tds);
json.set ("phsensor", phsen);
Serial.println(phsen);

}

void loop() {
jsondata();
Serial.println(json.toString(Serial,true));
Firebase.updateNodeAsync(fbdo, "MonitoringDebitQualityApp/MonitoringQuality/Kualitas1/", json);

//Firebase.setFloat(firebaseData, PHREF, ph);
//Serial.println(phsen);
Serial.println();

delay(20000);

}
