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

FirebaseData fbdo;
FirebaseJson json;
FirebaseAuth auth;
FirebaseConfig config;

void setup() {
  // Initialize Serial port
  Serial.begin(9600);

    WiFi.begin(ssid, pass);    //try to connect with wifi
  //Serial.print("Connecting to ");
  Serial.print(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print(".");
    yield();
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
  
  nodemcu.begin(9600);
  while (!Serial) continue;
  yield();
}

void datajson(){
  //receive data from arduino through json
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& data = jsonBuffer.parseObject(nodemcu);

  float phsen = data["ph"];
  float turbid = data["turbidity"];
  float tempe = data["temperature"];
  float tdsen = data["tds"];
  //float orps = data["orp"];
  
  Serial.println(phsen);
  json.set("ph", phsen);
  Serial.println(turbid);
  json.set("turbidity", turbid);
  Serial.println(tempe);
  json.set("temperature", tempe);
  Serial.println(tdsen);
  json.set("tds", tdsen);
  //Serial.println(orps);
  //json.set("orp", orps);
  
  }

void loop() {

 // if (data == JsonObject::invalid()) {
    //Serial.println("Invalid Json Object");
  //  jsonBuffer.clear();
    //return;
   //  yield();
 // }
  //Serial.println("JSON Object Recieved");
  String Datastring;fi
  datajson();

  Serial.println(json.toString(Serial,true));
  Firebase.updateNodeAsync(fbdo, "MonitoringDebitQualityApp/MonitoringQuality/Kualitas1/", json);
  //Serial.println(fbdo.errorReason());
  delay(10000);
  yield();
}
