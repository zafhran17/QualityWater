#include <SoftwareSerial.h>
#include <ArduinoJson.h>

//Initialise Arduino to NodeMCU (5=Rx & 6=Tx)
SoftwareSerial nodemcu(11, 10);

float phsen;
float turbid;
float tempe;
float tdsen;
float orps;
#define VOLTAGE 5.0

//PH sensor
float resolution;
int measurings;
float voltage;
float b = 4.25;
float m = 0.167;

//temperature sensor
#include <OneWire.h>
int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2
OneWire ds(DS18S20_Pin);  //Temperature chip i/o on digital pin 2

//TDS sensor
#define TdsSensorPin A1
//#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0, tdsValue = 0;
float temperature = 25;

//orp sensor
//#define VOLTAGE 5.0    //system voltage
#define OFFSET 0        //zero drift voltage
double orpValue;
#define ArrayLenth  40    //times of collection
#define orpPin A0          //orp meter output,connect to Arduino controller ADC pin
int orpArray[ArrayLenth];
int orpArrayIndex=0;

void setup() {
  Serial.begin(9600);
  resolution = 1024.0;
  pinMode(TdsSensorPin,INPUT);
  nodemcu.begin(9600);
  delay(1000);
  Serial.println("Program started");
}

void loop() {
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& data = jsonBuffer.createObject();

  ph_func();
  turbi_func();
  float tmp = getTemp();
  float tmpres = tmp;
  //float number only with two digit after point
  int intmp;
  intmp = (int (tmpres * 100));
  tempe = float (intmp)/100;
  //Serial.print('\n');
  //Serial.print("Temperature: ");
  Serial.println(tempe);
  tds_func();
  orp_func();
  
  data["ph"] = phsen;
  data["turbidity"] = turbid;
  data["temperature"] = tempe;
  data["tds"] = tdsen;
  data["orp"] = orps;

  data.printTo(nodemcu);
  jsonBuffer.clear();
  delay(5000);
  yield();
}

void ph_func() {
  measurings = 0;
  for (int i = 0; i < 10; i++) {
    measurings = measurings + analogRead(A3);
    delay(10);
  }
  voltage = (( 5 / resolution) * (measurings / 10));
  float PH_value = ((7 + ((2.5 - voltage) / m))) + b;
  
  float phres = PH_value;
  //float number only with two digit after point
  int intph;
  intph = (int (phres * 100));
  phsen = float (intph)/100;
  //Serial.print ("pH = ");
  Serial.println(phsen);
  //delay(5000);
}

void turbi_func() {
  int sensorValue = analogRead(A2);// read the input on analog pin 0:
  float turbi = sensorValue * (5.0 / 3866.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //Serial.print("Turbidity: ");
  
  float turbires = turbi;
  //float number only with two digit after point
  int inttb;
  inttb = (int (turbires * 100));
  turbid = float (inttb)/100;
  Serial.println(turbid); // print out the value you read:
}

float getTemp() {
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    Serial.println("no more sensors on chain, reset search!");
    ds.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}

void tds_func(){
  static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VOLTAGE / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(177.22*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      //Serial.print("TDS: ");
      float tdsres = tdsValue;
      int intds;
      intds = (int (tdsres *100));
      tdsen = float (intds)/100;
      Serial.println(tdsen,0);
      //Serial.println('\n');
      //Serial.println("ppm");
   }
}
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void orp_func(){
  static unsigned long orpTimer=millis();   //analog sampling interval
  static unsigned long printTime=millis();
  if(millis() >= orpTimer)
  {
    orpTimer=millis()+20;
    orpArray[orpArrayIndex++]=analogRead(orpPin);    //read an analog value every 20ms
    if (orpArrayIndex==ArrayLenth) {
      orpArrayIndex=0;
    }
    orpValue=((30*(double)VOLTAGE*1000)-(75*avergearray(orpArray, ArrayLenth)*VOLTAGE*1000/1024))/75-OFFSET;

    //convert the analog value to orp according the circuit
  }
  if(millis() >= printTime)   //Every 800 milliseconds, print a numerical
  {
    printTime=millis()+800;
    orps = ((int)orpValue);
    //Serial.print("ORP: ");
    Serial.println((int)orpValue);
  }
}

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    printf("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}
