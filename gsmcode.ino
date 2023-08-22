//Code for Arduino

#include <TinyGPSPlus.h>
#include <NeoSWSerial.h>
#include <ArduinoJson.h>
#include <AS3935.h>
#include <AsyncDelay.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>


//#include <dht.h>
// my two serial connectins will be through my GPRS and GSM 


//dht DHT;
//rread temp from pin 
int sim900RxPin=0;
int sim900TxPin=1;

int GPSBaud = 9600;
int GSMBaud =9600;

int sensorValue;

int tempValue; 

boolean countStatus;
int beat, bpm;
unsigned long millisBefore;

String host = "https://engineering-final-project.onrender.com/api/send";
String APN = "WEB";


//helper variables for waitUntilResponse() function
String response = "";
static long maxResponseTime = 5000;
unsigned long lastTime;
boolean reboot = false;

boolean gprsConnectionSetup = false;
int maxNumberOfErrors = 10;
int errorsEncountered = 0;

//The frequency of http requests (seconds)
int refreshRate = 15; //SECONDS
//variables for a well-scheduled loop - in which the sendData() gets called every 15 secs (refresh rate)
unsigned long last;
unsigned long current;
unsigned long elapsed;
// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial" and "gsmSerial"
//NeoSWSerial gpsSerial(RXPin, TXPin);
NeoSWSerial gprsSerial(sim900RxPin,sim900TxPin);


//p=3 g=4 g=0 1=y


void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  gprsSerial.begin(GSMBaud);

  gprsSerial.write(27); //Clears buffer for safety
  Serial.println("Beginning...");
  delay(15000); //Waiting for Sim800L to get signal
  Serial.println("SIM900 has booted up");

  //gprsSerial.listen(); The GSM module and GPS module can't communicate with the arduino board at once - so they need to get focus once we need them
  gprsSerial.println("AT");
  delay(1000);
  setupGPRSConnection(); //Enable the internet connection to the SIM card
  Serial.println("Connection set up");
 
 // gpsSerial.listen();

 
}

void loop()
{

  int sensorValue = analogRead(A0);
  // print out the value you read:
  //Serial.println(sensorValue);
  if (countStatus == 0)
  {
    if (sensorValue > 600)
    {
      countStatus = 1;
      beat++;
    }
  }
  else
  {
    if (sensorValue < 500)
    {
      countStatus = 0;
    }
  }
  if (millis() - millisBefore > 15000)
  {
    bpm = beat * 4;
    beat = 0;
    millisBefore = millis();
  }
  delay(100);        // delay in betwee

  /*int*/ sensorValue = analogRead(A1);  // Read from analog pin A1
  delay(1000);  // Wait for 1 second
  
 //gpsSerial.listen();
 while (Serial.available() > 0){
//    Serial.println("Hello");
    if (gps.encode(Serial.read())){
//      Serial.println("Hi");
      displayInfo();
        }
      }

}

char sz[32];

void displayInfo()
{
 
  Serial.println("POST: ");
  if (gps.date.isValid() && gps.time.isValid() && gps.location.isValid())
  {
   
    sprintf(sz, "%02d-%02d-%02dT%02d:%02d:%02d.%02dZ", gps.date.year(), gps.date.month(), gps.date.day(),gps.time.hour(),gps.time.minute(),gps.time.second(),gps.time.centisecond());
    
    StaticJsonDocument<64> doc;

    doc["temperature"] = sensorValue;//temp variable
    doc["heartrate"] = bpm;
    //doc["humidity"] = humid;
    doc["lat"] =  String(gps.location.lat(),6);
    doc["lng"] = String(gps.location.lng(),6);
    doc["dateTime"] = sz;    


    String output;
    serializeJson(doc, output);
   
    gprsSerial.println("AT+HTTPPARA=\"URL\", \"" + host +  "/data\"");
    waitUntilResponse("OK");
    delay(1000);
    //readResponse();

    gprsSerial.println(F("AT+HTTPPARA=\"CONTENT\",\"application/json\""));
    waitUntilResponse("OK");
    delay(1000);
    //readResponse();

    gprsSerial.println("AT+HTTPDATA="+String(output.length())+",100000");
    waitUntilResponse("DOWNLOAD");
    delay(1000);
    //readResponse();

    Serial.println(output);
    gprsSerial.println(output);
    delay(1000);
    //readResponse();

    gprsSerial.println("AT+HTTPACTION=1");
    waitUntilResponse("200");
    delay(1000);
    readResponse();
   
    //gprsSerial.println(F("AT+HTTPREAD"));
    //waitUntilResponse("OK");
    //delay(1000);
  //  readResponseRead();
   
    //gprsSerial.println(F("AT+HTTPTERM"));
    //waitUntilResponse("OK");
    //delay(1000);
   // readResponse();  

   
  }
  else
  {
    Serial.println("Not Available");
  }

 
  Serial.println();
  Serial.println();
  delay(1000);

 
}


void setupGPRSConnection(){
 
 gprsSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r"); //Connection type: GPRS
 waitUntilResponse("OK");
 
 gprsSerial.println("AT+SAPBR=3,1,\"APN\",\"" + APN + "\"\r"); //We need to set the APN which our internet provider gives us
 waitUntilResponse("OK");
 
 gprsSerial.println("AT+SAPBR=1,1"); //Enable the GPRS
 waitUntilResponse("OK");
 
 gprsSerial.println("AT+SAPBR=2,1");
 waitUntilResponse("OK");
 
 gprsSerial.println("AT+HTTPINIT"); //Enabling HTTP mode
 waitUntilResponse("OK");
 
 gprsSerial.println("AT+HTTPPARA=\"CID\",1");
 waitUntilResponse("OK");
 
 //gprsSerial.println("AT+HTTPSSL=1");
 
 gprsConnectionSetup = true;
}



void waitUntilResponse(String resp){
  lastTime = millis();
  response="";
  String totalResponse = "";
  while(response.indexOf(resp) < 0 && millis() - lastTime < maxResponseTime)
  {
    readResponse();
    totalResponse = totalResponse + response;
    Serial.println(response);
  }
 
  if(totalResponse.length() <= 0)
  {
    Serial.println("NO RESPONSE");
   // digitalWrite(sim800Pin, HIGH);
    if (gprsConnectionSetup == true){
      Serial.println("error");
      errorsEncountered++;
    }
  }
  else if (response.indexOf(resp) < 0)
  {
    if (gprsConnectionSetup == true){
      Serial.println("error");
      errorsEncountered++;
    }
    Serial.println("UNEXPECTED RESPONSE");
    Serial.println(totalResponse);
    //digitalWrite(sim800Pin, HIGH);
  }else{
    Serial.println("SUCCESSFUL");
   // digitalWrite(sim800Pin, LOW);
    errorsEncountered = 0;
  }

  //if there are more errors or equal than previously set ==> reboot!
  if (errorsEncountered>= maxNumberOfErrors){
    reboot = true;
  }
}


void readResponse(){
  response = "";
  while(response.length() <= 0 || !response.endsWith("\n"))
  {
    tryToRead();
    if(millis() - lastTime > maxResponseTime)
    {
      return;
    }
  }
}

void tryToRead(){
  while(gprsSerial.available()){
    char c = gprsSerial.read();  //gets one byte from serial buffer
    //Serial.write(c);
    response += c; //makes the string readString
  }
}
