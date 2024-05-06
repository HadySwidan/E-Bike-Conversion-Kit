#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include <TinyGPSPlus.h>
#include <Firebase_ESP_Client.h>
MAX30105 particleSensor;
#define WIFI_SSID "WE"
#define WIFI_PASSWORD "02468024680"
#define MAX_BRIGHTNESS 255
//======================================== 

//Provide the token generation process info.
#include "addons/TokenHelper.h"

//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Defines the Digital Pin of the "On Board LED".

// Insert Firebase project API Key
#define API_KEY "YOUR_API_KEY"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "YOUR_DATABASE_URL" 
void maxim_heart_rate_and_oxygen_saturation(uint32_t* irBuffer, int32_t bufferLength, uint32_t* redBuffer, int32_t* spo2, int8_t* validSPO2, int32_t* heartRate, int8_t* validHeartRate);

// Define Firebase Data object.
FirebaseData fbdo;

// Define firebase authentication.
FirebaseAuth auth;

// Definee firebase configuration.
FirebaseConfig config;

//======================================== Millis variable to send/store data to firebase database.
unsigned long sendDataPrevMillis = 0;
const long sendDataIntervalMillis = 1000; //--> Sends/stores data to firebase database every 5 seconds.
//======================================== 

// Boolean variable for sign in status.
bool signupOK = false;


//________________________________________________________________________________ VOID SETUP
int val;
int tempPin = 33;
int cel;
bool Status = false;

#define Threshold 200
#define MQ2pin 32
float sensorValue;  //variable to store sensor value

             // we start, assuming no motion detected

// The TinyGPSPlus object

TinyGPSPlus gps;
#if defined(AVR_ATmega328P) || defined(AVR_ATmega168)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13;


/* Flame Sensor analog example.
To test view the output, point a serial monitor such as Putty at your arduino. 
*/

// lowest and highest sensor readings:
const int sensorMin = 0;     // sensor minimum
const int sensorMax = 1024;  // sensor maximum

//constants for the pins where sensors are plugged into.
const int sensorPin = 34; //LDR sensor pin
const int ledPin = 25; // Red LED 

//Set up some global variables for the light level an initial value.
int lightInit;  // initial value
int lightVal;   // light reading
String firestatus="";
void displayInfo();
void setup() {

  // initialize serial communication @ 9600 baud:
  Serial.begin(9600);  
  Serial.begin(115200); 
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);



  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("---------------Connection");
  Serial.print("Connecting to : ");
  Serial.println(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED){
    Serial.print("."); delay(100);

  }
  Serial.println();
  Serial.print("Successfully connected to : ");
  Serial.println(WIFI_SSID);
  //Serial.print("IP : ");
  //Serial.println(WiFi.localIP());
  Serial.println("---------------");
  //---------------------------------------- 

  // Assign the api key (required).
  config.api_key = API_KEY;

  // Assign the RTDB URL (required).
  config.database_url = DATABASE_URL;

  // Sign up.
  Serial.println();
  Serial.println("---------------Sign up");
  Serial.print("Sign up new user... ");
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  Serial.println("---------------");
  
  // Assign the callback function for the long running token generation task.
  config.token_status_callback = tokenStatusCallback; //--> see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  

Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
   Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096;
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();
  //The LEDs are very low power and won't affect the temp reading much but
  //you may want to turn off the LEDs to avoid any local heating
  particleSensor.setup(0);
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings //Configure sensor. Turn off LEDs

  particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.

  // We'll set up the LED pin to be an output.
  pinMode(ledPin, OUTPUT);
  lightInit = analogRead(sensorPin);
  //we will take a single reading from the light sensor and store it in the lightCal       
  //variable. This will give us a prelinary value to compare against in the loop

Serial.println("MQ2 warming up!");// allow the MQ2 to warm up
  delay(100); 
  
  
}
  

void loop() {
    if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > sendDataIntervalMillis || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    lightVal = analogRead(sensorPin);
    delay(100);
      Serial.println("lightVal =");
      Serial.println(lightVal);
      if (Firebase.RTDB.getBool(&fbdo, "Flash/Status")) {
      if(fbdo.dataType() == "boolean"){
        Status = fbdo.boolData();
        if(Status==true){
          digitalWrite(ledPin, HIGH);
          Serial.println("Light turned on");
        }else{
          // If lightVal is less than the initial reading within a threshold, it is dark.
          if (lightVal  - lightInit < 500)

  {  
    digitalWrite(ledPin, LOW); // Turn off the light
            Serial.println("Light turned off");
 // turn on light
    } else if (lightVal > 1800) {
            digitalWrite(ledPin, HIGH); // Turn on the light
            Serial.println("Light turned on");
          }
  }

      }

        }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }


  Serial.println();


    delay(100);
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC); // PRINT THE HEARTRATE VALUE
if (Firebase.RTDB.setInt(&fbdo, "Body/Heart Beat",heartRate)) {
    Serial.println( ); Serial.println("heartBeat =");Serial.println(heartRate); ;
    Serial. print(" - successfully saved to :" +fbdo.dataPath());
        Serial.println( " (" + fbdo.dataType() + ")");
    
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC); // PRINT THE PERCENTAGE OF OXYGEN IN BLOOD VALUE
        if (Firebase.RTDB.setInt(&fbdo, "Body/Blood Ox",spo2)) {
    Serial.println( ); Serial.println("SpO2 =");Serial.println(spo2); ;
    Serial. print(" - successfully saved to :" +fbdo.dataPath());
        Serial.println( " (" + fbdo.dataType() + ")");
    
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
  delay(200);
    
    Serial.println();

    // Temperature body using MAX3010
  int temperature = particleSensor.readTemperature();

  Serial.print("temperatureC=");
  Serial.print(temperature, 4);
  Serial.println();

  float temperatureF = particleSensor.readTemperatureF();

  Serial.print(" temperatureF=");
  Serial.print(temperatureF, 4);
if (Firebase.RTDB.setInt(&fbdo, "Body/Temperature",temperature)) {
    Serial.println( ); Serial.print (temperature) ;
    Serial. print(" - successfully saved to :" +fbdo.dataPath());
        Serial.println( " (" + fbdo.dataType() + ")");
    
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

  Serial.println();

  sensorValue = analogRead(MQ2pin); // read analog input pin 0
  
  Serial.print("Sensor Value Smoke: ");
  Serial.print(sensorValue);

  if(sensorValue > Threshold)
  {
    Serial.print(" | Smoke detected!");
  }
  
  Serial.println("");
  if (Firebase.RTDB.setFloat(&fbdo, "Smoke/Value",sensorValue)) {
    Serial.println( ); Serial.print (sensorValue) ;
    Serial. print(" - successfully saved to :" +fbdo.dataPath());
        Serial.println( " (" + fbdo.dataType() + ")");
    
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  delay(100); // wait 2s for next reading
  
    Serial.println();
    Serial.println("---------------Store Battery temp");
    val = analogRead(tempPin);
    float mv = ( val/1024.0)*5000;
    int cel = mv/50;
    float farh = (cel*9)/5 + 32;

    Serial.print("TEMPRATURE = ");
    Serial.print(cel);
    Serial.print("*C");
    Serial.println();
    delay(100);
    //temperature using lm35
  if (Firebase.RTDB.setInt(&fbdo, "Battery/Temperature",cel)) {
    Serial.println( ); Serial.print (cel) ;
    Serial. print(" - successfully saved to :" +fbdo.dataPath());
        Serial.println( " (" + fbdo.dataType() + ")");
    
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      displayInfo();
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    // Handle the GPS error here, you can choose to continue the loop even if GPS is not detected
  }
  delay(200);
   if (gps.location.isValid()) {
     double latitude = gps.location.lat();
      double longitude = gps.location.lng();
        if (Firebase.RTDB.setDouble(&fbdo, "Location/Latitude",latitude)) {
    Serial.println( ); Serial.print (latitude) ;
    Serial. print(" - successfully saved to :" +fbdo.dataPath());
        Serial.println( " (" + fbdo.dataType() + ")");
    
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    Serial.println();
    if (Firebase.RTDB.setDouble(&fbdo, "Location/longitude",longitude)) {
    Serial.println( ); Serial.print (longitude) ;
    Serial. print(" - successfully saved to :" +fbdo.dataPath());
        Serial.println( " (" + fbdo.dataType() + ")");
    
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    
  }
  Serial.println();
  
    }
}
void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print("Lng: ");
    Serial.print(gps.location.lng(), 6);
    Serial.println();
  } else {
    Serial.print(F("INVALID"));
  }
}

void updateSerial() {
  delay(1000);
  while (Serial.available()) {
    Serial2.write(Serial.read()); //Forward what Serial received to Software Serial Port
  }
  while (Serial2.available()) {
    Serial.write(Serial2.read()); //Forward what Software Serial received to Serial Port
  }
  
}


