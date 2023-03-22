#include <Arduino.h>
#include <Wire.h>
#include <WebServer.h>
#include <WiFi.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255


uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

#define REPORTING_PERIOD_MS 1000

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 2; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
 
//Wifi vars

// HTML & CSS contents which display on web server
String HTML, s_spo2, s_hr;
// SSID & Password
const char* ssid = "MiFibra-F392"; // Enter your SSID here
const char* password = "5QUisHGE"; //Enter your Password here

WebServer server(80);// Object of WebServer(HTTP port, 80 is default)

void handle_root();

void setup()
{
  ledcSetup(0, 0, 8); // PWM Channel = 0, Initial PWM Frequency = 0Hz, Resolution = 8 bits
  ledcAttachPin(pulseLED, 0); //attach pulseLED pin to PWM Channel 0
  ledcWrite(0, 255); //set PWM Channel Duty Cycle to 255

  Serial.begin(115200); // initialize serial communication at 115200 bits per second:

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  //Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  //while (Serial.available() == 0) ; //wait until user presses a key
  //Serial.read();
  
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  //WIFI CONNECTION
  Serial.print("Try Connecting to ");
  Serial.println(ssid);
  // Connect to your wi-fi modem
  WiFi.begin(ssid, password);
  // Check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) 
  {
      delay(1000);
      Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP()); //Show ESP32 IP on serial
  server.on("/", handle_root);
  server.begin();
  Serial.println("HTTP server started");
  //END
}

void loop()
{
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
    Serial.print(F("\t ir="));
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
      Serial.print(F("\t ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F("\t HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F("\t HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F("\t SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F("\t SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    
    //Web view
    /*if(validHeartRate)*/String s_hr = String(heartRate, DEC);
    /*if(validSPO2)*/String s_spo2 = String(spo2, DEC);
    
    HTML =  "<!DOCTYPE html>"
            "<html lang ='es'>"
            "    <head>"
            "        <meta charset='utf-8'>"
            "        <title>Heart Rate & SPO2</title>"
            "        <link rel='preconnect' href='https://fonts.googleapis.com'>"
            "        <link rel='preconnect' href='https://fonts.gstatic.com' crossorigin>"
            "        <link href='https://fonts.googleapis.com/css2?family=Poppins:wght@200&display=swap' rel='stylesheet'>"
            "    </head>"
            "    <style>"
            "        body{"
            "            background-color: #252525;"
            "            color: #bfbbbb;"
            "            font: 200 20px 'Poppins',sans-serif ;"
            "        }"
            "        h1{"
            "            color: #ffffff;"
            "            font-size: 36px;"
            "            text-align: center;"
            "        }"
            "        p{"
            "            text-align: center;"
            "        }"
            "        div{"
            "            background-color: #6a676734;"
            "            border-radius: 15px;"
            "        }"
            "    </style>"
            "            "
            "    <body>"
            "        <div>"
            "            <h1>Heart Rate and Oxygen Saturation</h1>"
            "            <p>Heart Rate : "+s_hr+" BPM &#9825;</p>"
            "            <p>Oxygen Saturation : "+s_spo2+" % </p>"
            "        </div>"
            "    </body>"
            "</html>";
    
    server.handleClient();
  }
}

// Handle root url (/)
void handle_root() 
{
    server.send(200, "text/html", HTML);
}