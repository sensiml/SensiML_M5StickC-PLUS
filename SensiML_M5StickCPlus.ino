/* ------------------------------------------------------------------------------------------------------------------------------------------------------
*   SensiML M5StickC PLUS Data Capture Application - Arduino M5StickC PLUS (ESP32-PICO)
*   Version: 1.0
*   Date: December 13, 2023
*   Author: Chris Rogers
*   Copyright (c) 2023 SensiML Corporation
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*
*   3. Neither the name of the copyright holder nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
*   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------------------------------------------------------------------------------ */

#include <M5StickCPlus.h> //Note: M5StickCPlus2 not yet validated 
#include <WiFi.h> // Tested on version 1.2.7

// Select to build recognition firmware or data collection firmware
// RECOGNITIUON 0 - Build data collection firmware
// RECOGNITION 1 - Build firmware for recognition (requires importing a sensiml library)

#define RECOGNITION 0

//Wi-Fi setup parameters
const int PACKET_RATE = 30;   // WiFi streaming packet rate in packets/sec (impacts DCL streaming choppiness)
char wifiSSID[33] = "";     // Configure to SSID for desired WiFi AP to connect
char wifiPassword[64] = ""; // Configure to password for desired WiFi AP to connect


#if RECOGNITION
#include <kb.h>
#include <kb_output.h>
#define SERIAL_OUT_CHARS_MAX 256
static char serial_out_buf[SERIAL_OUT_CHARS_MAX];
#endif


//----------------------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------------------------
//Configure these values to setup data capture options and WiFi connection parameters:
#define CAPTURE_TRIGGER 1
#define CHANNELS_PER_SAMPLE  (6 + CAPTURE_TRIGGER)

//Set desired IMU sensor sample rate
#define SAMPLE_RATE 250  // valid sample rates are ODR_10Hz, ODR_50Hz, ODR_100Hz, ODR_125Hz, ODR_200Hz, ODR_250Hz, ODR_500Hz, or ODR_1kHz
int sampleRate = MPU6886::Fodr::ODR_250Hz;
#define BUTTON_A G37
#define BUTTON_B G39
#define NUM_CLASSES 7  // Number of distinct trigger levels to cycle through with Button B

//If trigger channel is used (#define CAPTURE_TRIGGER true), trigger channel amplitude will be zero unless Button A on M5Stick is pressed
//When Button A is pressed, trigger channel value = (classIndex * 1024), where ClassIndex increments from 0 to NUM_CLASSES for each press of M5Stick Button B
//Intended use is to assist in annotating segment classes during data collection in DCL
char classIndex = 0;  // State variable for starting trigger channel amplitude/1024

// Sensor Data Array and SSFv1 HTTP streaming format declarations

int SAMPLES_PER_PACKET = (PACKET_RATE * (SAMPLE_RATE + 1)) / 1000;  //Adjust samples/packet to yield 10 packets/sec stream no matter the sample rate
int PACKET_SIZE = SAMPLES_PER_PACKET * CHANNELS_PER_SAMPLE; // accelx,y,z + gyrox,y,z + target gesture class number from BtnA trigger

#if RECOGNITION
static int16_t pData[CHANNELS_PER_SAMPLE];
#else
char dataPacket[700 * 2];  // Define as maximum array size for 1kHz sample rate
#endif
uint32_t packetIndex = 0;
String ssfConfig =""; // DCL simple streaming format descriptor string (see https://sensiml.com/documentation/simple-streaming-specification/simple-wifi-streaming.html#simple-stream-wi-fi-endpoints)

// IMU sensor channel variables
int16_t accX = 0, accY = 0, accZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0;


// HTTP timer declarations
const long TIMEOUT_TIME = 2000; // 2sec TCP/HTTP session timeout
unsigned long httpTime;

// HTTP state variables
String header = "";
int crcount = 0;
bool streaming = false;

// Button variables
uint8_t btnState[2] = {HIGH, HIGH};
uint32_t btnTime[2] = {0, 0};
                                  
WiFiServer server(80);  // Set web server port number to 80
WiFiClient client;   // Instantiate WiFi client

// Application state variables
bool resetReady = false;
int16_t sampleIndex = 0;
bool isStreaming = false; // Boolean state variable for DCL_MODE streaming state

int ButtonPressTime(uint8_t button) {  // returns 0 if button is currently depressed, -1 if not depressed, and number of milliseconds held down if just released

  if (digitalRead(button)==LOW) {  //Button is being pressed, set state variable and counter if this just now happened
    if (btnState[button==BUTTON_B]==HIGH) {
      btnState[button==BUTTON_B] = LOW;
      btnTime[button==BUTTON_B] = millis();
    }
    return 0;
  }
  
  if ((digitalRead(button)==HIGH) && (btnState[button==BUTTON_B]==LOW)) { // Button is just now released, return total time pressed
    btnState[button==BUTTON_B] = HIGH;
    int32_t timeval = millis() - btnTime[button==BUTTON_B];
    return timeval;
  }
  
  return -1;  //  Button not depressed
}

#if RECOGNITION
int GetSensorDataSampleRecognition() {  //get one sample frame from IMU FIFO and return number of bytes read

  int16_t temp;
  
  if (M5.IMU.getFIFOData(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ,&temp) == 0){ //MPU6886 FIFO read returns accelXYZ, gyroXYZ and chip temp, we discard temp
    pData[0]= accX;
    pData[1]= accY;
    pData[2]= accZ;
    pData[3]= gyroX;
    pData[4]= gyroY;
    pData[5]= gyroZ;
    if (CAPTURE_TRIGGER)
    {
    char btnAState = digitalRead(BUTTON_A);
      pData[6]= (int16_t)digitalRead(BUTTON_A);
      return 14;
    }
    return 12;
  }
   
  return 0;
}
#else 
int GetSensorDataSampleStream(int samplenum) {  //get one sample frame from IMU FIFO and return number of bytes read
  

  int16_t temp;
  
  if (M5.IMU.getFIFOData(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ,&temp) == 0){ //MPU6886 FIFO read returns accelXYZ, gyroXYZ and chip temp, we discard temp

    uint16_t offset = samplenum * CHANNELS_PER_SAMPLE * 2;
    dataPacket[offset + 0] = (char)(accX & 0xff);
    dataPacket[offset + 1] = (char)(accX >> 8 & 0xff);
    dataPacket[offset + 2] = (char)(accY & 0xff);
    dataPacket[offset + 3] = (char)(accY >> 8 & 0xff);
    dataPacket[offset + 4] = (char)(accZ & 0xff);
    dataPacket[offset + 5] = (char)(accZ >> 8 & 0xff);
    dataPacket[offset + 6] = (char)(gyroX & 0xff);
    dataPacket[offset + 7] = (char)(gyroX >> 8 & 0xff);
    dataPacket[offset + 8] = (char)(gyroY & 0xff);
    dataPacket[offset + 9] = (char)(gyroY >> 8 & 0xff);
    dataPacket[offset + 10] = (char)(gyroZ & 0xff);
    dataPacket[offset + 11] = (char)(gyroZ >> 8 & 0xff);
    if (CAPTURE_TRIGGER)
    {
    char btnAState = digitalRead(BUTTON_A);
      dataPacket[offset + 12] = (char)0;
      dataPacket[offset + 13] = (char)(((classIndex << 2) * (btnAState == LOW))  & 0xff);
      return 14;
    }
    return 12;
  }
   
  return 0;
}
#endif



void ResetDCL() {


  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print(WiFi.localIP());

  packetIndex=0;

  IPAddress ipAddr;
  
  ssfConfig = "{\"sample_rate\":";
  ssfConfig += SAMPLE_RATE;
  ssfConfig += ",\"version\":1,\"samples_per_packet\":";
  ssfConfig += SAMPLES_PER_PACKET;
  ssfConfig += ",\"column_location\":{";
  ssfConfig += "\"AccelerometerX\":0,\"AccelerometerY\":1,\"AccelerometerZ\":2,\"GyroscopeX\":3,\"GyroscopeY\":4,\"GyroscopeZ\":5";

  if (CAPTURE_TRIGGER) {
      ssfConfig += ",\"Trigger\":6";
  }
  ssfConfig += "}}\r\n";
}

void StreamData(WiFiClient client)
{
  while (isStreaming && client.connected()) {
    if (ButtonPressTime(BUTTON_B)>0) {
      classIndex++;
      if (classIndex>NUM_CLASSES) {
        classIndex = 1;
      }
    }
    #if RECOGNITION
    if (GetSensorDataSampleRecognition()>0)
    {
      int ret = kb_run_model(pData, CHANNELS_PER_SAMPLE, 0);
      if (ret >= 0)
      {
          kb_sprint_model_result(0, serial_out_buf, true, false, false );
          M5.Lcd.println(ret);
          client.write(serial_out_buf);
          kb_reset_model(0);
      }
    }
    #else
    if (GetSensorDataSampleStream(sampleIndex) > 0) {
      sampleIndex++;
      if (sampleIndex >= SAMPLES_PER_PACKET) {
        client.write(dataPacket, PACKET_SIZE * 2);
        sampleIndex = 0;
        packetIndex++;
      }
    } 
    #endif 
  } //if isStreaming
}

void ServiceHttpRequests(WiFiClient c) {

  String httpResponse = "";
  
  httpTime = millis();  
  while (c) {
    if (c.connected() && ((millis() - httpTime) <= TIMEOUT_TIME)) {
      httpTime = millis();
      if (c.available()) {
        char ch = c.read();             // read a byte, then
        Serial.write(ch);                    // print it out the serial monitor
        header += ch;
        if (ch == '\n') {
          crcount++;
        }
        else {
          if (ch != '\r') {
            crcount = 0;
          }
        }
        if (crcount>1) {
          crcount = 0;
        
          if (header.indexOf("GET /config") >= 0) {
              httpResponse = "HTTP/1.1 200 OK\r\nContent-type:text/json\r\n\r\n";
              httpResponse += ssfConfig;
          }  // if "GET /config"
          
          if (header.indexOf("GET /stream") >= 0) {
              isStreaming = true;
              httpResponse = "HTTP/1.1 200 OK\r\nContent-type:application/octet-stream\r\n";
              M5.Lcd.fillScreen(BLACK);
              M5.Lcd.setCursor(0, 0);
              M5.Lcd.println("Streaming Data");         
          }  // if "GET /stream"

          if (header.indexOf("GET /results") >= 0) {
              isStreaming = true;
              httpResponse = "HTTP/1.1 200 OK\r\nContent-type:application/octet-stream\r\n";
              M5.Lcd.fillScreen(BLACK);
              M5.Lcd.setCursor(0, 0);
              M5.Lcd.println("Streaming Results");         
          }  // if "GET /stream"

          if (header.indexOf("GET /disconnect") >= 0) {
              isStreaming = false;
              httpResponse = "HTTP/1.1 200 OK\r\nContent-type:application/octet-stream\r\n\r\n";
              M5.Lcd.fillScreen(BLACK);
              M5.Lcd.setCursor(0, 0);
              M5.Lcd.println("Disconnect");
          }  // if "GET /disconnect"
          if (httpResponse.length()>0) {
              c.println(httpResponse);
              Serial.println(httpResponse);
          }
          header = "";
          if (!isStreaming) {
            c.stop();
          }
          else
          {
            StreamData(c);
          }
        }
      }
    } 
  }
}

void setup() {

  int i = 0;  // timeout variable for connectivity

  M5.begin();
  M5.IMU.Init();
  Serial.begin(115200);
  Serial.println("SensiML Data Capture");
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);

  //Initialize LCD Screen
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);

  M5.IMU.enableFIFO((MPU6886::Fodr)sampleRate); 

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(wifiSSID);
  WiFi.begin(wifiSSID, wifiPassword);
  M5.Lcd.println("Attempting WiFi Connection");
  while ((WiFi.status() != WL_CONNECTED) && (i<60)) {
    delay(500);
    i++;
    M5.Lcd.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print(WiFi.localIP());
    Serial.print("IP:");
    Serial.println(WiFi.localIP());
    server.begin();
  }
  else
  {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print("Failed To Connect");
  }

#if RECOGNITION
 kb_model_init();
#endif

  ResetDCL();
}

void loop(){
    if (isStreaming) {
      isStreaming = false;
      ResetDCL();
    }
    client = server.available();   // Listen for incoming clients
    if (client) {
      ServiceHttpRequests(client);
    }
}

