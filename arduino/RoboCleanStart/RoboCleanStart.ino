
// #define SERIAL_DEBUG
// #define MOTOR_FULLSPEED
// #define NO_FLASH

#include <WiFi101.h>
#include <WiFiMDNSResponder.h>
#ifndef NO_FLASH
  #include <SPIMemory.h>
#endif
#include <Servo.h>
#include <SimpleDHT.h>
#include "qr23_png.h"
#include "qr24_png.h"
#include "appstore_png.h"
#include "playstore_png.h"
#include "eitech_png.h"
#include "index_html.h"

#define FLASH_ADDRESS 31
#define DIST_TRIGGER 14
#define DIST_RECEIVE 13
#define MAX_PACKET_LEN 1400

#define J1 15
#define J2 25
#define J3 18
#define J4 19
#define J5 20
#define J6 21
#define J7 16

#define DHT_PIN 25 // DHT on J2

#define OUTPUT_BUFFER 4096

bool buttonPressed = false;
bool connectWifi = false;
int pressCount = 0;
int timeOut = 500; // milliseconds needed between calls
unsigned long uptime = 0;
unsigned long lastHttpReq = 0;
unsigned long duration = 0;  
unsigned long lastServoSet = 0;
volatile int anemoTicks; 
int anemoRpm = 0;
unsigned long lastAnemoRollback = 0; 
char defConfigDescription[] = "Eitech-Robo-Caterpillar-v1.1";

Servo servoA;
Servo servoL;
Servo servoR; 
SimpleDHT22 dht22;

#ifndef NO_FLASH
SPIFlash flash(24);
#endif

WiFiServer webserver(80); // webserver at port 80
WiFiMDNSResponder mdnsResponder; // MDNS responder
char *httpRequestURL; // HTTP request from client
String jsonReply; // Reply data, usually JSON
String preparedNetworks; // prepare networks in AP mode
char outBuff[OUTPUT_BUFFER];

// bool isAP = false;
// char *ssid = "hulla";
// char *password = "bulla";
// maximum voltage for M1-M4 and S1
// int maxVoltage[5] = { 0, 0, 0, 0, 0 };
int motorSpeed[5] = { 0, 0, 0, 0, 0 };
int servoPos[3] = { -1, -1, -1 };
int jpins[7] = { J1, J2, J3, J4, J5, J6, J7 }; 
int servoQueue[3] = { 90, 90, 90 }; 
const uint8_t pinMotor[4][2] = { { RM1A, RM1B }, { RM2A, RM2B }, { RM3A, RM3B }, { RM4A, RM4B } };
bool testMode = false;
unsigned long lastModeChange = 0; 
int testProg = 0; 
// settings for demo mode
bool demoMode = false;
unsigned long rotateTime = 2000; // milliseconds to rotate
unsigned long obstacleDistance = 3000; // microseconds an obstacle is away before robo rotates in place
unsigned long slideTime = 1000; // milliseconds to rotate when microswitches detect approach in 
 
// char apName[12] = "eitech-robo";

struct settings {
  char fwVersion[14] = "20191024-0900";
  bool isAp = true; 
  char apSSID[32] = "1234567890123456789012345678901";
  char apPSK[64] = "123456789012345678901234567890123456789012345678901234567890123";
  char infSSID[32] = "1234567890123456789012345678901";
  char infPSK[64] = "123456789012345678901234567890123456789012345678901234567890123";
  char devIdent[32] = "1234567890123456789012345678901";
  int maxVoltage[5] = { 0, 0, 0, 0, 0 };
  bool motorInvert[4] = { false, false, false, false };
  bool servoInvert[2] = { false, false };
  uint8_t pinMode[7] = { 0, 1, 0, 0, 0, 3, 3 };
  char configDescription[32] =  "1234567890123456789012345678901";
};

settings defSettings;
settings backupSettings; 
settings flashSettings; 

/*
   start WiFi connection
   ssid:     network SSID
   password: network password
   isAP:     create AP: true or false
*/
bool startWifi() {
  int status;
  WiFi.end();
  WiFi.hostname(defSettings.apSSID); 
  // read config to determine whether running an access point or connecting to an existing network

  if (defSettings.isAp) {
    // Create list of ESSIDs nearby first
    WiFi.begin();
    defSettings.isAp = false;
    listNetworks(true);
    defSettings.isAp = true;
    preparedNetworks = jsonReply;
    WiFi.end(); 
    // create AP
    // Serial.print("Create an open Access Point: ");
    // Serial.println(ssid);
    status = WiFi.beginAP(defSettings.apSSID);
    if (status != WL_AP_LISTENING) {
      // Serial.println("Creating access point failed");
      return false; // return error
    }
  } else {
    // connect to WPA/WPA2 network
    // Serial.print("Attempting to connect to SSID: ");
    // Serial.println(ssid);
    status = WiFi.begin(defSettings.infSSID, defSettings.infPSK);
    if (status != WL_CONNECTED) {
      // Serial.println("Connecting failed");
      return false; // return error
    }
  }
  connectWifi = false;
  for (int i=0; i<10; i++) {
    setSwitch(255); // digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    setSwitch(0); // digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
  webserver.begin();
  return true;
}

void changeMode() {
  // Pin modes are
  // 0: output
  // 1: input - also use for DHT22 on J2!
  // 2: input_pullup - for interrupt, J7 only
  // 3: servo - currently only pin J6 and J7 - changes need reboot!
  // 4: endstop - J1 lower endstop for M1, J2 upper endstop for M1, 
  //    J3 and J4 for M3, J5 and J6 for M4, no endstops for M2
  // 5: endstop as above, but NO ("normally open") 
  for (int i=0; i<7; i++) {
    if (defSettings.pinMode[i] == 0) {
      pinMode(jpins[i], OUTPUT); 
    } else if (defSettings.pinMode[i] == 1) {
      pinMode(jpins[i], INPUT); 
    } else if (defSettings.pinMode[i] == 2) {
      if (i == 6) {
        // J7 configured as interrupt pin for anemometer and similar
        pinMode(jpins[i], INPUT_PULLUP); 
        attachInterrupt(jpins[i], anemoCount, FALLING);
      }
    } else if (defSettings.pinMode[i] == 3) {
      // Do nothing! Servos are attached upon startup!
    } else if (defSettings.pinMode[i] > 3 /* 4 and 5! */ ) {
      pinMode(jpins[i], INPUT_PULLUP); 
    }
  }
}

void anemoCount() {
  anemoTicks++; 
}

void writeFlash() {
  #ifndef NO_FLASH
  flash.powerUp();
  flash.eraseSection(FLASH_ADDRESS, sizeof(defSettings));
  flash.writeAnything(FLASH_ADDRESS, defSettings);
  flash.readAnything(FLASH_ADDRESS, flashSettings);
  // flash.eraseSection(VERS_ADDRESS, 12);
  // flash.writeCharArray(VERS_ADDRESS, backupSettings.fwVersion, 12);
  flash.powerDown();
  #endif
}

void resetToDefaults() {
  #ifndef NO_FLASH
  flash.powerUp();
  flash.eraseSection(FLASH_ADDRESS, sizeof(defSettings));
  flash.writeAnything(FLASH_ADDRESS, backupSettings);
  flash.powerDown();
  #endif
}

/*
   initialize motor port (1...4) and set speed to 0
*/
void initMotor(uint32_t motorNr) {
  if ((motorNr >= 1) && (motorNr <= 4))
  {
    motorNr--;
    pinMode(pinMotor[motorNr][0], OUTPUT);
    pinMode(pinMotor[motorNr][1], OUTPUT);
    if (motorNr == 1) {
      digitalWrite(pinMotor[motorNr][0], LOW);
      digitalWrite(pinMotor[motorNr][1], LOW);
    } else {
      #ifdef MOTOR_FULLSPEED
      digitalWrite(pinMotor[motorNr][0], LOW);
      digitalWrite(pinMotor[motorNr][1], LOW);
      #else
      analogWrite(pinMotor[motorNr][0], 0);
      analogWrite(pinMotor[motorNr][1], 0);
      #endif
    }
  }
}

/*
   show WiFI status
*/
void printWifiStatus() {
  #ifdef SERIAL_DEBUG
  // print the SSID of the network you're attached to:
  Serial.print("WiFi firmware: ");
  Serial.print(WiFi.firmwareVersion());
  Serial.print(" (expected: ");
  Serial.print(WIFI_FIRMWARE_LATEST_MODEL_B);
  Serial.println(")");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  #endif
}

void getVoltage() {
  int value = analogRead(PIN_A2);  // A2 is connected to the power supply (VIN)
  jsonReply += " \"vbat\" : ";
  jsonReply += (value * 3.3 * (62 + 14) / (14 * 1024)); // implicit conversion to string
}

void getMotorSpeeds() {
  jsonReply += " \"motorspeeds\" : [ ";
  for (int i=0; i<5; i++) {
    jsonReply += motorSpeed[i];
    if (i<4) jsonReply += " , ";
  }
  jsonReply += " ] ";
}

void getServoPos() {
  jsonReply += " \"servopos\" : [ ";
  for (int i=0; i<3; i++) {
    jsonReply += servoPos[i];
    if (i<2) jsonReply += " , ";
  }
  jsonReply += " ] ";
}

/*
 * Timeout is in microseconds!
 */

void getDistance(int timeout) {
  digitalWrite(DIST_TRIGGER, HIGH);
  delayMicroseconds(20);
  // delay(500);
  digitalWrite(DIST_TRIGGER, LOW);
  // duration is in microseconds 
  duration = pulseIn(DIST_RECEIVE, HIGH, timeout);
  jsonReply += " \"obstacleDistance\" : ";
  jsonReply += duration; 
  #ifdef SERIAL_DEBUG
  Serial.println(duration); 
  #endif
}

void printSettings(settings sets, char description[16]) {
  jsonReply += " \"";
  jsonReply += description;
  jsonReply += "\" : {\n\"version\" : \"";
  jsonReply += sets.fwVersion;
  jsonReply += "\", \n\"configDescription\" : ";
  jsonReply += sets.configDescription;
  jsonReply += "\", \n\"isAp\" : ";
  jsonReply += sets.isAp;
  jsonReply += ",\n \"accessPoint\" : [ \"";
  jsonReply += sets.apSSID;
  jsonReply += "\", \"";
  jsonReply += sets.apPSK; 
  jsonReply += "\" ],\n \"infraClient\" : [ \"";
  jsonReply += sets.infSSID;
  jsonReply += "\", \"";
  jsonReply += sets.infPSK;
  jsonReply += "\" ],\n \"maxVoltage\" : [ ";
  for (int i=0; i<5; i++) {
    jsonReply += sets.maxVoltage[i];
    if (i<4) jsonReply += ", ";
  }
  jsonReply += " ],\n \"motorInvert\" : [ ";
  for (int i=0; i<4; i++) {
    jsonReply += sets.motorInvert[i];
    if (i<3) jsonReply += ", ";
  }
  jsonReply += " ],\n \"servoInvert\" : [ ";
  for (int i=0; i<2; i++) {
    jsonReply += sets.servoInvert[i];
    if (i<1) jsonReply += ", ";
  }
  jsonReply += " ],\n \"pinMode\" : [ ";
  for (int i=0; i<7; i++) {
    jsonReply += sets.pinMode[i];
    if (i<6) jsonReply += ", ";
  }
  jsonReply += " ] }\n";
}

void listNetworks(bool apMode) {
  // use prepared networks
  if (defSettings.isAp) {
    jsonReply = preparedNetworks;
    return;
  }
  // scan for nearby networks:
  // Serial.println("** Scan Networks **");
  // print the list of networks seen:
  /* Serial.print("number of available networks:");
  Serial.println(numSsid); */
  byte numSsid = WiFi.scanNetworks();
  jsonReply = "{ \"networks\" : [ ";
  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    /* Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.print("\tEncryption: ");
    Serial.println(WiFi.encryptionType(thisNet)); */
    jsonReply += " { \"ssid\" : \"" ;
    jsonReply += WiFi.SSID(thisNet);
    jsonReply += "\" , \n \"signal\" : ";
    jsonReply += WiFi.RSSI(thisNet);
    jsonReply += " , \n \"enctype\" : ";
    jsonReply += WiFi.encryptionType(thisNet);
    if (thisNet < numSsid-1) {
      jsonReply += " },\n ";
    } else { 
      jsonReply += " }\n ";
    }
  }
  jsonReply += " ] ,\n \"isap\" : ";
  jsonReply += apMode;
  jsonReply += ", \"ssid\" : \"";
  jsonReply += WiFi.SSID();
  byte mac[6];
  char str[2];
  jsonReply += "\", \"mac\" : \"";
  WiFi.macAddress(mac);
  for (int j = 5; j > 0; j--) {
    utoa((unsigned int) mac[j], str, 16);
    jsonReply += str;
    jsonReply += ":";
  }
  utoa((unsigned int) mac[0], str, 16);
  jsonReply += str;
  jsonReply += "\", \"signal\" : ";
  jsonReply += WiFi.RSSI();
  jsonReply += ", \"addr\" : \"";
  IPAddress ip = WiFi.localIP();
  for (int j = 0; j < 4; j++) {
    jsonReply += ip[j];
    if (j<3) jsonReply += ".";
  }
  jsonReply += "\", \"fwInstalled\" : \"";
  jsonReply += WiFi.firmwareVersion();
  jsonReply += "\", \"fwExpected\" : \"";
  jsonReply += WIFI_FIRMWARE_LATEST_MODEL_B;
  jsonReply += "\", \n";
  getVoltage();
  jsonReply += " }\n";

  // for( int i=0 ; i<jsonReply.length() ; i++) {
  //   Serial.print(jsonReply.charAt(i));
  // }
}

void getNet() {
  byte mac[6];
  char str[2];
  WiFi.macAddress(mac);
  jsonReply += "\"mac\" : \"";
  for (int j = 5; j > 0; j--) {
    utoa((unsigned int) mac[j], str, 16);
    jsonReply += str;
    jsonReply += ":";
  }
  utoa((unsigned int) mac[0], str, 16);
  jsonReply += str;
  jsonReply += "\", \"signal\" : ";
  jsonReply += WiFi.RSSI();
  jsonReply += ", \"addr\" : \"";
  IPAddress ip = WiFi.localIP();
  for (int j = 0; j < 4; j++) {
    jsonReply += ip[j];
    if (j<3) jsonReply += ".";
  }
  jsonReply += "\", \"fwInstalled\" : \"";
  jsonReply += WiFi.firmwareVersion();
  jsonReply += "\", \"fwExpected\" : \"";
  jsonReply += WIFI_FIRMWARE_LATEST_MODEL_B;
  jsonReply += "\",\n";
}

void getDht() {
  float temperature = 0;
  float humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht22.read2(DHT_PIN, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); Serial.println(err);delay(2000);
    jsonReply += "\"dhterror\" : true";
    return;
  }
  #ifdef SERIAL_DEBUG
  Serial.print("Sample OK: ");
  Serial.print((float)temperature); Serial.print(" *C, ");
  Serial.print((float)humidity); Serial.println(" RH%");
  #endif
  jsonReply += "\"dhterror\" : false, \"temperature\" : ";
  jsonReply += temperature;
  jsonReply += " , \"humidity\" : ";
  jsonReply += humidity;
}

bool setMaxVoltage(String outport, String volt) {
  bool retcode = false;
  if (outport == "m1" || outport == "M1") {
    defSettings.maxVoltage[0] = volt.toInt();
    retcode = true;
  } else if (outport == "m2" || outport == "M2") {
    defSettings.maxVoltage[1] = volt.toInt();
    retcode = true;
  } else if (outport == "m3" || outport == "M3") {
    defSettings.maxVoltage[2] = volt.toInt();
    retcode = true;
  } else if (outport == "m4" || outport == "M4") {
    defSettings.maxVoltage[3] = volt.toInt();
    retcode = true;
  } else if (outport == "s1" || outport == "S1") {
    defSettings.maxVoltage[4] = volt.toInt();
    retcode = true;
  }
  getMaxVoltages();
  return retcode;
}

void getMaxVoltages() {
  jsonReply += "\"maxvoltages\" : [ \n";
  for (int i = 0; i < 5; i++) {
    jsonReply += defSettings.maxVoltage[i];
    if (i<4) jsonReply += " , ";
  }
  jsonReply += " ] ";
}

void setSwitch(int value) {
  int rawVoltage;
  int absMax = 255;
  rawVoltage  = analogRead(PIN_A2);
  if (defSettings.maxVoltage[4] > 0) {
    // Serial.println(rawVoltage);
    int calcVoltage = ( rawVoltage * ( 62 + 14 ) * 33 / 1023 / 14 );
    #ifdef SERIAL_DEBUG
    // Serial.print("Voltage, divide by 10: ");
    // Serial.println(calcVoltage);
    #endif
    if (calcVoltage >  defSettings.maxVoltage[4]) {
      value = value * defSettings.maxVoltage[4] / calcVoltage;
      absMax = 255 * defSettings.maxVoltage[4] / calcVoltage;
    }
    if (value > absMax) {
      value = absMax;
    }
  }
  motorSpeed[4] = value;
  if (value > absMax) motorSpeed[4] = absMax;
  #ifdef SERIAL_DEBUG
  // Serial.print("Set switch to PWM: ");
  // Serial.println(value);
  #endif
  analogWrite(LED_BUILTIN, value);
}

/*
   set motor speed
   motorNr: 1..4
   value:   -255...255
   breaks:  motor break activated, when true
*/
void setMotorSpeed(int motorNr, int value, bool brakes = false) {
  int in1, in2;
  int rawVoltage;
  int endstop;
  if ((motorNr >= 1) && (motorNr <= 4)) {
    // first check the endstops: 
    if (motorNr == 1 && defSettings.pinMode[0] > 3 && value < 0) {
      endstop = digitalRead(jpins[0]);
      if (defSettings.pinMode[0] == 4 && endstop == HIGH) {
        // M1 lower endstop NC reached
        value = 0;
      } else if (defSettings.pinMode[0] == 5 && endstop == LOW) {
        // M1 lower endstop NO reached
        value = 0;
      }
    } else if (motorNr == 1 && defSettings.pinMode[1] > 3 && value > 0) {
      endstop = digitalRead(jpins[1]);
      if (defSettings.pinMode[1] == 4 && endstop == HIGH) {
        // M1 upper endstop NC reached
        value = 0;
      } else if (defSettings.pinMode[1] == 5 && endstop == LOW) {
        // M1 upper endstop NO reached
        value = 0;
      }
    } else if (motorNr == 3 && defSettings.pinMode[2] > 3 && value < 0) {
      endstop = digitalRead(jpins[2]);
      if (defSettings.pinMode[2] == 4 && endstop == HIGH) {
        // M3 lower endstop NC reached
        value = 0;
      } else if (defSettings.pinMode[2] == 5 && endstop == LOW) {
        // M3 lower endstop NO reached
        value = 0;
      }
    } else if (motorNr == 3 && defSettings.pinMode[3] > 3 && value > 0) {
      endstop = digitalRead(jpins[3]);
      if (defSettings.pinMode[3] == 4 && endstop == HIGH) {
        // M3 upper endstop NC reached
        value = 0;
      } else if (defSettings.pinMode[3] == 5 && endstop == LOW) {
        // M3 upper endstop NO reached
        value = 0;
      }
    } else if (motorNr == 4 && defSettings.pinMode[4] > 3 && value < 0) {
      endstop = digitalRead(jpins[4]);
      if (defSettings.pinMode[4] == 4 && endstop == HIGH) {
        // M4 lower endstop NC reached
        value = 0;
      } else if (defSettings.pinMode[4] == 5 && endstop == LOW) {
        // M4 lower endstop NO reached
        value = 0;
      } 
    } else if (motorNr == 4 && defSettings.pinMode[5] > 3 && value < 0) {
      endstop = digitalRead(jpins[5]);
      if (value > 0 && defSettings.pinMode[5] == 4 && endstop == HIGH) {
        // M4 upper endstop NC reached
        value = 0;
      } else if (value > 0 && defSettings.pinMode[5] == 5 && endstop == LOW) {
        // M4 upper endstop NO reached
        value = 0;
      }
    }
    motorNr--;
    int absMax = 255;
    motorSpeed[motorNr] = value;
    if (value > absMax) motorSpeed[motorNr] = absMax;
    if (defSettings.maxVoltage[motorNr] > 0) {
      rawVoltage  = analogRead(PIN_A2);
      // Serial.println(rawVoltage);
      int calcVoltage = ( rawVoltage * ( 62 + 14 ) * 33 / 1023 / 14 );
      if (calcVoltage >  defSettings.maxVoltage[motorNr]) {
        value = value * defSettings.maxVoltage[motorNr] / calcVoltage;
        absMax = 255 * defSettings.maxVoltage[motorNr] / calcVoltage;
      }
      #ifdef SERIAL_DEBUG
      // Serial.print("Voltage, divide by 10: ");
      // Serial.println(calcVoltage);
      #endif
    }
    if (value >= 0) {
      if (value > absMax)
        value = absMax;
      if (brakes) {
        in1 = absMax;
        in2 = absMax - value;
      } else {
        in1 = value;
        in2 = 0;
      }
    }
    else  // value is negative, reverse motor
    {
      value = -value;
      if (value > absMax)
        value = absMax;

      if (brakes)
      {
        in1 = absMax - value;
        in2 = absMax;
      }
      else
      {
        in1 = 0;
        in2 = value;
      }
    }
    if (motorNr == 1) {
      if (in1 > 240) in1 = HIGH;
      if (in1 < 16)  in1 = LOW;
      if (in2 > 240) in2 = HIGH;
      if (in2 < 16)  in2 = LOW;
      digitalWrite(pinMotor[motorNr][0], in1);
      digitalWrite(pinMotor[motorNr][1], in2);
      #ifdef SERIAL_DEBUG
      // Serial.print("Set motor to PWM: ");
      // Serial.println(value);
      #endif
    } else {
      #ifdef MOTOR_FULLSPEED
      if (in1 > 127) {
        in1 = HIGH;
      } else {
        in1 = LOW;
      }
      if (in2 > 127) {
        in2 = HIGH;
      } else {
        in2 = LOW;
      }
      digitalWrite(pinMotor[motorNr][0], in1);
      digitalWrite(pinMotor[motorNr][1], in2);
      #else
      analogWrite(pinMotor[motorNr][0], in1);
      analogWrite(pinMotor[motorNr][1], in2);
      #endif
    }
  }
}

void checkEndstops() {
  int pinval = 0;
  for (int i=0; i<6; i++) {
    if (defSettings.pinMode[i] > 3) {
      pinval = digitalRead(jpins[i]);
      if ( (defSettings.pinMode[i] == 4 && pinval == HIGH) ||
        (defSettings.pinMode[i] == 5 && pinval == LOW) ) {
          if ( (i == 0 && motorSpeed[0] < 0) ||
            (i == 1 && motorSpeed[0] > 0) ) {
            setMotorSpeed(1, 0);
          } else if ( (i == 2 && motorSpeed[2] < 0) ||
            (i == 3 && motorSpeed[2] > 0) ) {
            setMotorSpeed(3, 0);
          } else if ( (i == 4 && motorSpeed[3] < 0) ||
            (i == 5 && motorSpeed[3] > 0) ){
            setMotorSpeed(4, 0);
          }
        }
    }
  }
}

/*
   split HTTP request string
*/
bool httpSplitRequest(char *httpRequest, size_t len) {
  char *reqpart[2];
  if (len >= 3) {
    reqpart[0] = strtok(httpRequest, " \t\n");
    if (strncmp(reqpart[0], "GET\0", 4) == 0) {
      httpRequestURL = strtok(NULL, " \t"); // url
      //if(!strncmp(httpRequestURL, "/\0", 2)) // rename empty url to index.html
      //  httpRequestURL = "/index.html";
      reqpart[1] = strtok (NULL, " \t\n");
      if (!strncmp(reqpart[1], "HTTP/1.0", 8) || \
          !strncmp(reqpart[1], "HTTP/1.1", 8)) {
#ifdef DEBUG
        //Serial.println(reqpart[0]);
        //Serial.println(reqpart[1]);
        //Serial.println(httpRequestURL);
#endif
        return true;
      }
    }
  }
  // wrong request
  return false;
}

void httpSendData(WiFiClient &client, const char *data, int len) {
  while(len > 0)
  {
    int sendlen = len;
    if(len > MAX_PACKET_LEN)
      sendlen = MAX_PACKET_LEN;

    client.write(data, sendlen);
    data += sendlen;
    len -= sendlen;
  }
}

/*
   get HTTP resource string
*/
String httpGetResource(int nr) {
  String resource = "";
  char *pnt = httpRequestURL;
  if (*pnt == '/') {
    while (*pnt) {
      if (*pnt++ == '/') {
        if (nr-- <= 0) {
          while (*pnt != '\0' && *pnt != '/') {
            resource += *pnt++;
          }
          break;
        }
      }
    }
  }
  return resource;
}

void stopEverything() {
  for (int i = 1; i < 5; i++) {
    setMotorSpeed(i, 0);
  }
  setSwitch(0);
  // setServoPos(0, 90);
  // setServoPos(1, 90); 
  testMode = false;
  jsonReply = "{ \"stopped\" : 1, \n";
  getVoltage();
  jsonReply += " }\n";
}

// obsolete? FIXME, clean out
void setServoPos(int snum, int spos) {
        if (snum == 0 && defSettings.pinMode[6] == 3) {
                servoR.write(spos);
        } else if (snum == 1 && defSettings.pinMode[5] == 3) {
                servoL.write(spos);
        } else if (snum == 2 && defSettings.pinMode[4] == 3) {
                servoA.write(spos);
        }
}

void clearServoQueue() {
  for (int i=0; i<3; i++) {
    if (i == 0 && defSettings.pinMode[6] == 3 && servoQueue[i] > -1) {
      servoR.write(servoQueue[i]);
    } else if (i == 1 && defSettings.pinMode[5] == 3 && servoQueue[i] > -1) {
      servoL.write(servoQueue[i]);
    } else if (i == 2 && defSettings.pinMode[4] == 3 && servoQueue[i] > -1) {
      servoA.write(servoQueue[i]);
    }
    servoQueue[i] = -1;
  }
  lastServoSet = millis();
}

void setup() {
  char defAp[] = "eitech-robo-abcd"; 
  char defPSK[] = "empty";
  byte mac[6];
  char mstr[2];
  WiFi.begin();
  WiFi.macAddress(mac);
  utoa((unsigned int) mac[1], mstr, 16);
  defAp[12] = mstr[0];
  defAp[13] = mstr[1];
  utoa((unsigned int) mac[0], mstr, 16);
  defAp[14] = mstr[0];
  defAp[15] = mstr[1];
  strcpy(defSettings.apSSID, defAp);
  strcpy(defSettings.apPSK, defPSK);
  strcpy(defSettings.infSSID, defAp);
  strcpy(defSettings.infPSK, defPSK);
  strcpy(defSettings.configDescription, defConfigDescription);
  defSettings.isAp = true; 
  // defSettings.apSSID = defAp; 
  backupSettings = defSettings; 
  flashSettings = defSettings;
  pinMode(DIST_TRIGGER, OUTPUT);
  pinMode(DIST_RECEIVE, INPUT);
  
  // init pins
  initMotor(1);
  initMotor(2);
  initMotor(3);
  initMotor(4);
  // pinMode(RS1, OUTPUT);
  // init analog inputs
  analogReadResolution(10); // 10 bit (0...1023)
  analogReference(AR_DEFAULT); // internal reference
  
  #ifdef SERIAL_DEBUG
  // init serial port
  Serial.begin(9600);
  while (!Serial) { /* wait for serial port to connect */ } 
  #endif

  #ifndef NO_FLASH  
  flash.begin();
  flash.powerUp();
  flash.setClock(1000000); 
  #ifdef SERIAL_DEBUG
  Serial.println(flash.getCapacity());
  #endif
  flash.readAnything(FLASH_ADDRESS, flashSettings);
  // char flashVersion[12];
  // flash.readCharArray(VERS_ADDRESS, flashVersion, 12);
  #endif 
  
  if ( memcmp( (const void *)flashSettings.fwVersion, (const void *)backupSettings.fwVersion, sizeof(backupSettings.fwVersion)) == 0) {
  // if ( memcmp( (const void *)flashVersion, (const void *)backupSettings.fwVersion, sizeof(backupSettings.fwVersion)) == 0) {
    #ifdef SERIAL_DEBUG
    Serial.println("Version matches!");
    #endif
    defSettings = flashSettings; 
    #ifdef SERIAL_DEBUG
    Serial.println(flashSettings.infSSID);
    Serial.println(flashSettings.isAp);
    #endif
  } else {
    #ifdef SERIAL_DEBUG
    Serial.println("Writing default config to flash..."); 
    #endif
    // flash.eraseSection(VERS_ADDRESS, 12);
    #ifndef NO_FLASH  
    flash.eraseSection(FLASH_ADDRESS, sizeof(defSettings));
    flash.writeAnything(FLASH_ADDRESS, defSettings);
    // flash.writeCharArray(VERS_ADDRESS, backupSettings.fwVersion, 12);
    flash.readAnything(FLASH_ADDRESS, flashSettings);
    #endif
  }
  if (defSettings.pinMode[4] == 3) {
    servoA.attach(J5);
    setServoPos(2, 90);
    servoPos[2] = 90;
  }
  if (defSettings.pinMode[5] == 3) {
    servoL.attach(J6);
    setServoPos(1, 90);
    servoPos[1] = 90;
  }
  if (defSettings.pinMode[6] == 3) {
    servoR.attach(J7);
    setServoPos(0, 90);
    servoPos[0] = 90;
  }
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_A2, INPUT);
  if (analogRead(PIN_A2) < 20) {
    testMode = true;
    #ifdef SERIAL_DEBUG
    Serial.println("Entering test mode!");
    #endif
  }
  changeMode(); 
  #ifdef SERIAL_DEBUG
  Serial.println(backupSettings.fwVersion);
  Serial.println(flashSettings.fwVersion);
  /* Serial.println(backupSettings.apSSID);
  Serial.print(flashSettings.apSSID); */
  /* for (uint8_t i = 0; i < 11; i++) {
    Serial.print(backupSettings.apSSID[i]);
    Serial.print(",");
  } */ 
  Serial.println();
  #endif
  // wait for WiFi connection
  if (!startWifi()) {
    delay(5000); // wait 5 seconds and retry
    if (!startWifi()) {
      defSettings.isAp = true;
      if (!startWifi()) {
        #ifdef SERIAL_DEBUG
        Serial.println("Starting the WiFi module failed!");
        #endif
      }
    }
  }
  if (analogRead(PIN_A2) < 20) {
    testMode = true;
    #ifdef SERIAL_DEBUG
    Serial.println("Entering test mode!");
    #endif
  }
  // you're connected now, so print out the status
  printWifiStatus();
}

void loop() {
  checkEndstops();
  if (testMode == true) {
    timeOut = 0;
    if (millis() - lastModeChange > 5000) {
      if (testProg == 0) {
        #ifdef SERIAL_DEBUG
        Serial.println("TEST MODE: Full forward, all servos 0");
        #endif
        // both motors forward 
        setMotorSpeed(1, 255);
        setMotorSpeed(3, 255);
        // both servos 0
        setServoPos(0, 0);
        setServoPos(1, 0);
        // S1 on
        setSwitch(255);
        testProg = 1;
      } else if (testProg == 1) {
        #ifdef SERIAL_DEBUG
        Serial.println("TEST MODE: mixed 1");
        #endif
        // M1 forward, M3 reverse
        setMotorSpeed(3, -255);
        // servo 1 180, servo 2 0
        setServoPos(0, 180);
        // S1 off
        setSwitch(0);
        testProg = 2;
      } else if (testProg == 2) {
        #ifdef SERIAL_DEBUG
        Serial.println("TEST MODE: Full reverse, all servos 180");
        #endif
        // both motors reverse
        setMotorSpeed(1, -255);
        // servo 1 180, servo 2 180
        setServoPos(1, 180);
        // S1 on
        setSwitch(255);
        testProg = 3; 
      } else if (testProg == 3) {
        #ifdef SERIAL_DEBUG
        Serial.println("TEST MODE: mixed 2");
        #endif
        // M1 reverse, M3 forward
        setMotorSpeed(1, 255);
        // servo 1 0, servo 2 180
        setServoPos(0, 0);
        // S1 off
        setSwitch(0);
        testProg = 0; 
      }
      lastModeChange = millis();  
    }
  } 
  if (demoMode == true) {
    // turn left
    if (digitalRead(15) == HIGH) {
        #ifdef SERIAL_DEBUG
        Serial.println("Turn left!");
        #endif
        setMotorSpeed(3, 0);
        setMotorSpeed(1, 0);
        setMotorSpeed(1, -255);
        delay(slideTime);
    } else if (digitalRead(18) == HIGH) {
        #ifdef SERIAL_DEBUG
        Serial.println("Turn right!");
        #endif
        setMotorSpeed(1, 0);
        setMotorSpeed(3, 0);
        setMotorSpeed(3, -255);
        delay(slideTime);
    } else {
      // obstacle in front?
      getDistance(20000);
      if (duration < obstacleDistance && duration > 0) {
        #ifdef SERIAL_DEBUG
        Serial.println("Rotate in place!");
        #endif
        if (millis() % 2 == 0) {
          setMotorSpeed(3, 255);
          setMotorSpeed(1, -255);
        } else {
          setMotorSpeed(1, 255);
          setMotorSpeed(3, -255);
        }
        delay(rotateTime);
      } else {
        #ifdef SERIAL_DEBUG
        Serial.println("Straight forward!");
        #endif
        if (motorSpeed[0] < 255) setMotorSpeed(1, 255);
        if (motorSpeed[2] < 255) setMotorSpeed(3, 255);
        delay(250); 
      }
    }
  }
  if (millis() - lastServoSet > 80) { clearServoQueue(); }
  if (timeOut > 0) {
    if (millis() - lastHttpReq > timeOut && timeOut > 0) {
      stopEverything(); 
    }
  }
  if (analogRead(PIN_A2) > 19) {
    pressCount = 0;
  }
  if (analogRead(PIN_A2) < 20) {
    // six seconds to reset to AP mode
    pressCount += 1;
    if (pressCount == 1) {
      stopEverything();
    } else if (150 > pressCount > 12) {
      setSwitch(255); // digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      setSwitch(0); // digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    } else {
      setSwitch(255); // digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      setSwitch(0); // digitalWrite(LED_BUILTIN, LOW);
      delay(450);
    }
  }
  if (pressCount > 150) {
    resetToDefaults(); 
  } else if (pressCount == 12) {
    connectWifi = true;
    defSettings.isAp = !defSettings.isAp; 
  }
  // Hard reset required after
  while (pressCount > 150) {
    setSwitch(255); // digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    setSwitch(0); // digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
  if (connectWifi) {
    stopEverything(); 
    printWifiStatus();
    if (!startWifi()) {
      delay(2000);
      #ifdef SERIAL_DEBUG
      Serial.println("Starting the WiFi module failed, trying to switch to AP mode");
      #endif
      defSettings.isAp = true;
      startWifi(); 
    }
    printWifiStatus();
    connectWifi = false; 
  }
  WiFiClient client = webserver.available();
  uptime = millis();
  if (uptime - lastAnemoRollback > 10000 && anemoTicks > 25) {
    anemoRpm = anemoTicks * 6; 
    anemoTicks = 0;
    lastAnemoRollback = uptime;
    #ifdef SERIAL_DEBUG
    Serial.print("Anemometer RPM: ");
    Serial.println(anemoRpm); 
    #endif
  } else if (uptime - lastAnemoRollback > 30000) {
    anemoRpm = anemoTicks * 2; 
    anemoTicks = 0;
    lastAnemoRollback = uptime;
    #ifdef SERIAL_DEBUG
    Serial.print("Anemometer RPM: ");
    Serial.println(anemoRpm); 
    #endif
  }
  if (client) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    char httpRequest[1024];
    size_t httpRequestLen = 0;

    while (client.connected()) {
      if (client.available()) {
        lastHttpReq = millis();
        char c = client.read();
        if (httpRequestLen < (sizeof(httpRequest) - 1)) {
          httpRequest[httpRequestLen++] = c;
          httpRequest[httpRequestLen] = '\0';
        }
        #ifdef SERIAL_DEBUG
        Serial.print("HTTP REQUEST: ");
        Serial.println(httpRequest);
        #endif
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          for ( int i = 0; i < OUTPUT_BUFFER; i++) {
            outBuff[i] = '\0';
          }
          if (httpSplitRequest(httpRequest, httpRequestLen)) {
            if (httpGetResource(0) == "stop") {
              stopEverything();
              demoMode = false;
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            } 
            else if (httpGetResource(0) == "test") {
              testMode = true; 
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            } 
            else if (httpGetResource(0) == "demo") {
              pinMode(15, INPUT_PULLUP);
              pinMode(18, INPUT_PULLUP); 
              delay(5);
              if (digitalRead(15) == LOW && digitalRead(18) == LOW) {
                demoMode = true; 
                timeOut = 0;
              }
              rotateTime = httpGetResource(1).toInt();
              obstacleDistance = httpGetResource(2).toInt();
              slideTime = httpGetResource(3).toInt();
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "wificonnect") {
              httpGetResource(1).toCharArray(defSettings.infSSID, 32);
              httpGetResource(2).toCharArray(defSettings.infPSK, 64);
              defSettings.isAp = false;
              #ifdef SERIAL_DEBUG
              Serial.println(defSettings.infSSID);
              Serial.println(defSettings.infPSK);
              #endif
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              connectWifi = true;
              writeFlash();
              break;
            } 
            else if (httpGetResource(0) == "switchnet") {
              defSettings.isAp = !defSettings.isAp;
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              connectWifi = true;
              break;
            }
            else if (httpGetResource(0) == "hostname") {
              httpGetResource(1).toCharArray(defSettings.apSSID, 32);
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              connectWifi = true;
              writeFlash();
              break;
            }
            else if (httpGetResource(0) == "write") {
              writeFlash(); 
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "timeout") {
              timeOut = httpGetResource(1).toInt();
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "getdistance") { 
              int timeout = httpGetResource(1).toInt();
              jsonReply = "{ ";
              getDistance(timeout);
              jsonReply += ",\n";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "get") {
              // Serial.println("Get current state");
              jsonReply = "{ ";
              printSettings(defSettings, "currentSettings");
              jsonReply += ", \n";
              printSettings(flashSettings, "flashSettings");
              jsonReply += ", \n";
              printSettings(backupSettings, "defaultSettings");
              jsonReply += ", \n";
              getNet();
              getVoltage();
              jsonReply += " , \n ";
              getMaxVoltages();
              jsonReply += " , \n ";
              getMotorSpeeds(); 
              jsonReply += " , \n ";
              getServoPos(); 
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "setmotor") {
              // Serial.println("Setting motors");
              setMotorSpeed(1, httpGetResource(1).toInt());
              setMotorSpeed(2, httpGetResource(2).toInt());
              setMotorSpeed(3, httpGetResource(3).toInt());
              setMotorSpeed(4, httpGetResource(4).toInt());
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " , \n ";
              getMaxVoltages();
              jsonReply += " , \n ";
              getMotorSpeeds(); 
              jsonReply += " , \n ";
              getServoPos(); 
              jsonReply += "\n}\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "setswitch") {
              // Serial.println("Setting motors");
              setSwitch(httpGetResource(1).toInt());
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " , \n ";
              getMaxVoltages();
              jsonReply += " , \n ";
              getMotorSpeeds(); 
              jsonReply += " , \n ";
              getServoPos(); 
              jsonReply += "\n}\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "setservo") {
              // Serial.println("Setting motors");
              // setServoPos(httpGetResource(1).toInt(), httpGetResource(2).toInt());
              servoQueue[httpGetResource(1).toInt()] = httpGetResource(2).toInt();
              servoPos[httpGetResource(1).toInt()] = httpGetResource(2).toInt();
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " , \n ";
              getMaxVoltages();
              jsonReply += " , \n ";
              getMotorSpeeds(); 
              jsonReply += " , \n ";
              getServoPos(); 
              jsonReply += "\n}\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "getwifi") {
              // Serial.println("Getting WiFi parameters and networks");
              listNetworks(defSettings.isAp);
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "setmode") {
              // Pin modes are
              // 0: output
              // 1: input - also use for DHT22 on J2!
              // 2: input_pullup - for interrupt, J7 only
              // 3: servo - currently pin J7, J6 and J5 - changes here need reboot!
              // 4: endstop NC (typical configuration for microswitches)
              // 5: endstop NO (reed contact)
              for (int i=0; i<7; i++) {
                if (httpGetResource(i+1) != "") {
                  defSettings.pinMode[i] = httpGetResource(i+1).toInt();
                }
              }
              changeMode(); 
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "setanalog") {
              // Serial.println("Setting analogue output on J1 to J7");
              for (int i=0; i<7; i++) {
                  if (httpGetResource(i+1) != "") {
                    if (defSettings.pinMode[i] == 0) {
                      analogWrite(jpins[i], httpGetResource(i+1).toInt()); 
                    }
                  }
              }
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "getanalog") {
              // Serial.println("Getting analogue input on J1 to J7");
              int value = -1;
              int idx = -1;
              if (httpGetResource(1) == "j1" || httpGetResource(1) == "J1") {
                idx = 0;
              } else if (httpGetResource(1) == "j2" || httpGetResource(1) == "J2") {
                idx = 1;
              } else if (httpGetResource(1) == "j3" || httpGetResource(1) == "J3") {
                idx = 2;
              } else if (httpGetResource(1) == "j4" || httpGetResource(1) == "J4") {
                idx = 3;
              } else if (httpGetResource(1) == "j5" || httpGetResource(1) == "J5") {
                idx = 4;
              } else if (httpGetResource(1) == "j6" || httpGetResource(1) == "J6") {
                idx = 5;
              } else if (httpGetResource(1) == "j7" || httpGetResource(1) == "J7") {
                idx = 6;
              }
              if (idx>-1 && defSettings.pinMode[idx] == 1) {
                value = analogRead(jpins[idx]); 
              }
              jsonReply = "{ \"analogValue\" : ";
              jsonReply += value;
              jsonReply += ", \"pinName\" : \"j";
              jsonReply += (idx + 1); 
              jsonReply += "\" ,\n";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "setmax") {
              // Serial.println("Setting maximum voltage of motors M1 to m4");
              setMaxVoltage(httpGetResource(1), httpGetResource(2));
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "getrpm") {
              // Serial.println("Get the distance of HC-SR04");
              // modes of port: 0 output
              //                1 input
              //                2 interrupt 
              // Serial.println("Changing mode of Port");
              jsonReply = "{ \"rpm\" : ";
              jsonReply += anemoRpm;
              jsonReply += " ,\n"; 
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "getdht") {
              jsonReply = "{ ";
              getDht();
              jsonReply += " ,\n"; 
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              #ifdef SERIAL_DEBUG
              Serial.print("REPLY: ");
              Serial.println(outBuff);
              #endif
              client.write(outBuff);
              break;
            }
            else if (httpGetResource(0) == "img") {
              client.write("HTTP/1.0 200 OK\r\nContent-type: image/png\r\n\r\n");
              // httpSendData(client, (const char *)background_jpg, sizeof(background_jpg));
              int len = 0;
              if (httpGetResource(1) == "qr23.png") {
                httpSendData(client, (const char *)qr23_png, sizeof(qr23_png));
              } else if (httpGetResource(1) == "qr24.png") {
                httpSendData(client, (const char *)qr24_png, sizeof(qr24_png));
              } else if  (httpGetResource(1) == "appstore.png") {
                httpSendData(client, (const char *)appstore_png, sizeof(appstore_png));
              } else if  (httpGetResource(1) == "playstore.png") {
                httpSendData(client, (const char *)playstore_png, sizeof(playstore_png));
              } else if  (httpGetResource(1) == "eitech.png") {
                httpSendData(client, (const char *)eitech_png, sizeof(eitech_png));
              } 
              break;
            }
            else if (httpGetResource(0) == "update.js") {
              // HTTP/1.1 302 Moved Temporarily
              if (defSettings.isAp) {
                client.write("HTTP/1.0 200 OK\r\nContent-type: aplication/javascript\r\n\r\n/* DUMMY */\r\n\r\n");                  
              } else {
                client.write("HTTP/1.0 302 Moved Temporarily\r\nLocation: https://static.eitech-robotics.de/js/robo-"); 
                client.write(defSettings.fwVersion);
                client.write(".js\r\n\r\n");
              }
            }
            else if (httpGetResource(0) == "index.html" || httpGetResource(0) == "") {
            // Serial.println(httpRequestURL);
            // client.write("HTTP/1.0 200 OK\nContent-type: text/plain\n\nHallo Welt!\n\n");
              client.write("HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n");
              httpSendData(client, (const char *)index_html, sizeof(index_html));
              break; 
            }
            client.write("HTTP/1.0 200 OK\nContent-type: text/plain\n\nHallo Welt!\n\n");
            break;
          }
        }
      }
    }
    client.stop();
  }
}
