#include <WiFi101.h>
#include <WiFiMDNSResponder.h>
#include <SPIMemory.h>
#include <Servo.h>
#include <SimpleDHT.h>

// #define SERIAL_DEBUG

#define FLASH_ADDRESS 15

#define DIST_TRIGGER 14
#define DIST_RECEIVE 13

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
unsigned long duration = 0;  
volatile int anemoTicks; 
int anemoRpm = 0;
unsigned long lastAnemoRollback = 0; 

Servo servoL;
Servo servoR; 
SimpleDHT22 dht22;

SPIFlash flash(24);
WiFiServer webserver(80); // webserver at port 80
char *httpRequestURL; // HTTP request from client
String jsonReply; // Reply data, usually JSON
char outBuff[OUTPUT_BUFFER];

// bool isAP = false;
// char *ssid = "hulla";
// char *password = "bulla";
// maximum voltage for M1-M4 and S1
// int maxVoltage[5] = { 0, 0, 0, 0, 0 };
int motorSpeed[5] = { 0, 0, 0, 0, 0 };
int jpins[5] = { J1, J2, J3, J4, J7 }; 
const uint8_t pinMotor[4][2] = { { RM1A, RM1B }, { RM2A, RM2B }, { RM3A, RM3B }, { RM4A, RM4B } };

// char apName[12] = "eitech-robo";

struct settings {
  char fwVersion[14] = "20190812-1145";
  bool isAp = true; 
  char apSSID[32] = "1234567890123456789012345678901";
  char apPSK[64] = "123456789012345678901234567890123456789012345678901234567890123";
  char infSSID[32] = "1234567890123456789012345678901";
  char infPSK[64] = "123456789012345678901234567890123456789012345678901234567890123";
  int maxVoltage[5] = { 0, 0, 0, 0, 0 };
  bool motorInvert[4] = { false, false, false, false };
  bool servoInvert[2] = { false, false };
  uint8_t pinMode[5] = { 0, 0, 0, 0, 0 };
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
  // read config to determine whether running an access point or connecting to an existing network
  // bool isAP = false;
  // char *ssid = DEFSSID;
  // char *password = DEFPASSWORD;
  // WiFi.hostname(mdnsName); //use MDNS name as host name (DHCP option 12)

  if (defSettings.isAp) {
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
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
  webserver.begin();
  return true;
}

void changeMode() {
  for (int i=0; i<5; i++) {
    if (defSettings.pinMode[i] == 0) {
      pinMode(jpins[i], OUTPUT); 
    } else if (defSettings.pinMode[i] == 1) {
      pinMode(jpins[i], INPUT); 
    } else if (defSettings.pinMode[i] == 2) {
      if (i == 4 || i == 1) {
        // J7 configured as interrupt pin for anemometer and similar
        pinMode(jpins[i], INPUT_PULLUP); 
        attachInterrupt(jpins[i], anemoCount, FALLING);
      } else if (i == 1) {
        // J2 setup for DHT22
        pinMode(jpins[i], INPUT); 
      }
    }
  }
}

void anemoCount() {
  anemoTicks++; 
}

void writeFlash() {
  flash.powerUp();
  flash.eraseSection(FLASH_ADDRESS, sizeof(defSettings));
  flash.writeAnything(FLASH_ADDRESS, defSettings);
  flash.readAnything(FLASH_ADDRESS, flashSettings);
  // flash.eraseSection(VERS_ADDRESS, 12);
  // flash.writeCharArray(VERS_ADDRESS, backupSettings.fwVersion, 12);
  flash.powerDown();
}

void resetToDefaults() {
  flash.powerUp();
  flash.eraseSection(FLASH_ADDRESS, sizeof(defSettings));
  flash.writeAnything(FLASH_ADDRESS, backupSettings);
  flash.powerDown();
}

/*
   initialize motor port (1...4) and set speed to 0
*/
void initMotor(uint32_t motorNr)
{
  if ((motorNr >= 1) && (motorNr <= 4))
  {
    motorNr--;
    if (motorNr == 1) {
      pinMode(pinMotor[motorNr][0], OUTPUT);
      digitalWrite(pinMotor[motorNr][0], LOW);
      pinMode(pinMotor[motorNr][1], OUTPUT);
      digitalWrite(pinMotor[motorNr][1], LOW);
    } else {
      pinMode(pinMotor[motorNr][0], OUTPUT);
      analogWrite(pinMotor[motorNr][0], 0);
      pinMode(pinMotor[motorNr][1], OUTPUT);
      analogWrite(pinMotor[motorNr][1], 0);
    }
  }
}

/*
   show WiFI status
*/
void printWifiStatus() {
  #ifdef SERIAL_DEBUG
  // print the SSID of the network you're attached to:
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
  jsonReply += " \", \n\"isAp\" : ";
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
  for (int i=0; i<5; i++) {
    jsonReply += sets.pinMode[i];
    if (i<4) jsonReply += ", ";
  }
  jsonReply += " ] }\n";
}

void listNetworks() {
  // scan for nearby networks:
  // Serial.println("** Scan Networks **");
  byte numSsid = WiFi.scanNetworks();

  // print the list of networks seen:
  /* Serial.print("number of available networks:");
  Serial.println(numSsid); */
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
  jsonReply += defSettings.isAp;
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
  for (int j = 4; j > 0; j--) {
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
  jsonReply = "{ \"maxvoltages\" : [ \n";
  for (int i = 0; i < 5; i++) {
    jsonReply += defSettings.maxVoltage[i];
    jsonReply += ", ";
  }
  jsonReply += "\n], \"motorspeeds\" : [ \n";
  for (int i = 0; i < 5; i++) {
    jsonReply += motorSpeed[i];
    jsonReply += ", ";
  }
  jsonReply += "\n] }\n";
}

void setSwitch(int value) {
  int rawVoltage;
  int absMax = 255;
  rawVoltage  = analogRead(PIN_A2);
  // Serial.println(rawVoltage);
  int calcVoltage = ( rawVoltage * ( 62 + 14 ) * 33 / 1023 / 14 );
  if (calcVoltage >  defSettings.maxVoltage[4]) {
    value = value * defSettings.maxVoltage[4] / calcVoltage;
    absMax = 255 * defSettings.maxVoltage[4] / calcVoltage;
  }
  if (value > absMax) {
    value = absMax;
  }
  motorSpeed[4] = value;
  if (value > absMax) motorSpeed[4] = absMax;
  digitalWrite(RS1, value);
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
  if ((motorNr >= 1) && (motorNr <= 4)) {
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
    } else {
      analogWrite(pinMotor[motorNr][0], in1);
      analogWrite(pinMotor[motorNr][1], in2);
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
  setServoPos(0, 90);
  setServoPos(1, 90); 
  jsonReply = "{ \"stopped\" : 1, \n";
  getVoltage();
  jsonReply += " }\n";
}

void setServoPos(int snum, int spos) {
        if (snum == 0) {
                servoL.write(spos);
        } else {
                servoR.write(spos);
        }
}

void setup() {
  char defAp[] = "eitech-robo"; 
  strcpy(defSettings.apSSID, defAp); 
  // defSettings.apSSID = defAp; 
  backupSettings = defSettings; 
  pinMode(DIST_TRIGGER, OUTPUT);
  pinMode(DIST_RECEIVE, INPUT);
  servoL.attach(J6);
  servoR.attach(J5);
  setServoPos(0, 90);
  setServoPos(1, 90);
  // init pins
  initMotor(1);
  initMotor(2);
  initMotor(3);
  initMotor(4);
  pinMode(RS1, OUTPUT);
  // init analog inputs
  analogReadResolution(10); // 10 bit (0...1023)
  analogReference(AR_DEFAULT); // internal reference
  
  #ifdef SERIAL_DEBUG
  // init serial port
  Serial.begin(9600);
  while (!Serial) { /* wait for serial port to connect */ } 
  #endif
    
  flash.begin();
  flash.setClock(1000000); 
  #ifdef SERIAL_DEBUG
  Serial.println(flash.getCapacity());
  #endif
  flash.readAnything(FLASH_ADDRESS, flashSettings);
  // char flashVersion[12];
  // flash.readCharArray(VERS_ADDRESS, flashVersion, 12);
  
  if ( memcmp( (const void *)flashSettings.fwVersion, (const void *)backupSettings.fwVersion, sizeof(backupSettings.fwVersion)) == 0) {
  // if ( memcmp( (const void *)flashVersion, (const void *)backupSettings.fwVersion, sizeof(backupSettings.fwVersion)) == 0) {
    #ifdef SERIAL_DEBUG
    Serial.println("Version matches!");
    #endif
    defSettings = flashSettings; 
    #ifdef SERIAL_DEBUG
    Serial.println(defSettings.infSSID);
    Serial.println(defSettings.isAp);
    #endif
  } else {
    #ifdef SERIAL_DEBUG
    Serial.println(flashSettings.fwVersion);
    Serial.println(flashSettings.infSSID);
    Serial.println(flashSettings.infPSK);
    Serial.println(flashSettings.isAp);
    Serial.println("Writing default config to flash..."); 
    #endif
    // flash.eraseSection(VERS_ADDRESS, 12);
    flash.eraseSection(FLASH_ADDRESS, sizeof(backupSettings));
    flash.writeAnything(FLASH_ADDRESS, backupSettings);
    // flash.writeCharArray(VERS_ADDRESS, backupSettings.fwVersion, 12);
    flash.readAnything(FLASH_ADDRESS, flashSettings);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_A2, INPUT);
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
  while (!startWifi())
  {
    delay(5000); // wait 5 seconds and retry
  }

  // 
  // String teststr = "Hallo Welt!";
  /* flash.eraseSection(0, 256);
    flash.writeByte(ADDR_WIFIMODE, B00000000);
    flash.writeStr(ADDR_SSID_AP, teststr);  */

  // start webserver
  // webserver.begin();

  // start MDNS responder to listen to the configured name
// #ifdef MDNS_SUPPORT
//   mdnsResponder.begin(mdnsName);
// #endif

  // you're connected now, so print out the status
  printWifiStatus();
}

void loop() {
  if (timeOut > 0) {
    if (millis() - uptime > timeOut) {
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
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
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
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
  if (connectWifi) {
    stopEverything(); 
    startWifi();
    printWifiStatus();
  }
  WiFiClient client = webserver.available();
  uptime = millis();
  if (uptime - lastAnemoRollback > 10000 && anemoTicks > 25) {
    anemoRpm = anemoTicks * 6; 
    anemoTicks = 0;
    lastAnemoRollback = uptime;
    #ifdef SERIAL_DEBUG
    Serial.println(anemoRpm); 
    #endif
  } else if (uptime - lastAnemoRollback > 30000) {
    anemoRpm = anemoTicks * 2; 
    anemoTicks = 0;
    lastAnemoRollback = uptime;
    #ifdef SERIAL_DEBUG
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
        char c = client.read();
        if (httpRequestLen < (sizeof(httpRequest) - 1)) {
          httpRequest[httpRequestLen++] = c;
          httpRequest[httpRequestLen] = '\0';
        }
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
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "wificonnect") {
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
            } else if (httpGetResource(0) == "switchnet") {
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
            } else if (httpGetResource(0) == "write") {
              writeFlash(); 
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "timeout") {
              timeOut = httpGetResource(1).toInt();
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "getdistance") { 
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
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "get") {
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
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "setmotor") {
              // Serial.println("Setting motors");
              setMotorSpeed(1, httpGetResource(1).toInt());
              setMotorSpeed(2, httpGetResource(2).toInt());
              setMotorSpeed(3, httpGetResource(3).toInt());
              setMotorSpeed(4, httpGetResource(4).toInt());
              getMaxVoltages();
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "setswitch") {
              // Serial.println("Setting motors");
              setSwitch(httpGetResource(1).toInt());
              getMaxVoltages();
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "setservo") {
              // Serial.println("Setting motors");
              setServoPos(httpGetResource(1).toInt(), httpGetResource(2).toInt());
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "getwifi") {
              // Serial.println("Getting WiFi parameters and networks");
              listNetworks();
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "setmode") {
              // modes of port: 0 output
              //                1 input
              //                2 interrupt 
              // Serial.println("Changing mode of Port");
              for (int i=0; i<5; i++) {
                defSettings.pinMode[i] = httpGetResource(i+1).toInt();
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
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "setanalog") {
              // Serial.println("Setting analogue output on J1 to J5");
              for (int i=0; i<5; i++) {
                  analogWrite(jpins[i], httpGetResource(i+1).toInt()); 
              }
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "getanalog") {
              // Serial.println("Getting analogue input on J1 to J5");
              int value = -1;
              int idx = 0;
              if (httpGetResource(1) == "j1" || httpGetResource(1) == "J1") {
                idx = 0;
              } else if (httpGetResource(1) == "j2" || httpGetResource(1) == "J2") {
                idx = 1;
              } else if (httpGetResource(1) == "j3" || httpGetResource(1) == "J3") {
                idx = 2;
              } else if (httpGetResource(1) == "j4" || httpGetResource(1) == "J4") {
                idx = 3;
              } else if (httpGetResource(1) == "j7" || httpGetResource(1) == "J7") {
                idx = 4;
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
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "setmax") {
              // Serial.println("Setting maximum voltage of motors M1 to m4");
              setMaxVoltage(httpGetResource(1), httpGetResource(2));
              client.write("HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n");
              for ( int i = 0 ; i < jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "getrpm") {
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
              client.write(outBuff);
              break;
            } else if (httpGetResource(0) == "getdht") {
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
              client.write(outBuff);
              break;
            }
            // Serial.println(httpRequestURL);
            client.write("HTTP/1.0 200 OK\nContent-type: text/plain\n\nHallo Welt!\n\n");
            break;
          }
        }
      }
    }
    client.stop();
  }
}
