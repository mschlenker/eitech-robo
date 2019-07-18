#include <WiFi101.h>
#include <WiFiMDNSResponder.h>
#include<SPIMemory.h>

// Adresses for configuration: 
#define ADDR_WIFIMODE   1 // bitmask for wifi modes, currently just lowest byte used to select between AP (0) and infrastructure (1)
#define ADDR_SSID_AP    1 // SSID used in access point mode
#define ADDR_SSID_IF   34 // SSID used when run in infrastructure mode
#define ADDR_WPA_AP    67 // Password used in accesspoint mode
#define ADDR_WPA_IF   131 // Password used in infrastructure mode
 
#define WAIT_FOR_SERIAL_CONNECTION
#define IS_ACCESS_POINT 0 // 0 or 1

#define DEFSSID "pinguinbaendiger"
#define DEFPASSWORD "883"

#define OUTPUT_BUFFER 4096

bool buttonPressed = false;
int pressCount = 0;

// SPIFlash flash(24);
WiFiServer webserver(80); // webserver at port 80
char *httpRequestURL; // HTTP request from client
String jsonReply; // Reply data, usually JSON
char outBuff[OUTPUT_BUFFER];

bool isAP = false;
char *ssid = DEFSSID;
char *password = DEFPASSWORD;
// maximum voltage for M1-M4 and S1
int maxVoltage[5] = { 0, 0, 0, 0, 0 }; 
const uint8_t pinMotor[4][2] = { { RM1A, RM1B }, { RM2A, RM2B }, { RM3A, RM3B }, { RM4A, RM4B } };


/*
 * start WiFi connection
 * ssid:     network SSID
 * password: network password
 * isAP:     create AP: true or false
 */
bool startWifi() {
  int status;
  // read config to determine whether running an access point or connecting to an existing network
  // bool isAP = false;
  // char *ssid = DEFSSID;
  // char *password = DEFPASSWORD;
  // WiFi.hostname(mdnsName); //use MDNS name as host name (DHCP option 12)
  
  if(isAP) {
    // create AP
    Serial.print("Create an open Access Point: ");
    Serial.println(ssid);
    status = WiFi.beginAP(ssid);
    if(status != WL_AP_LISTENING) {
      Serial.println("Creating access point failed");
      return false; // return error
    }
  } else {
    // connect to WPA/WPA2 network
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, password);
    if(status != WL_CONNECTED) {
      Serial.println("Connecting failed");
      return false; // return error
    }
  }
  return true;
}

/*
 * initialize motor port (1...4) and set speed to 0
 */
void initMotor(uint32_t motorNr)
{
  if((motorNr >= 1) && (motorNr <= 4))
  {
    motorNr--;
    pinMode(pinMotor[motorNr][0], OUTPUT);
    analogWrite(pinMotor[motorNr][0], 0);
    pinMode(pinMotor[motorNr][1], OUTPUT);
    analogWrite(pinMotor[motorNr][1], 0);
  }
}

/*
 * show WiFI status
 */
void printWifiStatus() 
{
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
}

void getVoltage() {
  int value = analogRead(PIN_A2);  // A2 is connected to the power supply (VIN)
  jsonReply += " \"vbat\" : ";
  jsonReply += (value * 3.3 * (62 + 14) / (14 * 1024)); // implicit conversion to string
}

void listNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks **");
  byte numSsid = WiFi.scanNetworks();

  // print the list of networks seen:
  Serial.print("number of available networks:");
  Serial.println(numSsid);
  jsonReply = "{ \"networks\" : [ ";
  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet<numSsid; thisNet++) {
    Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.print("\tEncryption: ");
    Serial.println(WiFi.encryptionType(thisNet));
    jsonReply += " { \"ssid\" : \"" ;
    jsonReply += WiFi.SSID(thisNet);
    jsonReply += "\" , \n \"signal\" : "; 
    jsonReply += WiFi.RSSI(thisNet);
    jsonReply += " , \n \"enctype\" : ";
    jsonReply += WiFi.encryptionType(thisNet);
    jsonReply += " },\n ";  
  }
  jsonReply += " ] ,\n \"isap\" : ";
  jsonReply += isAP; 
  jsonReply += ", \"ssid\" : \"";
  jsonReply += WiFi.SSID();
  byte mac[6];
  char str[2]; 
  jsonReply += "\", \"mac\" : \"";
  WiFi.macAddress(mac);
  for (int j=5; j>0; j--) {
    utoa((unsigned int) mac[j],str,16);
    jsonReply += str; 
    jsonReply += ":";
  }
  utoa((unsigned int) mac[0],str,16);
  jsonReply += str; 
  jsonReply += "\", \"signal\" : ";
  jsonReply += WiFi.RSSI();
  jsonReply += ", \"addr\" : \"";
  IPAddress ip = WiFi.localIP();
  jsonReply += ip;
  jsonReply += "\", \n";
  getVoltage();
  jsonReply += " }\n";
  
  // for( int i=0 ; i<jsonReply.length() ; i++) {
  //   Serial.print(jsonReply.charAt(i));
  // }
}


bool setMaxVoltage(String outport, String volt) {
  bool retcode = false;
  if (outport == "m1" || outport == "M1") {
    maxVoltage[0] = volt.toInt();
    retcode = true;
  } else if (outport == "m2" || outport == "M2") {
    maxVoltage[1] = volt.toInt();
    retcode = true; 
  } else if (outport == "m3" || outport == "M3") {
    maxVoltage[2] = volt.toInt();
    retcode = true; 
  } else if (outport == "m4" || outport == "M4") {
    maxVoltage[3] = volt.toInt();
    retcode = true; 
  } else if (outport == "s1" || outport == "S1") {
    maxVoltage[4] = volt.toInt();
    retcode = true; 
  }
  getMaxVoltages();
  return retcode;
}

void getMaxVoltages() {
  jsonReply = "{ \"maxvoltages\" : [ \n";
  for (int i=0; i<5; i++) {
    jsonReply += maxVoltage[i];
    jsonReply += ", ";
  } 
  jsonReply += "\n] }\n"; 
}

void setSwitch(int value) {
  int rawVoltage;
  int absMax;
  rawVoltage  = analogRead(PIN_A2);
  Serial.println(rawVoltage); 
  int calcVoltage = ( rawVoltage * ( 62 + 14 ) * 33 / 1023 / 14 );
  if (calcVoltage >  maxVoltage[4]) {
    value = value * maxVoltage[4] / calcVoltage; 
    absMax = 255 * maxVoltage[4] / calcVoltage; 
  }
  if (value > absMax) { value = absMax; }
  digitalWrite(RS1, value);   
}

/*
 * set motor speed
 * motorNr: 1..4
 * value:   -255...255
 * breaks:  motor break activated, when true
 */
void setMotorSpeed(int motorNr, int value, bool brakes = false)
{
  int in1, in2;
  int rawVoltage;
  if((motorNr >= 1) && (motorNr <= 4)) {
    motorNr--;
    int absMax = 255;
    if (maxVoltage[motorNr] > 0) {
      rawVoltage  = analogRead(PIN_A2);
      Serial.println(rawVoltage); 
      int calcVoltage = ( rawVoltage * ( 62 + 14 ) * 33 / 1023 / 14 );
      if (calcVoltage >  maxVoltage[motorNr]) {
        value = value * maxVoltage[motorNr] / calcVoltage; 
        absMax = 255 * maxVoltage[motorNr] / calcVoltage; 
      }
    }
    if(value >= 0)
    {
      if(value > absMax)
        value = absMax;
  
      if(brakes)
      {
        in1 = absMax;
        in2 = absMax - value;
      }
      else
      {
        in1 = value;
        in2 = 0;
      }
    }
    else  // value is negative, reverse motor
    {
      value = -value;
      if(value > absMax)
        value = absMax;

      if(brakes)
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

    analogWrite(pinMotor[motorNr][0], in1);
    analogWrite(pinMotor[motorNr][1], in2);
  }
}

/*
 * split HTTP request string
 */
bool httpSplitRequest(char *httpRequest, size_t len) {
  char *reqpart[2];
  if(len >= 3) {
    reqpart[0] = strtok(httpRequest, " \t\n");
    if(strncmp(reqpart[0], "GET\0", 4) == 0) {
      httpRequestURL = strtok(NULL, " \t"); // url
      //if(!strncmp(httpRequestURL, "/\0", 2)) // rename empty url to index.html
      //  httpRequestURL = "/index.html";
      reqpart[1] = strtok (NULL, " \t\n");
      if(!strncmp(reqpart[1], "HTTP/1.0", 8) || \
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
 * get HTTP resource string
 */
String httpGetResource(int nr) {
  String resource = "";
  char *pnt = httpRequestURL;
  if(*pnt == '/') {
    while(*pnt) {
      if(*pnt++ == '/') {
        if(nr-- <= 0) {
          while(*pnt != '\0' && *pnt != '/') {
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
  for (int i=1; i<5; i++) {
    setMotorSpeed(i, 0); 
  }
  jsonReply = "{ \"stopped\" : 1, \n";
  getVoltage();
  jsonReply += " }\n"; 
}

void setup() 
{
  // init pins
  initMotor(1);
  initMotor(2);
  initMotor(3);
  initMotor(4);
  pinMode(RS1, OUTPUT);
  // init analog inputs
  analogReadResolution(10); // 10 bit (0...1023)
  analogReference(AR_DEFAULT); // internal reference
  // flash.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_A2, INPUT);   

  // init serial port
  Serial.begin(9600);
  #ifdef WAIT_FOR_SERIAL_CONNECTION  
    while(!Serial)
    {
      // wait for serial port to connect, needed for native USB port only
    }
  #endif

  
 

  // wait for WiFi connection   
  while(!startWifi())
  {
    delay(5000); // wait 5 seconds and retry
  } 

  // Serial.println(flash.getCapacity()); 
  String teststr = "Hallo Welt!";
  /* flash.eraseSection(0, 256); 
  flash.writeByte(ADDR_WIFIMODE, B00000000); 
  flash.writeStr(ADDR_SSID_AP, teststr);  */
  
  // start webserver
  webserver.begin();

  // start MDNS responder to listen to the configured name
  #ifdef MDNS_SUPPORT
    mdnsResponder.begin(mdnsName);
  #endif

  // you're connected now, so print out the status
  printWifiStatus();
}

void loop() {
  if (analogRead(PIN_A2) > 19) {
    pressCount = 0; 
  }
  if (analogRead(PIN_A2) < 20) {
    // six seconds to reset to AP mode
    pressCount += 1;
    if (150 > pressCount > 12) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(20);
      digitalWrite(LED_BUILTIN, LOW);
      delay(80);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(300);
    }
  }
  if (pressCount > 150) {
    Serial.println("Resetting everything!"); 
    // flash.eraseSection(0, 195);
  } else if (pressCount == 12) {
    Serial.println("Changing network mode!");
    // flash.eraseSection(0, 1);
    // flash.writeByte(ADDR_WIFIMODE, B00000000); 
    // String teststr = "Hallo Welt!";
    // flash.writeStr(ADDR_SSID_AP, teststr); 
  }
  // Hard reset required after 
  while (pressCount > 150) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  /* String outputString = "";
  if (flash.readStr(ADDR_SSID_AP, outputString)) {
    Serial.println(outputString); 
  } */ 
  // delay(100); 
  WiFiClient client = webserver.available();
  if(client) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;    
    char httpRequest[1024];
    size_t httpRequestLen = 0;

    while(client.connected()) {
      if(client.available()) {
        char c = client.read();
        if(httpRequestLen < (sizeof(httpRequest)-1)) {
          httpRequest[httpRequestLen++] = c;
          httpRequest[httpRequestLen] = '\0';
        }
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if(c == '\n' && currentLineIsBlank) {
          for ( int i=0; i<OUTPUT_BUFFER; i++) { outBuff[i] = '\0'; }
          if(httpSplitRequest(httpRequest, httpRequestLen)) {
            if(httpGetResource(0) == "stop") {
              stopEverything();
              client.write("HTTP/1.0 200 OK\nContent-type: application/json\n\n");
              for( int i=0 ; i<jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff); 
              break;
            } else if (httpGetResource(0) == "get") {
              // Serial.println("Get current state");
              jsonReply = "{ ";
              getVoltage();
              jsonReply += " }\n";
              client.write("HTTP/1.0 200 OK\nContent-type: application/json\n\n");
              for( int i=0 ; i<jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff); 
              break;
            } else if (httpGetResource(0) == "setmotor") {
              Serial.println("Setting motors");
              setMotorSpeed(1, 128); 
            } else if (httpGetResource(0) == "setwifi") {
              Serial.println("Setting WiFi parameters");
            } else if (httpGetResource(0) == "getwifi") {
              Serial.println("Getting WiFi parameters and networks");
              listNetworks();
              client.write("HTTP/1.0 200 OK\nContent-type: application/json\n\n");
              for( int i=0 ; i<jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff); 
              break;
            } else if (httpGetResource(0) == "setmode") {
              Serial.println("Changing mode of Port");
            } else if (httpGetResource(0) == "setservo") {
              Serial.println("Setting servos on J1 to J4");
            } else if (httpGetResource(0) == "setanalogue") {
              Serial.println("Setting analogue output on J1 to J7");
            } else if (httpGetResource(0) == "getanalogue") {
              Serial.println("Getting analogue input on J1 to J7");
            } else if (httpGetResource(0) == "setmax") {
              // Serial.println("Setting maximum voltage of motors M1 to m4");
              setMaxVoltage(httpGetResource(1), httpGetResource(2));
              client.write("HTTP/1.0 200 OK\nContent-type: application/json\n\n");
              for( int i=0 ; i<jsonReply.length() ; i++) {
                outBuff[i] = jsonReply.charAt(i);
                // client.write(jsonReply.charAt(i));
              }
              client.write(outBuff); 
              break;
            } else if (httpGetResource(0) == "getdistance") {
              Serial.println("Get the distance of HC-SR04");
            } 
            Serial.println(httpRequestURL);
            client.write("HTTP/1.0 200 OK\nContent-type: text/plain\n\nHallo Welt!\n\n");
            break;
          } 
        }
      }
    }
    client.stop();
  }
}
