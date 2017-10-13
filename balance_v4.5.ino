//53, 861, 964, 965, 975, 980, 1016, 1079

//for file in `ls -A1`; do curl -F "file=@$PWD/$file" 192.168.4.1/edit; done
//$ DEBUG
//$ EEPROM
//$ MOTOR
//$ RTC
//$ WEB
//$ WIFI
//$ SIGFOX
//$ MAIN


#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <FS.h>
//#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <Wire.h>
//#include <SoftwareSerial.h>
//#include <AltSoftSerial.h>



#define debugRTCReg 0
#define debugMotor 1


const char compile_date[] = "Compile time : " __DATE__ " " __TIME__;


const char* ssid = "le_poulpe_libre";
const char* password = "56df32eac1";

IPAddress staticIP(192, 168, 0, 47);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);
const byte bssid[] = {0xde, 0xf3, 0x49, 0xaa, 0x53, 0x68};

const char* host = "balance-v4-5";


//telnet
WiFiServer telnetServer(23);
WiFiClient telnetClient;



//SoftwareSerial sigfoxSerial(0, 2, false, 256); //RX, TX
//AltSoftSerial sigfoxSerial(); //0, 2, true, 256); //RX, TX
#define sigfoxSerial Serial

#define RTCsda 4
#define RTCscl 5

byte RTCReg[20];
byte seconds, minutes, hours;


IPAddress timeServerIP; //(192,168,0,78);

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

//WiFiManager wifiManager;
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;

//holds the current upload
File fsUploadFile;



byte wakeUpPeriod; //minutes
byte sigfoxSendEnable;

#include <Ticker.h>
#include <EEPROM.h>


/*#define D5 14
  #define D6 12
  #define D7 13
  #define D8 15*/
//byte stepPinsArr[] = {D5,D6,D7,D8};
byte stepIdx, stepPin, stepIdxPrev;
//int stepPinsArr[] = {16, 14, 12, 13};
int stepPinsArr[] = {15, 14, 12, 13};
int stepHalfArr[] = {0b1000, 0b1001, 0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100};

//const int opticalLedPin = 15;
const int opticalLedPin = 16;

unsigned long lastSend;


Ticker Timer1;



//optical sensor
int v0, v1, optDiff, diffMin, diffMinBuf, diffMax, diffMaxBuf, diffPrev, diffSens, diffLowPass, diffCC;
long diffLastMin, diffLastMax;
long pesee;

//stepper motor

long motorPosition, motorLastPos, motorTarget;
int motorDirection;

bool motorOn;
#define periodSuperFast 3
#define periodFast 80
#define periodSlow 150
#define periodSuperSlow 300

//timer
long period;
unsigned long lastStep, lastOpticalLoop, lastReadReg;

//pesee
#define stepToGram 0.64
long rechercheEquilibre();

bool motorCheck;
#define motorCheckPin 3


/*=======================================================
  =================   $DEBUG  ==============================
  =========================================================*/
String debugText;
bool debugLastSameLine = true;

void debug(String text, bool sameLine = false) {
  debugLastSameLine = sameLine;
  /*if (!debugNewLine) {
    Serial.print("//");
    }
    Serial.print(text);*/

  text.replace("'", "\\'");
  text.replace("'", "\\\"");

  debugText = debugText + text;

  if (!sameLine) {
    //Serial.println("");
    debugText = debugText + "\\n";
  }

  if (debugText.length() > 1000) {
    debugText = debugText.substring(debugText.length() - 1000, debugText.length());
  }
}

void debug(int intInput, bool sameLine = false) {
  debug(String(intInput), sameLine);
}

/*=========================================================
  ======================   $EEPROM   ======================
  =========================================================*/

void StoreEEPROM() {
  for (int i = 0; i < 4; i++)
    EEPROM.write(i + 2, ((byte*)&motorPosition)[i]);

  debug("EEPROM Write : stepIdx = " + String(stepIdx));
  EEPROM.write(6, stepIdx);

  EEPROM.write(7, wakeUpPeriod);
  EEPROM.write(8, sigfoxSendEnable);

  EEPROM.commit();
}

void ReadEEPROM() {
  for (int i = 0; i < 4; i++)
    ((byte*)&motorPosition)[i] = EEPROM.read(i + 2);

  stepIdx = EEPROM.read(6);
  debug("EEPROM Read : stepIdx = " + String(stepIdx));

  wakeUpPeriod = EEPROM.read(7);
  sigfoxSendEnable = EEPROM.read(8);

  motorTarget = motorPosition;
}


/*=========================================================
  ======================   $MOTOR   ======================
  =========================================================*/

void timerIsr()
{


  if (((millis() - lastStep) >= 2000) && motorOn) {
    if (stepIdx != 0) {
        motorTarget -= stepIdx;
    } else {
      motorOn = false;
      for (int i = 0; i < 4; i++)
        digitalWrite(stepPinsArr[i], LOW);
  
      if (debugMotor) {
        debug("M0" + String(stepIdx));
        StoreEEPROM();
      }
    }
  }

  if ((millis() - lastStep) >= period) {
    motorDirection = (motorTarget == motorPosition) ? 0 : (motorTarget < motorPosition ? -1 : 1);

    if (motorDirection != 0) {
      lastStep = millis();

      if (motorOn) {
        motorPosition += motorDirection;
        stepIdx += motorDirection;
      } else {
        if (debugMotor) {
          debug("M>" + String(stepIdx));
        }
      }

      motorOn = true;

      if ((stepIdx == -1) || (stepIdx == 255)) stepIdx = 7;
      if (stepIdx == 8) stepIdx = 0;

      for (int i = 0; i < 4; i++) {
        if ((motorLastPos != motorPosition) && ((stepHalfArr[stepIdx] >> i) & 1))
          digitalWrite(stepPinsArr[i], HIGH);
        else
          digitalWrite(stepPinsArr[i], LOW);
      }


      motorLastPos = motorPosition;
    }
    
  }
}


void opticalLoop() {

  v0 = analogRead(A0);
  //v1 = analogRead(A1);

  optDiff = v0 - 550;


  if (diffSens == 1) {
    if (optDiff < diffPrev) diffLowPass ++;
    else diffLowPass = 0;

    if (optDiff > diffMaxBuf) diffMaxBuf = optDiff;

    if (diffLowPass > 2) {
      diffMinBuf = optDiff;
      diffMax = diffMaxBuf;
      diffSens = -1;
      diffLastMax = millis();
    }

  } else {
    if (optDiff > diffPrev) diffLowPass ++;
    else diffLowPass = 0;

    if (optDiff < diffMinBuf) diffMinBuf = optDiff;

    if (diffLowPass > 2) {
      diffMaxBuf = optDiff;
      diffMin = diffMinBuf;
      diffSens = 1;
      diffLastMin = millis();
    }
  }
  diffPrev = optDiff;


  if (((millis() - diffLastMin) > 4000) || ((millis() - diffLastMax) > 4000))
    diffCC = optDiff;
  else
    diffCC = (diffMax + diffMin) / 2;

}


long rechercheEquilibre() {
  bool continueLoop = true;
  unsigned long lastMotorAction = millis();
  unsigned long timeout = millis() + 60000;
  long motorDirection;
  long motorLastBalance;
  
  opticalLoop();


  if ((v0 < 650) && (v0 > 200)) {
    motorTarget -= 50;
    delay(1000);
  }


  if (v0 > 650) {
    while ((v0 > 650) && (millis() < timeout)) {
      motorTarget -= 2;
      delay(10);
      opticalLoop();
    }
  }

  delay(2000);
  opticalLoop();


  while ((v0 < 200) && (millis() < timeout)) {
    motorTarget += 2;
    delay(10);
    opticalLoop();
  }


  delay(2000);
  opticalLoop();

  while ((v0 > 650) && (millis() < timeout)) {
    motorTarget -= 2;
    delay(100);
    opticalLoop();
    if (debugMotor) {
      debug("lent 100 ", true);
      debug(motorPosition, true);
      debug(" / ", true);
      debug(v0);
    }
  }


  for (int i = 0; i < 40; i += 2) {
    motorTarget += 2;
    delay(100);
  }


  delay(1000);
  opticalLoop();

  while ((v0 < 400) && (millis() < timeout)) {
    motorTarget += 2;
    delay(1000);
    opticalLoop();
    if (debugMotor) {
      debug("lent 1000 %i", true);
      debug(motorPosition, true);
      debug(" / ", true);
      debug(v0);
    }
  }


  motorLastBalance = motorPosition;

  debug("rechercheEquilibre terminé : ", true);
  debug(motorLastBalance);

  motorStepIdx0();

  return motorLastBalance;
}

void motorStepIdx0() {
  motorTarget -= stepIdx;
  while (motorTarget != motorPosition) delay(10);
  StoreEEPROM();
}


void rechercheMotorCheck() {
  long positionResult;
  bool continueLoop = true;
  unsigned long lastMotorAction = millis();
  unsigned long timeout = millis() + 60000;
  long motorDirection;

  motorCheck = digitalRead(motorCheckPin);
  if (motorCheck) {
    motorTarget -= 200;
    delay(1000);
  }
  motorCheck = digitalRead(motorCheckPin);


  while ((!motorCheck) && (millis() < timeout)) {
    motorTarget += 2;
    delay(10);
    motorCheck = digitalRead(motorCheckPin);
  }
  
  positionResult = motorPosition;
  
  motorStepIdx0();
    
  debug("checkMotor : ", true);
  debug(positionResult);
}


/*========================================================
  =======================   $RTC    ========================
  =========================================================*/

byte bcd2int(byte in) {
  return (in & 0xF) + (10 * ((in & 0xF0) >> 4));
}

byte BCD(byte in) {
  return ((in / 10) << 4) | (in % 10);
}


void RTCReset() {
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.write(0x58);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x01);
  Wire.write(0b00000010);
  //Wire.write(0b10000000); //battery switch over standard, no battery monitor
  Wire.write(0b11100000); //no battery switch over, no battery monitor
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  /*Wire.write(0x03);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);*/
  Wire.write(0x06);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
}

void RTCConfigureTimer() {

  Wire.beginTransmission(0x68);
  Wire.write(0x0F);
  Wire.write(0b00111010); //TAM : permanent interrupt, CLKOUT Disabled,  timer A countdown, timer B disabled
  //Wire.write(0b10110010); //TAM : permanent interrupt, CLKOUT 1Hz,  timer A countdown, timer B disabled
  Wire.write(0b00000011); //TAQ : 1time.nist.go/60Hz
  //Wire.write(0b00000010); //TAQ : 1Hz
  Wire.write(wakeUpPeriod); //T_A
  //Wire.write(30); //T_A
  //Wire.write(60); //T_A
  Wire.endTransmission();
}

void RTCClearInterrupt() {
  delay(100); //for Serial to finish flush

  //WiFi.forceSleepBegin();
  //delay(10);

  Wire.beginTransmission(0x68);
  Wire.write(0x01);
  Wire.write(0b00111010); //clear CTAF
  Wire.endTransmission();

  //ESP.deepSleep(10000000, WAKE_RF_DISABLED);
}

void RTCReadReg() {
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 20);

  for (int i = 0; Wire.available() && i < 20; i++) {
    RTCReg[i] = Wire.read();

    switch (i) {
      case 3: seconds = (RTCReg[i]) & 0b01111111; break;
      case 4: minutes = (RTCReg[i]) & 0b01111111; break;
      case 5: hours = (RTCReg[i]) & 0b00111111; break;
    }
  }

  if (debugRTCReg) {
    String dbg = "RTC register : ";
    for (int i = 0; i < 20; i++) {
      if (RTCReg[i] < 0x10) dbg = dbg + "0";
      dbg = dbg + String(RTCReg[i], HEX) + " ";        // print the character
    }
    debug(dbg);
  }
}


// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address, WiFiUDP udp)
{
  debug("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void GetNTPConfigureRTC() {
  unsigned int localPort = 2390;      // local port to listen for UDP packets
  const char* ntpServerName = "0.pool.ntp.org";
  // A UDP instance to let us send and receive packets over UDP


  WiFiUDP udp;
  debug("Starting UDP");
  udp.begin(localPort);
  debug("Local port: ", true);
  debug(udp.localPort());


  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP, udp); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);

  int cb = 0;
  while (!cb) {
    sendNTPpacket(timeServerIP, udp);
    delay(1000);
    cb = udp.parsePacket();
  }

  debug("packet received, length=", true);
  debug(cb);
  // We've received a packet, read the data from it
  udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

  //the timestamp starts at byte 40 of the received packet and is four bytes,
  // or two words, long. First, esxtract the two words:

  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  debug("Seconds since Jan 1 1900 = " , true);
  debug(secsSince1900);


  int h, m, s;

  h = (secsSince1900  % 86400L) / 3600;
  m = (secsSince1900  % 3600) / 60;
  s = secsSince1900 % 60;

  debug("NTP time : ", true);
  debug(h, true);
  debug(":", true);
  debug(m, true);
  debug(":", true);
  debug(s, true);
  debug("");

  Wire.beginTransmission(0x68);
  Wire.write(0x03);
  Wire.write(BCD(s));
  Wire.write(BCD(m));
  Wire.write(BCD(h));
  Wire.endTransmission();
}



/*=============================================================
  ========================  $WEB  ===============================
  ===============================================================*/




//format bytes
String formatBytes(size_t bytes) {
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  } else {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

String getContentType(String filename) {
  if (server.hasArg("download")) return "application/octet-stream";
  else if (filename.endsWith(".htm")) return "text/html";
  else if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".gif")) return "image/gif";
  else if (filename.endsWith(".jpg")) return "image/jpeg";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".xml")) return "text/xml";
  else if (filename.endsWith(".pdf")) return "application/x-pdf";
  else if (filename.endsWith(".zip")) return "application/x-zip";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path) {
  //debug("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) {
    if (SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload() {
  if (server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    debug("handleFileUpload Name: ", true); debug(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    //debug("handleFileUpload Data: ",true); debug(upload.currentSize);
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile)
      fsUploadFile.close();
    //debug("handleFileUpload Size: ", true); debug(upload.totalSize);
  }
}

void handleFileDelete() {
  if (server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  debug("handleFileDelete: " + path);
  if (path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if (!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate() {
  if (server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  debug("handleFileCreate: " + path);
  if (path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if (SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if (file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  if (!server.hasArg("dir")) {
    server.send(500, "text/plain", "BAD ARGS");
    return;
  }

  String path = server.arg("dir");
  debug("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while (dir.next()) {
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }

  output += "]";
  server.send(200, "text/json", output);
}



void initWebServer() {
  //SERVER INIT
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  //load editor
  server.on("/edit", HTTP_GET, []() {
    if (!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
  });
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, []() {
    server.send(200, "text/plain", "");
  }, handleFileUpload);

  //called when the url is not defined here
  //use it to load content from SPIFFS
  server.onNotFound([]() {
    if (!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });

  //get heap status, analog input value and all GPIO statuses in one json call
  server.on("/all", HTTP_GET, []() {
    String json = "{";
    json += "\"millis\":" + String(millis());
    json += ",\"v0\":" + String(v0);
    json += ",\"optDiff\":" + String(optDiff);
    json += ",\"diffMin\":" + String(diffMin);
    json += ",\"diffMax\":" + String(diffMax);
    json += ",\"diffCC\":" + String(diffCC);
    json += ",\"motorPosition\":" + String(motorPosition);
    //json += ",\"motorLastBalance\":" + String(motorLastBalance);
    json += ",\"motorTarget\":" + String(motorTarget);
    json += ",\"wakeUpPeriod\":" + String(wakeUpPeriod);
    json += ",\"sigfoxSendEnable\":" + String(sigfoxSendEnable);
    json += ",\"debugText\":\"" + debugText + "\"";
    json += ",\"motorCheck\":" + String(motorCheck);
    json += "}";
    server.send(200, "text/json", json);
    json = String();
    //debug("/all : ",true);
    //debug(millis());
  });


  server.on("/motorTarget", HTTP_GET, []() {
    String motorTargetStr = server.arg(0);
    motorTarget = motorTargetStr.toInt();
    debug("motorTarget : ", true);
    debug(motorTarget);

    while (motorTarget != motorPosition) delay(10);
    StoreEEPROM();

    server.send(200, "text/plain", "OK");
  });

  server.on("/wakeUpPeriod", HTTP_GET, []() {
    String wakeUpPeriodStr = server.arg(0);
    wakeUpPeriod = wakeUpPeriodStr.toInt();
    debug("wakeUpPeriod : ", true);
    debug(wakeUpPeriod);

    StoreEEPROM();
    RTCConfigureTimer();

    server.send(200, "text/plain", "OK");
  });


  server.on("/sigfoxSendEnable", HTTP_GET, []() {
    String sigfoxSendEnableStr = server.arg(0);
    sigfoxSendEnable = sigfoxSendEnableStr.toInt();
    debug("sigfoxSendEnable : ", true);
    debug(sigfoxSendEnable);

    StoreEEPROM();

    server.send(200, "text/plain", "OK");
  });


  server.on("/sleep", HTTP_GET, []() {
    server.send(200, "text/plain", "OK");
    //ESP.deepSleep(620 * 1000000 - (millis() * 1000));
  });

  server.on("/equilibre", HTTP_GET, []() {
    server.send(200, "text/plain", "OK");
    rechercheEquilibre();
  });

  server.on("/motorCheck", HTTP_GET, []() {
    server.send(200, "text/plain", "OK");
    rechercheMotorCheck();
  });
  
  server.on("/tare", HTTP_GET, []() {
    server.send(200, "text/plain", "OK");
    motorPosition = 0;
    motorTarget = 0;
    StoreEEPROM();
  });

  server.on("/resetWifi", HTTP_GET, []() {
    server.send(200, "text/plain", "OK");
    //wifiManager.resetSettings();
  });



  server.on("/sigfoxATCommand", HTTP_GET, []() {
    String ATCommand = server.arg(0);
    
    debug("sigfoxATCommand : ", true);
    debug(ATCommand, true);


    File logfile = SPIFFS.open("/sigfoxlogmanual.txt", "a");

    sigfoxSendCommand(ATCommand, logfile);
    String ATAnswer = sigfoxGetAnswer(1000, logfile);

    logfile.close();
    debug(" >> ", true);
    debug(ATAnswer);

    server.send(200, "text/plain", ATAnswer);
  });

  server.begin();
  debug("HTTP server started");
}




/*========================================================
  ====================  $TELNET  =============================
  =========================================================*/
void telnetHandle() {
  //check if there are any new clients
  if (telnetServer.hasClient()) {

    //find free/disconnected spot
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
      debug("New telnet client");
    }

    //no free/disconnected spot so reject
    WiFiClient telnetClient2 = telnetServer.available();
    telnetClient2.stop();
  }

  //check clients for data
  if (telnetClient && telnetClient.connected()) {
    if (telnetClient.available()) {
      //get data from the telnet client and push it to the UART
      while (telnetClient.available()) sigfoxSerial.write(telnetClient.read());
    }
  }

  //check UART for data
  if (sigfoxSerial.available()) {
    size_t len = sigfoxSerial.available();
    uint8_t sbuf[len];
    sigfoxSerial.readBytes(sbuf, len);
    //push UART data to all connected telnet clients
    if (telnetClient && telnetClient.connected()) {
      telnetClient.write(sbuf, len);
      delay(1);
    }

  }
}


/*========================================================
  ====================  $WIFI  =============================
  =========================================================*/

/*void configModeCallback (WiFiManager *myWiFiManager) {
  debug("Entered config mode");
  debug(WiFi.softAPIP());

  debug(myWiFiManager->getConfigPortalSSID());
  }*/

int wifiStarted = 0;
void wifiStart() {
  unsigned long wifiStart;

  if (!wifiStarted) {
    wifiStarted = 1;

    wifiStart = millis();
    debug("Connecting");

    //Connexion au wifi
    /*wifiManager.setConnectTimeout(20);
      wifiManager.setAPCallback(configModeCallback);
      wifiManager.setDebugOutput(Serial);
      wifiManager.autoConnect();  */

    debug("Avant wifi start : " + String(WiFi.status()));

    WiFi.config(staticIP, gateway, subnet, dns);

    WiFi.begin("le_poulpe_libre", "56df32eac1", 1, bssid);


    while ((WiFi.status() != WL_CONNECTED) && ((millis() - wifiStart) < 20000))
    {
      delay(100);
      debug(String(WiFi.status()), true);
    }
    debug("");

    if (WiFi.status() != WL_CONNECTED) {
      debug("Wifi connection failed.... " + String(WiFi.status()));
      //RTCClearInterrupt(); // => switch OFF

    }

    debug("Fin wifi start : " + String(WiFi.status()));

    File logfile = SPIFFS.open("/wifilog.txt", "a");
    logfile.println("");
    logfile.println(String(bcd2int(hours)) + String(bcd2int(minutes)) + String(bcd2int(seconds)));
    logfile.println("Fin wifi start : " + String(WiFi.status()));
    logfile.close();


    unsigned long wifiConnected;
    wifiConnected = millis();

    debug(wifiConnected );

  }
}

void wifiStop() {
  if (wifiStarted == 1) {
    // Insert whatever code here to turn off all your web-servers and clients and whatnot
    WiFi.disconnect();
    WiFi.forceSleepBegin();
    delay(1); //For some reason the modem won't go to sleep unless you do a delay(non-zero-number) -- no delay, no sleep and delay(0), no sleep
  }

}

/*=======================================================
  =================   $SIGFOX   ===========================
  =========================================================*/
void sigfoxSendCommand(String command, File logfile) {
  logfile.print(millis());
  logfile.print(" >> ");
  logfile.println(command);



  sigfoxSerial.flush();
  sigfoxSerial.println(command);
  
}

String sigfoxGetAnswer(int timeout, File logfile) {
  char cr = '\r';
  char lf = '\n';
  String ret;
  unsigned long timeoutAbs;
  timeoutAbs = millis() + timeout;
  sigfoxSerial.setTimeout(timeout);


  //ret = sigfoxSerial.readStringUntil(lf);
  ret = sigfoxSerial.readStringUntil(cr);
  sigfoxSerial.readStringUntil(lf);


  logfile.print(millis());
  
  logfile.print(" overflow ");
  //logfile.print(sigfoxSerial.overflow());
  
  logfile.print(" << ");
  logfile.print(ret.length());
  logfile.print(" : ");
  logfile.println(ret);

  /*Serial.readStringUntil(lf);
    Serial.print("//");
    Serial.print(millis());
    Serial.print(" : ");
    Serial.print(ret.length());
    Serial.print(" ");
    Serial.println(ret);*/

  return ret;
}


String sigfoxSend() {
  String result = "";

  if (sigfoxSendEnable == 1) {
    RTCReadReg();
    File logfile = SPIFFS.open("/sigfoxlog.txt", "a");

    logfile.println("");
    logfile.println(String(bcd2int(hours)) + String(bcd2int(minutes)) + String(bcd2int(seconds)));

    debug("Send sigfox.....");
    while (sigfoxSerial.available()) sigfoxSerial.read();
    String ret;

    for (int i = 0; i < 5 && ret != "OK"; i++) {
      sigfoxSendCommand("AT", logfile);
      ret = sigfoxGetAnswer(1000, logfile);
    }

    if (ret == "OK") {
      
      /*
      sigfoxSendCommand("ATE0", logfile);
      sigfoxGetAnswer(1000, logfile);
      sigfoxSendCommand("ATV1", logfile);
      sigfoxGetAnswer(1000, logfile);
      sigfoxSendCommand("ATQ0", logfile);
      sigfoxGetAnswer(1000, logfile);
      */


      /*sigfoxSendCommand("ATI26", logfile);
      String tempStr = sigfoxGetAnswer(1000, logfile);
      int temp = tempStr.toInt();*/
      
      sigfoxSendCommand("AT$T?", logfile);
      String tempStr = sigfoxGetAnswer(1000, logfile);
      float temp = tempStr.toInt() / 10.0;
      
      
      debug("temp = " + String(temp));
      sigfoxGetAnswer(1000, logfile);


      String msgPesee = "0000" + String((unsigned long)pesee, HEX);
      String msgTemp = "00" + String((unsigned byte)temp, HEX);
      String msg = msgPesee.substring(msgPesee.length() - 4, msgPesee.length()) + msgTemp.substring(msgTemp.length() - 2, msgTemp.length());
      
      //sigfoxSendCommand("AT$SS=" + msg, logfile);
      sigfoxSendCommand("AT$SF=" + msg, logfile);
      result = sigfoxGetAnswer(10000, logfile);

      /*sigfoxSendCommand("ATI28", logfile);
      result = result + "_ATI28_" + sigfoxGetAnswer(1000, logfile);*/
      sigfoxSendCommand("AT$V?", logfile);
      result = result + "_AT$V?_" + sigfoxGetAnswer(1000, logfile);

      sigfoxGetAnswer(100, logfile);
      sigfoxGetAnswer(100, logfile);
      sigfoxGetAnswer(100, logfile);
      sigfoxGetAnswer(100, logfile);
    }

    logfile.close();
  }
  return result;
}






/*=======================================================
  =================   $MAIN  ==============================
  =========================================================*/
void setup(void) {

  // motor
  for (int i = 0; i < 4; i++) {
    pinMode(stepPinsArr[i], OUTPUT);
    digitalWrite(stepPinsArr[i], LOW);
  }
  WiFi.disconnect(); //necessary, without static IP connects, but wifi.status() returns always 6 even if connected.


  //delay(2000);
  sigfoxSerial.begin(9600);
  pinMode(0, INPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2,HIGH);

  
  debug("hello");
  debug(compile_date);
  //Serial.setDebugOutput(true);

  //wifiStart();



  motorOn = false;


  //optical sensor
  pinMode(opticalLedPin, OUTPUT); //power supply
  digitalWrite(opticalLedPin, HIGH);

  pinMode(A0, INPUT);
  
  //pinMode(motorCheckPin,INPUT);


  EEPROM.begin(9);
  ReadEEPROM();

  //motorPosition = 0;

  Timer1.attach_ms(2, timerIsr);
  lastStep = millis();
  period = periodSuperFast;


  bool spiffsBegin;

  spiffsBegin = SPIFFS.begin();

  /*if (!spiffsBegin) {
    SPIFFS.format();
    spiffsBegin = SPIFFS.begin();
  }*/


  {
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      debug("FS File: " + String(fileName.c_str()) + ", size: " + String(formatBytes(fileSize).c_str()));
    }
    debug("");
  }

  /*===============//Se fait on réveiller par l'horloge?===========*/

  Wire.setClock(100);
  Wire.begin(RTCsda, RTCscl);

  RTCReadReg();

  //Configuration NTP
  if (((RTCReg[0x0F]) & 2) == 0) {
    //debug("Configure NTP");
    //wifiStart();
    if (WiFi.status() == WL_CONNECTED) {
      //GetNTPConfigureRTC();
    }

    debug("Reset");
    RTCReset();

    debug("Configure timer");
    RTCConfigureTimer();
  }


  if (RTCReg[1] & 0b01000000) {
    debug("timer interrupt");

    pesee = rechercheEquilibre();
    debug("fin équilibre : ", true);
    debug(pesee);
    motorTarget = 0;
    while (motorTarget != motorPosition) delay(10);
    StoreEEPROM();
    
    //pesee = millis();


    wifiStart();



    HTTPClient http;
    int ret;
    int retryCount, okCnt;
    if (WiFi.status() == WL_CONNECTED) {
      retryCount = 0;
      okCnt = 0;
      while (okCnt < 1 && retryCount < 10) {
        RTCReadReg();
        String timePcf = String(bcd2int(hours)) + ":" + String(bcd2int(minutes)) + ":" + String(bcd2int(seconds));

        http.begin("www.pierrebeck.fr", 80, "/dumbpost.php?text=Balance_45_" + timePcf + "_Pesee_" + String(pesee)); //HTTP
        ret = http.GET();
        http.end();
        debug(ret);

        if (ret == 200) okCnt++;
        retryCount++;
      }


      File logfile = SPIFFS.open("/httplog.txt", "a");
      logfile.println("");
      logfile.println(String(bcd2int(hours)) + String(bcd2int(minutes)) + String(bcd2int(seconds)));
      logfile.println("HTTP post : " + String(ret));
      logfile.close();
      //wifiStop();
    }

    while (millis() < 2000) {
      delay(100);
    }

    String sigfoxResult = sigfoxSend();


    /*if (WiFi.status() == WL_CONNECTED) {
      retryCount = 0;
      okCnt = 0;
      while (okCnt < 1 && retryCount < 10) {
        RTCReadReg();
        String timePcf = String(bcd2int(hours)) + ":" + String(bcd2int(minutes)) + ":" + String(bcd2int(seconds));

        http.begin("www.pierrebeck.fr", 80, "/dumbpost.php?text=Balance_45_" + timePcf + "_Sigfoxreturn_" + sigfoxResult); //HTTP
        ret = http.GET();
        http.end();
        debug(ret);

        if (ret == 200) okCnt++;
        retryCount++;
      }
      wifiStop();
    }*/



    debug("Switch off....");


    RTCClearInterrupt(); // => switch OFF
  }


  /*====================================================================*/
  //wifiStart();
  //Serial.println("start soft wifi");


  char ssid[100];
  sprintf(ssid, "balance_%d", ESP.getChipId());
  
  WiFi.softAP(ssid, "apinet00");
  
  IPAddress myIP = WiFi.softAPIP();

  
  //Serial.println("AP IP address: ");
  //Serial.println(myIP);
  
  debug("AP IP address: ");
  debug(myIP);



  // Port defaults to 8266
  ArduinoOTA.setPort(8266);
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(host);
  // No authentication by default
  ArduinoOTA.setPassword((const char *)"123");


  ArduinoOTA.begin();

  //telnet
  telnetServer.begin();
  telnetServer.setNoDelay(true);


  httpUpdater.setup(&server);



  MDNS.begin(host);
  debug("Open http://", true);
  debug(host, true);
  debug(".local/edit to see the file browser");


  initWebServer();
}


unsigned long bitBangSend(char c) {
    unsigned long t = micros() + 104;
    
    digitalWrite(2,LOW);
    while (micros() < t);
    t += 104;

    for (int i = 0; i < 8; i++) {     
      digitalWrite(2,(c & (1 << i)) >> i);
      while (micros() < t);
      t += 104;
    }

    digitalWrite(2,HIGH);
    while (micros() < t);
    t += 104;

    return micros();

/*
    if (c == '\n') {
      unsigned long t2,t3, timeout;
      char cRet = 0;
      
      t2 = micros();
      timeout = millis() + 500;
    
      while ((digitalRead(0) == HIGH) && (millis() < timeout));

      
      t3 = micros();


      t = micros() + 156;
      while (micros() < t);
      
      for (int i = 0; i < 8; i++) {     
        if (digitalRead(0))
          cRet += 1 << i;
        
        t += 104;
        while (micros() < t);
      }
  


      Serial.print("delai RX ");
      Serial.println(t3 - t2);
      Serial.print((unsigned byte)cRet);
      Serial.print(" : ");
      Serial.println(cRet);
      

      

      
    }*/
}


char bitBangRead() {
    unsigned long t, timeout;
    char c = 0;


    timeout = millis() + 500;
    
    while ((digitalRead(0) == HIGH) && (millis() < timeout));
    
    t = micros() + 156;
    while (micros() < t);
    
    for (int i = 0; i < 8; i++) {     
      if (digitalRead(0))
        c += 1 << i;
      
      t += 104;
      while (micros() < t);
    }

    return c;
}


void loop(void) {
  ArduinoOTA.handle();

  server.handleClient();

  telnetHandle();

  if ((millis() - lastOpticalLoop) > 50) {
    lastOpticalLoop = millis();
    opticalLoop();
    //debug(v0);
  }


  if ((millis() - lastReadReg) > 1000) {
    lastReadReg = millis();
    /*
    bitBangSend('A');
    bitBangSend('T');
    unsigned long ttx = bitBangSend('\n');


    
    char c1 = bitBangRead();
    char c2 = bitBangRead();
    char c3 = bitBangRead();
    
    Serial.print(c1);
    Serial.print(c2);
    Serial.print(c3);*/
    
    
    RTCReadReg();
    if (RTCReg[1] & 0b01000000) {
      debug("timer interrupt");
      RTCClearInterrupt(); // => switch OFF
    }

    //debug("Wifi status : " + String(WiFi.status()));
    //debug(ESP.getFreeHeap());
  }
}
