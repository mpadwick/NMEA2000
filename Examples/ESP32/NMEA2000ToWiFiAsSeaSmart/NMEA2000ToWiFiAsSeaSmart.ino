// Demo: NMEA2000 library. Sends NMEA2000 to WiFi in NMEA0183 and SeaSmart format.
//
// The code has been tested with ESP32.

#define USE_N2K_CAN 1 // Force mcp_can
#define N2k_SPI_CS_PIN 26 // Pin for SPI Can Select
//#define N2k_CAN_INT_PIN 25 // Use interrupt and it is connected to pin 21
#define USE_MCP_CAN_CLOCK_SET 8  // possible values 8 for 8Mhz and 16 for 16 Mhz clock
//#define CAN_250KBPS  15
#include <NMEA2000_CAN.h>
#include <Seasmart.h>
#include <N2kMessages.h>
#include <WiFi.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <memory>
#include "N2kDataToNMEA0183.h"
#include "BoardSerialNumber.h"
#include "List.h"

#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>


#include "DHT.h"
#define DHTPIN 32     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

// Define your default settings here
const char* host = "smartboat";
const char* ssid     = "Guest";
const char* password = "Guest";
const uint16_t ServerPort=10100; // Define the port, where served sends data. Use this e.g. on OpenCPN
const char *ServerIP=""; // Define the IP, what server will use. This has to be within your local network. Leave empty for DHCP

const size_t MaxClients=10;
bool SendNMEA0183Conversion=true; // Do we send NMEA2000 -> NMEA0183 consverion
bool SendSeaSmart=true; // Do we send NMEA2000 messages in SeaSmart format
bool ResetWiFiSettings=true; // If you have tested other code in your module, it may have saved settings and have difficulties to make connection.

WebServer webserver(80);

WiFiServer server(ServerPort,MaxClients);

using tWiFiClientPtr = std::shared_ptr<WiFiClient>;
LinkedList<tWiFiClientPtr> clients;

tN2kDataToNMEA0183 tN2kDataToNMEA0183(&NMEA2000, 0);

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM={130311UL, // WaterTemp
                                               0};
const unsigned long ReceiveMessages[] PROGMEM={/*126992L,*/ // System time
                                              127250L, // Heading
                                              127258L, // Magnetic variation
                                              128259UL,// Boat speed
                                              128267UL,// Depth
                                              129025UL,// Position
                                              129026L, // COG and SOG
                                              129029L, // GNSS
                                              130306L, // Wind
                                              128275UL,// Log
                                              127245UL,// Rudder
                                              129038L, // AIS Class A
                                              129039L, // AIS Class B
                                              129040L, // AIS Class B Extended
                                              129794L, // AIS Class A Static and Voyage
                                              129792L, // AIS DGNSS Broadcast
                                              129793L, // AIS UTC and Date Report
                                              129795L, // AIS Addressed Binary Message
                                              129796L, // AIS Ack
                                              129797L, // AIS Broadcast Binary Message
                                              129798L, // AIS SAR Aircraft Position
                                              129803L, // AIS Interrogation
                                              129804L, // AIS Assignment Mode Command
                                              129805L, // AIS Data Link Management Message
                                              129806L, // AIS Channel Management
                                              129810L,
                                              129811L,
                                              130310UL, // WaterTemp
                                              130311UL, // WaterTemp
                                              0};

// Forward declarations
void LedOn(unsigned long OnTime);
void UpdateLedState();
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg);
void InitNMEA2000();

#include <nvs.h>
#include <nvs_flash.h>

void ResetWiFiSettingsOnNvs() {
  int err;
  err=nvs_flash_init();
  Serial.println("nvs_flash_init: " + err);
  err=nvs_flash_erase();
  Serial.println("nvs_flash_erase: " + err); 
}

//*****************************************************************************
void setup() {
  if ( ResetWiFiSettings ) ResetWiFiSettingsOnNvs();

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  Serial.print("Connecting to ");
  Serial.println(ssid);

 // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  /*WiFi.softAP(ssid,password);
  IPAddress myIP = WiFi.softAPIP();
  */
  WiFi.begin(ssid, password);

  Serial.println(F("DHTxx test!"));
  dht.begin();


  size_t WaitCount=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    WaitCount++;
    if ( WaitCount>80 ) {
      Serial.println();
      WaitCount=0;
    }
  }

  if ( ServerIP!=0 && ServerIP[0]!=0 ) { // Try to force ip.
    IPAddress local_IP; 
    if ( local_IP.fromString(ServerIP) && !WiFi.config(local_IP,WiFi.gatewayIP(),WiFi.subnetMask()) ) { //, gateway, subnet, primaryDNS, secondaryDNS)) {
      Serial.println("STA Failed to configure");
    }
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  //Serial.println(myIP);
  
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  
  server.begin();
  delay(1000);
  InitNMEA2000();
  InitWeb();
}

//*****************************************************************************
void AddClient(WiFiClient &client) {
  Serial.println("New Client.");
  clients.push_back(tWiFiClientPtr(new WiFiClient(client)));
}

//*****************************************************************************
void StopClient(LinkedList<tWiFiClientPtr>::iterator &it) {
  Serial.println("Client Disconnected.");
  (*it)->stop();
  it=clients.erase(it);
}

//*****************************************************************************
void CheckConnections() {
 WiFiClient client = server.available();   // listen for incoming clients

  if ( client ) AddClient(client);

  for (auto it=clients.begin(); it!=clients.end(); it++) {
    if ( (*it)!=NULL ) {
      if ( !(*it)->connected() ) {
        StopClient(it);
      } else {
        if ( (*it)->available() ) {
          char c = (*it)->read();
          if ( c==0x03 ) StopClient(it); // Close connection by ctrl-c
        }
      }
    } else {
      it=clients.erase(it); // Should have been erased by StopClient
    }
  }
}

//*****************************************************************************
void loop() {
  CheckConnections();
  ReadingTempHumid();
  NMEA2000.ParseMessages();
  tN2kDataToNMEA0183.Update();
  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if ( Serial.available() ) { Serial.read(); } 
  UpdateLedState();
  webserver.handleClient();
}

// Code below is just for handling led blinking.

#define LedOnTime 2
#define LedBlinkTime 1000
unsigned long TurnLedOffTime=0;
unsigned long TurnLedOnTime=millis()+LedBlinkTime;

//*****************************************************************************
void LedOn(unsigned long OnTime) {
  digitalWrite(LED_BUILTIN, HIGH);
  TurnLedOffTime=millis()+OnTime;
  TurnLedOnTime=0;
}

//*****************************************************************************
void UpdateLedState() {
  if ( TurnLedOffTime>0 && TurnLedOffTime<millis() ) {
    digitalWrite(LED_BUILTIN, LOW);
    TurnLedOffTime=0;
    TurnLedOnTime=millis()+LedBlinkTime;
  }
  
  if ( TurnLedOnTime>0 && TurnLedOnTime<millis() ) LedOn(LedBlinkTime);
}

// Reading serial number depends of used board. BoardSerialNumber module
// has methods for RPi, Arduino DUE and Teensy. For others function returns
// 0 and then DefaultSerialNumber will be used.
#define DefaultSerialNumber 999999
//*****************************************************************************
uint32_t GetSerialNumber() {
  uint32_t Sno=GetBoardSerialNumber();

  return ( Sno!=0?Sno:DefaultSerialNumber );
}

//*****************************************************************************
void InitNMEA2000() {
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(100);
  NMEA2000.SetForwardStream(&Serial);  // PC output on due native port
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text
  NMEA2000.EnableForward(true);                 // Disable all msg forwarding to USB (=Serial)

  char SnoStr[33];
  uint32_t SerialNumber=GetSerialNumber();
  snprintf(SnoStr,32,"%lu",(long unsigned int)SerialNumber);

  NMEA2000.SetProductInformation(SnoStr, // Manufacturer's Model serial code
                                 130, // Manufacturer's product code
                                 "N2k->NMEA0183 WiFi",  // Manufacturer's Model ID
                                 "1.0.0.1 (2018-04-08)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2018-04-08)" // Manufacturer's Model version
                                 );
  // Det device information
  NMEA2000.SetDeviceInformation(SerialNumber, // Unique number. Use e.g. Serial number.
                                130, // Device function=PC Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,32);
  //NMEA2000.EnableForward(false);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.AttachMsgHandler(&tN2kDataToNMEA0183); // NMEA 2000 -> NMEA 0183 conversion
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg); // Also send all NMEA2000 messages in SeaSmart format
  
  tN2kDataToNMEA0183.SetSendNMEA0183MessageCallback(SendNMEA0183Message);

  NMEA2000.Open();
}

//*****************************************************************************
void SendBufToClients(const char *buf) {
  for (auto it=clients.begin() ;it!=clients.end(); it++) {
    if ( (*it)!=NULL && (*it)->connected() ) {
      (*it)->println(buf);
    }
  }
}

#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500 
//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  LedOn(LedOnTime);

  if ( !SendSeaSmart ) return;
  
  char buf[MAX_NMEA2000_MESSAGE_SEASMART_SIZE];
  if ( N2kToSeasmart(N2kMsg,millis(),buf,MAX_NMEA2000_MESSAGE_SEASMART_SIZE)==0 ) return;
  SendBufToClients(buf);
}

#define MAX_NMEA0183_MESSAGE_SIZE 100
//*****************************************************************************
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg) {
  if ( !SendNMEA0183Conversion ) return;
  
  char buf[MAX_NMEA0183_MESSAGE_SIZE];
  if ( !NMEA0183Msg.GetMessage(buf,MAX_NMEA0183_MESSAGE_SIZE) ) return;
  SendBufToClients(buf);
}


// Code below is just for handling Temp and Humidity Sensor.
#define TempHumidTime 1000
unsigned long TempHumidOffTime=0;
unsigned long TempHumidOnTime=millis()+TempHumidTime;
//*****************************************************************************
void ReadingTempHumid() {
  if ( TempHumidOffTime>0 && TempHumidOffTime<millis() ) {
    TempHumidOffTime=0;
    TempHumidOnTime=millis()+TempHumidTime;
  }
  
  if ( TempHumidOnTime>0 && TempHumidOnTime<millis() ) {
    tN2kMsg N2kMsg;
    
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    // Compute heat index in Fahrenheit (the default)
    float hif = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);
    /*
    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
    Serial.print(f);
    Serial.print(F("°F  Heat index: "));
    Serial.print(hic);
    Serial.print(F("°C "));
    Serial.print(hif);
    Serial.println(F("°F"));
    Serial.println(millis());
    */
    TempHumidOffTime=millis()+TempHumidTime;
    TempHumidOnTime=0;

    SetN2kEnvironmentalParameters(N2kMsg, 1, tN2kTempSource(0), t, tN2kHumiditySource(0), h, N2kDoubleNA);
    NMEA2000.SendMsg(N2kMsg);
    tN2kDataToNMEA0183.HandleMsg(N2kMsg);
  }
}

//*****************************************************************************
  
  
  // Style
  String style =
  "<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
  "input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
  "#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
  "#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
  "form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
  ".btn{background:#3498db;color:#fff;cursor:pointer}</style>";

  /*
  // Login page
  String loginIndex = 
  "<form name=loginForm>"
  "<h1>ESP32 Login</h1>"
  "<input name=userid placeholder='User ID'> "
  "<input name=pwd placeholder=Password type=Password> "
  "<input type=submit onclick=check(this.form) class=btn value=Login></form>"
  "<script>"
  "function check(form) {"
  "if(form.userid.value=='admin' && form.pwd.value=='admin')"
  "{window.open('/serverIndex')}"
  "else"
  "{alert('Error Password or Username')}"
  "}"
  "</script>" + style;
*/ 

  String serverIndex =
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<h1>Update Page</h1>"
    "<input type='file' name='update'>"
    "<input type='submit' value='Update'>"
    "</form>"  + style;
  
void InitWeb() {
  // return index page which is stored in serverIndex
  webserver.on("/", HTTP_GET, []() {
    webserver.sendHeader("Connection", "close");
    webserver.send(200, "text/html", serverIndex);
  });
  webserver.on("/serverIndex", HTTP_GET, []() {
    webserver.sendHeader("Connection", "close");
    webserver.send(200, "text/html", serverIndex);
  });
  // handling uploading firmware file
  webserver.on("/update", HTTP_POST, []() {
      webserver.sendHeader("Connection", "close");
      webserver.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    }, []() {
      HTTPUpload& upload = webserver.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.setDebugOutput(true);
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin()) { //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
      } else {
        Serial.printf("Update Failed Unexpectedly (likely broken connection): status=%d\n", upload.status);
      }
    });
  webserver.begin();
}
