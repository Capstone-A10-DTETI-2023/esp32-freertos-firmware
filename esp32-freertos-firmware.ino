/*
 Web client with enc28j60 and EthernetENC
 
 This sketch connects to a test website (httpbin.org)
 and try to do a GET request, the output is printed
 on Serial
 
 by Renzo Mischianti <www.mischianti.org>
 
 https://www.mischianti.org
 
 **/
 
// NODE CONFIG
#define NODEID            1
#define SENSORID_PR       1
#define SENSORID_WF       2
#define ACTUATORID_PUMP   1

// PIN NUM DEFINITION
#define PIN_WFLWSENS  14
#define PIN_PRESSENS  27
#define PIN_TEMPSENS  13
#define PIN_TURBSENS  12
#define PIN_PUMPACT   26

// NTP CONFIG
#define NTP_DBG_PORT          Serial
#define _NTP_LOGLEVEL_        0
#define TIME_ZONE_OFFSET_HRS  (+7)

// Check Running Platform
#if !( defined(ESP32) )
  #error This code is intended to run on the ESP8266 or ESP32 platform! Please check your Tools->Board setting.
#endif

#include <SPI.h>
#include <UIPEthernet.h>
#include <MQTTPubSubClient_Generic.h>
#include <NTPClient_Generic.h>
#include <ArduinoJson.h>
 
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(74,125,232,128);  // numeric IP for Google (no DNS)
//char server[] = "www.google.com";    // name address for Google (using DNS)
// char server[] = "httpbin.org";    // name address for Google (using DNS)
IPAddress mqttServer(207,2,123,118);
 
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
 
// Set the static IP address to use if the DHCP fails to assign
#define MYIPADDR 192,168,1,28
#define MYIPMASK 255,255,255,0
#define MYDNS 192,168,1,1
#define MYGW 192,168,1,1
 
// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
EthernetClient ethClient;
EthernetUDP udp;
MQTTPubSubClient mqtt;
NTPClient timeClient(udp);

unsigned long next;
volatile long pulse;
unsigned long lastTime;

bool pumpState;
double pumpValue;

const char* SubTopic = "actuatorData";
const char* PubTopic = "sensorData"; 

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Begin Ethernet");
 
    // You can use Ethernet.init(pin) to configure the CS pin
    Ethernet.init(5);
 
    if (Ethernet.begin(mac)) { // Dynamic IP setup
        Serial.println("DHCP OK!");
    }else{
        Serial.println("Failed to configure Ethernet using DHCP");
        // Check for Ethernet hardware present
        if (Ethernet.hardwareStatus() == EthernetNoHardware) {
          Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
          while (true) {
            delay(1); // do nothing, no point running without Ethernet hardware
          }
        }
        if (Ethernet.linkStatus() == LinkOFF) {
          Serial.println("Ethernet cable is not connected.");
        }
 
          IPAddress ip(MYIPADDR);
          IPAddress dns(MYDNS);
          IPAddress gw(MYGW);
          IPAddress sn(MYIPMASK);
          Ethernet.begin(mac, ip, dns, gw, sn);
          Serial.println("STATIC OK!");
    }
    delay(5000);
 
 
    Serial.print("Local IP : ");
    Serial.println(Ethernet.localIP());
    Serial.print("Subnet Mask : ");
    Serial.println(Ethernet.subnetMask());
    Serial.print("Gateway IP : ");
    Serial.println(Ethernet.gatewayIP());
    Serial.print("DNS Server : ");
    Serial.println(Ethernet.dnsServerIP());
 
   Serial.println("Ethernet Successfully Initialized");
   delay(50);
  // if you get a connection, report back via serial:
  if (ethClient.connect(mqttServer, 1883)) {
    Serial.println("Connected!");
    mqtt.begin(ethClient);
    mqtt.connect("arduino", "node", "38t8b4HVHG2cfNr6");
    Serial.println("Connecting to MQTT..");
    while(!mqtt.isConnected()) {
      Serial.print(".");
    }
    Serial.println("Connected to MQTT!");
    mqtt.subscribe(SubTopic, [](const String & payload, const size_t size)
    {
      (void) size;

      controlActuator(payload);
    });
    timeClient.begin();
    timeClient.setTimeOffset(3600 * TIME_ZONE_OFFSET_HRS);
    // default 60000 => 60s. Set to once per hour
    timeClient.setUpdateInterval(SECS_IN_HR);
  
    Serial.println("Using NTP Server " + timeClient.getPoolServerName());
  } else {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }

  pinMode(PIN_WFLWSENS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_WFLWSENS), increase, RISING);

  xTaskCreatePinnedToCore(Task_ReadPressureSensor,"Task_ReadPressureSensor",10240,NULL,1,NULL,ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(Task_ReadWaterflowSensor,"Task_ReadWaterflowSensor",10240,NULL,1,NULL,ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(Task_UpdateCore,"Task_UpdateCore",4096,NULL,1,NULL,ARDUINO_RUNNING_CORE);
  // vTaskStartScheduler();
}
 
void loop() {
}

ICACHE_RAM_ATTR void increase() {
  pulse++;
}

void Task_ReadPressureSensor(void *pvParams) {
  (void)pvParams;

  while(1){
    String timeDateNowLoc = String(timeClient.getYear()) + "-" + String(timeClient.getMonth()) + "-" + String(timeClient.getDay()) + " " + String(timeClient.getFormattedTime());
    double pressure = analogRead(PIN_PRESSENS) * 0.033488372093 - 11.3860465116;
    StaticJsonDocument<200> docPR;

    docPR["timestamp"] = timeDateNowLoc;
    docPR["node_id"] = NODEID;
    docPR["sensor_id"] = SENSORID_PR;
    docPR["value"] = pressure;

    String jsonPRString;
    serializeJson(docPR, jsonPRString);
    // mqtt.publish(pubTopic, jsonPRString, false, 0);
    Serial.println(jsonPRString);
    vTaskDelay( 2000 / portTICK_PERIOD_MS ); 
  }
}

void Task_ReadWaterflowSensor(void *pvParams) {
  (void)pvParams;

  while(1) {
    String timeDateNowLoc = String(timeClient.getYear()) + "-" + String(timeClient.getMonth()) + "-" + String(timeClient.getDay()) + " " + String(timeClient.getFormattedTime());
    double volume = 2.663 * pulse / 1000 * 30;
    if (millis() - lastTime > 2000) {
      pulse = 0;
      lastTime = millis();
    }
    StaticJsonDocument<200> docWF;

    docWF["timestamp"] = timeDateNowLoc;
    docWF["node_id"] = NODEID;
    docWF["sensor_id"] = SENSORID_WF;
    docWF["value"] = volume;

    String jsonWFString;
    serializeJson(docWF, jsonWFString);
    // mqtt.publish(pubTopic, jsonWFString, false, 0);
    Serial.println(jsonWFString);
    vTaskDelay( 2000 / portTICK_PERIOD_MS ); 
  }
}

void Task_UpdateCore(void *pvParams) {
  (void)pvParams;

  while(1){
    mqtt.update();
    timeClient.update();
    Serial.print("MQTT Connected? ");
    Serial.print(mqtt.isConnected());
    Serial.print("\n");
    vTaskDelay( 500 / portTICK_PERIOD_MS ); 
  }
}

int controlActuator(String subPayloadJSON) {
  StaticJsonDocument<200> docPUMP;
  DeserializationError error = deserializeJson(docPUMP, subPayloadJSON);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return 1;
  }

  String node_id = docPUMP["node_id"];
  String actuator_id = docPUMP["actuator_id"];
  String action = docPUMP["action"];
  double value = docPUMP["value"];

  if (node_id != String(NODEID) && actuator_id != String(ACTUATORID_PUMP)) {
    Serial.println("Ignoring MQTT Message because mismatch target");
    return 1;
  }

  pumpValue = value;
}