/*
 Web client with enc28j60 and EthernetENC
 
 This sketch connects to a test website (httpbin.org)
 and try to do a GET request, the output is printed
 on Serial
 
 by Renzo Mischianti <www.mischianti.org>
 
 https://www.mischianti.org
 
 **/
 
#include <SPI.h>
#include <EthernetENC.h>
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
MQTTPubSubClient mqtt;
 
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Begin Ethernet");
 
    // You can use Ethernet.init(pin) to configure the CS pin
    Ethernet.init(5);   // MKR ETH Shield
 
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
  } else {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }
}
 
void loop() {
  StaticJsonDocument<200> doc;

  doc["timestamp"] = "2023-01-04 15:07:59";
  doc["node_id"] = "1";
  doc["sensor_id"] = "1";
  doc["value"] = "100.5";

  String jsonString;
  serializeJson(doc, jsonString);
  mqtt.update();
  mqtt.publish("sensorData", jsonString, false, 0);
  Serial.print("MQTT Connected? ");
  Serial.print(mqtt.isConnected());
  Serial.print("\n");
  delay(5000);
}