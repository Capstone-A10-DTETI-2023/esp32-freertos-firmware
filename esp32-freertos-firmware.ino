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
#define PRSENSORSAMPLING_INT_MS 100
#define PRSENSORDATASEND_INT_MS 10000
#define WFSENSORSAMPLING_INT_MS 1000
#define WFSENSORDATASEND_INT_MS 10000
#define TMSENSORSAMPLING_INT_MS 1000
#define TMSENSORDATASEND_INT_MS 10000
#define TRSENSORSAMPLING_INT_MS 1000
#define TRSENSORDATASEND_INT_MS 10000

#if NODEID == 1
  const int SENSORID_PR = 1;
  const int SENSORID_WF = 5;
  const int SENSORID_TM = 18;
  const int SENSORID_TR = 19;
  const int ACTUATORID_PUMP = 2;
  byte mac[] = { 0xD8, 0x00, 0xDF, 0xEF, 0xFE, 0x01 }; // node 1
  String clientId = "capstone_a10_node1";
#elif NODEID == 2
  const int SENSORID_PR = 2;
  const int SENSORID_WF = 6;
  const int ACTUATORID_PUMP = 2;
  byte mac[] = { 0xD8, 0x00, 0xDF, 0xEF, 0xFE, 0x02 }; // node 2
  String clientId = "capstone_a10_node2";
#elif NODEID == 3
  const int SENSORID_PR = 3;
  const int SENSORID_WF = 7;
  const int ACTUATORID_PUMP = 3;
  byte mac[] = { 0xD8, 0x00, 0xDF, 0xEF, 0xFE, 0x03 }; // node 3
  String clientId = "capstone_a10_node3";
#elif NODEID == 4
  const int SENSORID_PR = 4;
  const int SENSORID_WF = 8;
  const int ACTUATORID_PUMP = 4;
  byte mac[] = { 0xD8, 0x00, 0xDF, 0xEF, 0xFE, 0x04 }; // node 4
  String clientId = "capstone_a10_node4";
#endif

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
#include <OneWire.h>
#include <DallasTemperature.h>
// #include <Oversampling.h>
 
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(74,125,232,128);  // numeric IP for Google (no DNS)
//char server[] = "www.google.com";    // name address for Google (using DNS)
// char server[] = "httpbin.org";    // name address for Google (using DNS)
IPAddress mqttServer(207,2,123,118);
 
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
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

// Oversampling adc(12, 16, 1);
OneWire oneWire(PIN_TEMPSENS);
DallasTemperature tempsens(&oneWire);

String receiveJSON;
bool pumpState = false;
double pumpValue;

double sumPressure = 0;
double sumWaterFlow = 0;
double sumTemperature = 0;
double sumTurbidity = 0;

const char* SubTopic = "actuatorData";
const char* PubTopic = "sensorData"; 

const uint dataCountPress = PRSENSORDATASEND_INT_MS / PRSENSORSAMPLING_INT_MS;
const uint dataCountWFlow = WFSENSORDATASEND_INT_MS / WFSENSORSAMPLING_INT_MS;
const uint dataCountTemp = TMSENSORDATASEND_INT_MS / TMSENSORSAMPLING_INT_MS;
const uint dataCountTurb = TRSENSORDATASEND_INT_MS / TRSENSORSAMPLING_INT_MS;

static SemaphoreHandle_t xMutexPress;
static SemaphoreHandle_t xSemaphorePress;
static SemaphoreHandle_t xMutexWFlow;
static SemaphoreHandle_t xSemaphoreWFlow;
static SemaphoreHandle_t xMutexTempS;
static SemaphoreHandle_t xSemaphoreTempS;
static SemaphoreHandle_t xMutexTurb;
static SemaphoreHandle_t xSemaphoreTurb;

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
    mqtt.connect(clientId, "node", "38t8b4HVHG2cfNr6");
    Serial.println("Connecting to MQTT..");
    while(!mqtt.isConnected()) {
      Serial.print(".");
    }
    Serial.println("Connected to MQTT!");
    timeClient.begin();
    timeClient.setTimeOffset(3600 * TIME_ZONE_OFFSET_HRS);
    // default 60000 => 60s. Set to once per hour
    timeClient.setUpdateInterval(SECS_IN_HR);
  
    Serial.println("Using NTP Server :" + timeClient.getPoolServerName());
  } else {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }

  pinMode(PIN_WFLWSENS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_WFLWSENS), increase, FALLING);

  tempsens.begin();

  timeClient.update();

  xSemaphorePress = xSemaphoreCreateBinary();
  xMutexPress = xSemaphoreCreateMutex();
  xSemaphoreWFlow = xSemaphoreCreateBinary();
  xMutexWFlow = xSemaphoreCreateMutex();
  xSemaphoreTempS = xSemaphoreCreateBinary();
  xMutexTempS = xSemaphoreCreateMutex();
  xSemaphoreTurb = xSemaphoreCreateBinary();
  xMutexTurb = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(Task_ReadPressureSensor,"Task_ReadPressureSensor",8192,NULL,1,NULL,0);
  xTaskCreatePinnedToCore(Task_ReadWaterflowSensor,"Task_ReadWaterflowSensor",8192,NULL,1,NULL,1);
  xTaskCreatePinnedToCore(Task_CalcAveragePressSens, "Task_CalcAveragePressSens",16384,NULL,1,NULL,0);
  xTaskCreatePinnedToCore(Task_CalcAverageWFlowSens, "Task_CalcAverageWFlowSens",16384,NULL,1,NULL,1);
  xTaskCreatePinnedToCore(Task_PIDPump, "Task_PIDPump",10240,NULL,1,NULL,0);
  xTaskCreatePinnedToCore(Task_UpdateCore,"Task_UpdateCore",16384,NULL,1,NULL,0);
  xTaskCreatePinnedToCore(Task_ReadTemperatureSensor, "Task_ReadTemperatureSensor", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_CalcAverageTempSens, "Task_CalcAverageTempSens", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_ReadTurbiditySensor, "Task_ReadTurbiditySensor", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_CalcAverageTurbSens, "Task_CalcAverageTurbSens", 8192, NULL, 1, NULL, 1);
  // vTaskStartScheduler();
}
 
void loop() {
}

ICACHE_RAM_ATTR void increase() {
  pulse++;
}

void Task_ReadPressureSensor(void *pvParams) {
  (void)pvParams;

  while(timeClient.getSeconds()%(PRSENSORDATASEND_INT_MS/1000) != 0) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  uint counter = 0;
  double pressure = 0;
  while(1){
    if( NODEID == 1 ){
      pressure = analogRead(PIN_PRESSENS) * 0.04375 - 12.925; // node 1
    } else if ( NODEID == 2 ) {
      pressure = analogRead(PIN_PRESSENS) * 0.04375 - 11.5125; //node 2
    } else if ( NODEID == 3 ) {
      pressure = analogRead(PIN_PRESSENS) * 0.056 - 13.5; // node 3
    } else if ( NODEID == 4) {
      pressure = analogRead(PIN_PRESSENS) * 0.05 - 11.2; // node 4
    }

    // Serial.println("Current Pressure Sensor Data :" + String(pressure));
    sumPressure += pressure;
    counter++;
    if(counter == dataCountPress){
      counter = 0;
      xSemaphoreGive(xSemaphorePress);
    }
    vTaskDelay( PRSENSORSAMPLING_INT_MS / portTICK_PERIOD_MS );
  }
}

void Task_ReadWaterflowSensor(void *pvParams) {
  (void)pvParams;
  double flowRate;
  unsigned int flowMilliLitres;
  unsigned long oldTime;
  uint counter = 0;

  while(timeClient.getSeconds()%(WFSENSORDATASEND_INT_MS/1000) != 0) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  while(1) {
    detachInterrupt(PIN_WFLWSENS);
    flowRate = ((1000.0 / (millis() - oldTime)) * pulse) / 4.5;
    oldTime = millis();
    flowMilliLitres = (flowRate / 60) * 1000;
    pulse = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_WFLWSENS), increase, FALLING);
    sumWaterFlow += flowRate;
    // Serial.println("Current Water Flow Sum Sensor Data :" + String(sumWaterFlow));
    counter++;
    // Serial.println("Counter Water Flow :" + String(counter));
    if(counter == dataCountWFlow) {
      counter = 0;
      xSemaphoreGive(xSemaphoreWFlow);
      vTaskDelay( 100 / portTICK_PERIOD_MS);
    }
    
    vTaskDelay( WFSENSORSAMPLING_INT_MS / portTICK_PERIOD_MS ); 
  }
}

void Task_ReadTemperatureSensor(void *pvParams) {
  (void)pvParams;
  int counter = 0;

  while(1){
    double temperatureC = tempsens.getTempCByIndex(0);
    sumTemperature += temperatureC;
    // Serial.println("Temperature : " + String(temperatureC));
    counter++;

    if(counter == dataCountTemp) {
      counter = 0;
      xSemaphoreGive(xSemaphoreTempS);
      vTaskDelay( 100 / portTICK_PERIOD_MS);
    }
    vTaskDelay( TMSENSORSAMPLING_INT_MS / portTICK_PERIOD_MS);
  }
}

void Task_ReadTurbiditySensor(void *pvParams) {
  (void)pvParams;
  int counter = 0;

  while(1){
    int turbidityData = analogRead(PIN_TURBSENS);
    double turbidityPercent = (4096-turbidityData)/4096 * 100;
    sumTurbidity += turbidityPercent;

    if(counter == dataCountTurb) {
      counter = 0;
      xSemaphoreGive(xSemaphoreTurb);
      vTaskDelay( 100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task_CalcAveragePressSens(void *pvParams) {
  (void)pvParams;
  double dataSum;
  while(1) {
    String timeDateNowLoc = String(timeClient.getYear()) + "-" + String(timeClient.getMonth()) + "-" + String(timeClient.getDay()) + " " + String(timeClient.getFormattedTime());
    if (xSemaphoreTake(xSemaphorePress, portMAX_DELAY)){
      dataSum = sumPressure;
      sumPressure = 0;
    }
    double aveData = dataSum / (double)dataCountPress;
    Serial.println("Pressure Sensor Average Data: " + String(aveData));
    StaticJsonDocument<200> docPR;

    docPR["timestamp"] = timeDateNowLoc;
    docPR["node_id"] = String(NODEID);
    docPR["sensor_id"] = String(SENSORID_PR);
    docPR["value"] = String(aveData);

    String jsonPRString;
    serializeJson(docPR, jsonPRString);
    mqtt.publish(PubTopic, jsonPRString, false, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task_CalcAverageWFlowSens(void *pvParams) {
  (void)pvParams;
  double dataSum;
  while(1){
    String timeDateNowLoc = String(timeClient.getYear()) + "-" + String(timeClient.getMonth()) + "-" + String(timeClient.getDay()) + " " + String(timeClient.getFormattedTime());
    if (xSemaphoreTake(xSemaphoreWFlow, portMAX_DELAY)){
      dataSum = sumWaterFlow;
      sumWaterFlow = 0;
    }
    Serial.println("Water Flow Sum Data :" + String(dataSum));
    double aveData = dataSum / (double)dataCountWFlow;
    Serial.println("Water Flow Sensor Average Data: " + String(aveData));
    StaticJsonDocument<200> docWF;

    docWF["timestamp"] = timeDateNowLoc;
    docWF["node_id"] = String(NODEID);
    docWF["sensor_id"] = String(SENSORID_WF);
    docWF["value"] = String(aveData);

    String jsonWFString;
    serializeJson(docWF, jsonWFString);
    mqtt.publish(PubTopic, jsonWFString, false, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task_CalcAverageTempSens(void *pvParams) {
  (void)pvParams;
  double dataSum;

  while(1){
    String timeDateNowLoc = String(timeClient.getYear()) + "-" + String(timeClient.getMonth()) + "-" + String(timeClient.getDay()) + " " + String(timeClient.getFormattedTime());
    if(xSemaphoreTake(xSemaphoreTempS, portMAX_DELAY)) {
      dataSum = sumTemperature;
      sumTemperature = 0;
    }
    double aveData = dataSum / (double)dataCountTemp;
    StaticJsonDocument<200> docTempS;

    docTempS["timestamp"] = timeDateNowLoc;
    docTempS["node_id"] = String(NODEID);
    docTempS["sensor_id"] = String(SENSORID_TM);
    docTempS["value"] = String(aveData);

    String jsonTempString;
    serializeJson(docTempS, jsonTempString);
    mqtt.publish(PubTopic, jsonTempString, false, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task_CalcAverageTurbSens(void *pvParams) {
  (void)pvParams;
  double dataSum;

  while(1){
    String timeDateNowLoc = String(timeClient.getYear()) + "-" + String(timeClient.getMonth()) + "-" + String(timeClient.getDay()) + " " + String(timeClient.getFormattedTime());
    if(xSemaphoreTake(xSemaphoreTurb, portMAX_DELAY)) {
      dataSum = sumTurbidity;
      sumTurbidity = 0;
    }
    double aveData = dataSum / (double)dataCountTurb;
    StaticJsonDocument<200> docTurbS;

    docTurbS["timestamp"] = timeDateNowLoc;
    docTurbS["node_id"] = String(NODEID);
    docTurbS["sensor_id"] = String(SENSORID_TR);
    docTurbS["value"] = String(aveData);

    String jsonTurbString;
    serializeJson(docTurbS, jsonTurbString);
    mqtt.publish(PubTopic, jsonTurbString, false, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task_UpdateCore(void *pvParams) {
  (void)pvParams;

  mqtt.subscribe(SubTopic, [](const String & payload, const size_t size)
  {
    (void) size;
    receiveJSON = payload;
    Serial.println(receiveJSON);
    StaticJsonDocument<200> docPUMP;
    DeserializationError error = deserializeJson(docPUMP, receiveJSON);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }

    String node_id = docPUMP["node_id"];
    String actuator_id = docPUMP["actuator_id"];
    String action = docPUMP["action"];
    double value = docPUMP["value"];

    if (node_id != String(NODEID) && actuator_id != String(ACTUATORID_PUMP)) {
      Serial.println("Ignoring MQTT Message because mismatch target");
    } else {
      if (action == "on" || action == "ON") {
        pumpState = true;
      } else if (action == "off" || action == "OFF") {
        pumpState = false;
      }
      pumpValue = value;
      Serial.println("New pump value: " + String(pumpValue));
    }
  });

  while(1){
    mqtt.update();
    timeClient.update();
    vTaskDelay( 500 / portTICK_PERIOD_MS ); 
  }
}

void Task_PIDPump(void *pvParams){
  (void)pvParams;
  double sensed_output, control_signal, setpoint;
  double previous_error = 0.0;
  double Kp = 20;
  double Ki = 45;
  double Kd = 1;
  double PID_error, PID_value, elapsedTime, Time, timePrev;
  double PID_p, PID_i, PID_d;

  Time = millis();

  while(1){
    // Serial.println("Current pump state :" + String(pumpState));
    while(pumpState == false) {
      Serial.println("Current pump state :" + String(pumpState));
      analogWrite(PIN_PUMPACT, 0);
      vTaskDelay(1500 / portTICK_PERIOD_MS);
    }

    setpoint = pumpValue;
    double sumsensed_output = 0;
    double sensed_output = 0;
    for(int i = 0; i<4; i++){
      sumsensed_output += analogRead(PIN_PRESSENS) * 0.04375 - 12.925;
    }
    sensed_output = sumsensed_output/4;
    Serial.print("Press_sens:" + String(sensed_output) + ",");
    Serial.print("setpoint:" + String(setpoint) + ",");
    
    PID_error = setpoint - sensed_output;
    Serial.print("PID_error:" + String(PID_error) + ",");
    PID_p = Kp * PID_error;

    if (-3 < PID_error < 3) {
      PID_i = PID_i + (Ki * PID_error);
    }

    timePrev = Time;
    Time = millis();
    elapsedTime = (Time - timePrev) / 1000;
    PID_d = Kd*((PID_error - previous_error)/elapsedTime);
    PID_value = PID_p + PID_i + PID_d;

    Serial.print("PID_val:" + String(PID_value) + ",");

    int pwm = round(PID_value);

    if ( pwm > 255 ) {
      pwm = 255;
    } else if ( pwm < 0 ) {
      pwm = 0;
    }

    Serial.print("PWM:" + String(pwm) + "\n");

    analogWrite(PIN_PUMPACT,pwm);
    previous_error = PID_error;
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}