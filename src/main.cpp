#include <Arduino.h>
#include <pubsubclient.h>
#include <WiFi.h>
#include <DFRobot_BloodOxygen_S.h>

#define I2C_COMMUNICATION

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x57
  DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);
#else
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
SoftwareSerial mySerial(4, 5);
DFRobot_BloodOxygen_S_SoftWareUart MAX30102(&mySerial, 9600);
#else
DFRobot_BloodOxygen_S_HardWareUart MAX30102(&Serial1, 9600); 
#endif
#endif

// WiFi settings
// TODO : Replace with your WiFi credentials here
const char *ssid = "Allumettes";
const char *password = "salutc'estmoichoupi";

// MQTT Broker settings
// TODO : Update with your MQTT broker settings here if needed
const char *mqtt_broker = "broker.emqx.io";     // EMQX broker endpoint
// const char *mqtt_topic1 = "dataCastres/topic1"; // MQTT topic
const char *mqtt_topicMAC = "homeTrainerCastres/Group1-B/MAC"; // MQTT topic
const char *mqtt_topicHeartBeatDFRobot = "homeTrainerCastres/Group1-B/HeartBeatDFRobot";
const char *mqtt_topicHeartBeatSparkfun = "homeTrainerCastres/Group1-B/HeartBeatDFRobot";
const char *mqtt_topicSPO2 = "homeTrainerCastres/Group1-B/SPO2";
const int mqtt_port = 1883;                     // MQTT port (TCP)
String client_id = "Arduino-";
String MAC_address = "";

int BPM = 0;
int beat_old = 0;
float beats[500];
int beatIndex;

// Other global variables
static unsigned long lastPublishTime = 0;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

void connectToWiFi();
void connectToMQTTBroker();
void mqttCallback(char *topic, byte *payload, unsigned int length);

void setup()
{
  Serial.begin(9600);
  connectToWiFi();
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqttCallback);
  connectToMQTTBroker();

  pinMode(10, INPUT);
  pinMode(11, INPUT);

  while (false == MAX30102.begin())
  {
    Serial.println("init fail!");
    delay(1000);
  }
  Serial.println("init success!");
  Serial.println("start measuring...");
  MAX30102.sensorStartCollect();
/*
  plot.Begin();
  plot.AddTimeGraph("ECG graph", 500, "Vital constant", graphPoint);
  */
}

void calculateBPM () 
{  
  int beat_new = millis();    // get the current millisecond
  int diff = beat_new - beat_old;    // find the time between the last two beats
  float currentBPM = 60000 / diff;    // convert to beats per minute
  beats[beatIndex] = currentBPM;  // store to array to convert the average
  float total = 0.0;
  for (int i = 0; i < 500; i++){
    total += beats[i];
  }
  BPM = int(total / 500);
  beat_old = beat_new;
  beatIndex = (beatIndex + 1) % 500;  // cycle through the array instead of using FIFO queue
}


void printMacAddress()
{
  byte mac[6];
  Serial.print("MAC Address: ");
  WiFi.macAddress(mac);
  for (int i = 0; i < 6; i++)
  {
    MAC_address += String(mac[i], HEX);
    if (i < 5)
      MAC_address += ":";
    if (mac[i] < 16)
    {
      client_id += "0";
    }
    client_id += String(mac[i], HEX);
  }
  Serial.println(MAC_address);
}

void connectToWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  delay(3000);
  printMacAddress();
  Serial.println("Connected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void connectToMQTTBroker()
{
  while (!mqtt_client.connected())
  {
    Serial.print("Connecting to MQTT Broker as ");
    Serial.print(client_id.c_str());
    Serial.println(".....");
    if (mqtt_client.connect(client_id.c_str()))
    {
      Serial.println("Connected to MQTT broker");
      // mqtt_client.subscribe(mqtt_topic1);
      mqtt_client.subscribe(mqtt_topicMAC);
      // Publish message upon successful connection

      mqtt_client.subscribe(mqtt_topicHeartBeatSparkfun);
      mqtt_client.subscribe(mqtt_topicHeartBeatDFRobot);
      mqtt_client.subscribe(mqtt_topicSPO2);

    }
    else
    {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  String messageTemp;
  for (int i = 0; i < length; i++)
  {
    // Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println(messageTemp);
  Serial.println("-----------------------");
  // TODO : Add your message handling logic here
  // For example, you can check the topic and perform actions based on the message content
  // Example:
  //
  // if (String(topic) == "arduino/output")
  // {
  //   Serial.print("Changing output to ");
  //   if (messageTemp == "on")
  //   {
  //     Serial.println("LED on");
  //     digitalWrite(ledPin, HIGH);
  //   }
  //   else if (messageTemp == "off")
  //   {
  //     Serial.println("LED off");
  //     digitalWrite(ledPin, LOW);
  //   }
  // }
}

void loop()
{
  /*
  if((digitalRead(10) == 1)||(digitalRead(11) == 1)){
    Serial.println('!');
  }
  else{
    // Serial.println(analogRead(A0));
    graphPoint = analogRead(A0);
  }
  */

  //plot.Plot();

  //delay(1);

  MAX30102.getHeartbeatSPO2();
  // delay(4000);

  if (!mqtt_client.connected())
  {
    connectToMQTTBroker();
  }
  mqtt_client.loop();
  // TODO: Add your main code here, to run repeatedly (e.g., sensor readings, publishing messages, etc. )
  // Example below : Publish a message every 10 seconds

  calculateBPM();

  unsigned long currentTime = millis();
  if (currentTime - lastPublishTime >= 5000) // 10 seconds
  {
    lastPublishTime = currentTime;

    mqtt_client.publish(mqtt_topicMAC, MAC_address.c_str());

    Serial.println(MAC_address);
    Serial.println(MAX30102._sHeartbeatSPO2.SPO2);

    mqtt_client.publish(mqtt_topicSPO2, std::to_string(MAX30102._sHeartbeatSPO2.SPO2).c_str());
    mqtt_client.publish(mqtt_topicHeartBeatDFRobot, std::to_string(MAX30102._sHeartbeatSPO2.Heartbeat).c_str());

    mqtt_client.publish(mqtt_topicHeartBeatSparkfun, std::to_string(BPM).c_str());
  }
}