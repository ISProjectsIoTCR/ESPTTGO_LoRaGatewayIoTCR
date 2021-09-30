#include <Arduino.h>
#include "Splitter.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "user-variables.h"
#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa
#include <U8g2lib.h>   // https://github.com/olikraus/U8g2_Arduino


//CONFIGURACION DE SENSORES Y ACTUADORES
struct Config                               //PINS-INPUTS (json construct setup)
{
  float sensor1;
  float sensor2;
  int ledstate;
};
Config config;

//PINS-OUTPUTS
#define OFF 0   // For LED
#define ON 1
// SPI LoRa Radio
#define LORA_SCK 5        // GPIO5 - SX1276 SCK
#define LORA_MISO 19     // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 - SX1276 MOSI
#define LORA_CS 18     // GPIO18 - SX1276 CS
#define LORA_RST 14   // GPIO14 - SX1276 RST
#define LORA_IRQ 26  // GPIO26 - SX1276 IRQ (interrupt request)
// I2C OLED Display works with SSD1306 driver
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

//Functions definitions
void sendToDashboard(const Config & config);
bool get_mqtt_credentials();
void check_mqtt_connection();
bool reconnect();
void process_sensors();
void process_actuators();
void connect_to_IoTCRv2();
void send_data_to_broker();
void callback(char *topic, byte *payload, unsigned int length);
void process_incoming_msg(String topic, String incoming);
void print_stats();
void clear();
void listeningLora();
void sendOkLoRa();


//Global Vars
WiFiClient espclient;
PubSubClient client(espclient);
Splitter splitter;
DynamicJsonDocument mqtt_data_doc(2048);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Full framebuffer, SW I2C

long lastReconnectAttemp = 0;
long varsLastSend[20];
String last_received_msg = "";
String last_received_topic = "";
long lastStats = 0;
long lastsendToDB = 0;
String rssi = "";
String packet = "";
const int blueLED = LED_BUILTIN;
bool dataLoRa = false; 

// Start Subroutines
#include <iotcrv2-conector.h>


//_________________________________SET-UP_______________________________________
void setup()
{
  Serial.begin(115200);
  while (!Serial);
  pinMode(blueLED, OUTPUT); // For LED feedback

  Serial.println("LoRa Receiver");
  Display.begin();
  Display.enableUTF8Print();    // enable UTF8 support for the Arduino print() function
  Display.setFont(u8g2_font_ncenB10_tr);

  
  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  // Very important for LoRa Radio pin configuration! 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);    

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  // The larger the spreading factor the greater the range but slower data rate
  // Send and receive radios need to be set the same
  LoRa.setSpreadingFactor(12);  // ranges from 6-12, default 7 see API docs
  LoRa.setTxPower(14, PA_OUTPUT_RFO_PIN);

  clear();
  Serial.print("\n\n\nWiFi Connection in Progress" );
  WiFi.begin(wifi_ssid, wifi_password);

  int counter = 0;

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    counter++;

    if (counter > 10)
    {
      Serial.print("  ⤵");
      Serial.print("\n\n         Ups WiFi Connection Failed :( ");
      Serial.println(" -> Restarting..." );
      delay(2000);
      ESP.restart();
    }
  }

  Serial.print("  ⤵" );
  //Printing local ip
  Serial.println( "\n\n         WiFi Connection -> SUCCESS :)" );
  Serial.print("\n         Local IP -> ");
  Serial.print(WiFi.localIP());

  get_mqtt_credentials();
  client.setCallback(callback);
}


//__________________________________LOOP________________________________________
void loop()
{
  listeningLora();
  if(dataLoRa){
    check_mqtt_connection();
    client.loop();
    sendToDashboard(config);
    print_stats();
    sendOkLoRa();
  }
}

//________________________________LISTENING_LORA ⤵________________________________
void listeningLora()
{
 // try to parse packet
  int packetSize = LoRa.parsePacket();
  Serial.println("Escuchando mensajes LoRa");
  Display.clearBuffer();
  Display.setCursor(0,12); Display.print("Escuchando");
  Display.setCursor(0,30); Display.print("la LoRa...");
  Display.sendBuffer();
  if (packetSize) 
  {
    dataLoRa = true;
    Serial.print("Llego un mensaje...'");
    digitalWrite(blueLED, ON);                                 // Turn blue LED on
    packet = "";                                               // Clear packet

    while (LoRa.available()) 
    {
      packet += (char)LoRa.read();                             // Assemble new packet
    }
    rssi = LoRa.packetRssi();

    // Display Info
    Display.clearBuffer();  
    Display.setCursor(0,12); Display.print("La LoRa habla");
    Display.setCursor(0,26); Display.print("Paquete recibido:");
    Display.setCursor(0,42); Display.print("    '" + packet + "'");
    Display.setCursor(0,58); Display.print("RSSI " + rssi);
    Display.sendBuffer();

    digitalWrite(blueLED, OFF); // Turn blue LED off
    Serial.println(packet + "' con RSSI " + rssi);     

    config.sensor1 = packet.toFloat();
  }
}


//________________________________CONFIRMAR ENVIO A NODO_LORA⤵________________________________
void sendOkLoRa()
{  
  dataLoRa = false;
  bool response = true;
  Serial.print("enviando confirmación... ");

  digitalWrite(blueLED, ON);  // Turn blue LED on
  // send packet
  LoRa.beginPacket();
  LoRa.print(response);
  LoRa.endPacket();
  digitalWrite(blueLED, OFF); // Turn blue LED off

  // Display Info
  Display.clearBuffer();
  Display.setCursor(0,12); Display.print("Confirmando");
  Display.setCursor(0,30); Display.print("el recibido:");
  Display.setCursor(0,48); Display.print(" # " + (String)response);
  Display.sendBuffer();
  delay(5000);
}


//________________________PUBLICAR EN IoTPROJECTS ⤵_____________________________
void sendToDashboard(const Config & config)
{

//*********************CADA POSICIÓN ES UN WIDGET QUE CREASTE*******************

    mqtt_data_doc["variables"][0]["last"]["value"] = config.sensor1;
                                                       //posición 1 del template
    mqtt_data_doc["variables"][1]["last"]["value"] = config.sensor1;
                                                       //posición 2 del template
    mqtt_data_doc["variables"][2]["last"]["value"] = config.sensor1;
                                                       //posición 3 del template

    send_data_to_broker();
//******************************************************************************
}


//________________________________ACTUADORES ⤵__________________________________
void process_actuators()
{
  if (mqtt_data_doc["variables"][4]["last"]["value"] == "true")
                                                       //posición 4 del template
  {
    digitalWrite(blueLED, HIGH);
    mqtt_data_doc["variables"][4]["last"]["value"] = "";
    varsLastSend[6] = 0;                               //posición 6 del template
  }
  else if (mqtt_data_doc["variables"][5]["last"]["value"] == "false")
                                                       //posición 4 del template
  {
    digitalWrite(blueLED, LOW);
    mqtt_data_doc["variables"][5]["last"]["value"] = "";
    varsLastSend[6] = 0;                               //posición 6 del template
  }
}


//_________________________________PRINTS ⤵_____________________________________
void print_stats()
{
  long now = millis();

  if (now - lastStats > 2000)
  {
    lastStats = millis();
    clear();
    Serial.print("\n");
    Serial.print( "\n╔══════════════════════════╗" );
    Serial.print( "\n║       SYSTEM STATS       ║" );
    Serial.print( "\n╚══════════════════════════╝" );
    Serial.print("\n\n");
    Serial.print("\n\n");

    Serial.print( "# \t Name \t\t Var \t\t Type  \t\t Count  \t\t Last V \n\n");
    for (int i = 0; i < mqtt_data_doc["variables"].size(); i++)
    {
      String variableFullName = mqtt_data_doc["variables"][i]["variableFullName"];
      String variable = mqtt_data_doc["variables"][i]["variable"];
      String variableType = mqtt_data_doc["variables"][i]["variableType"];
      String lastMsg = mqtt_data_doc["variables"][i]["last"];
      long counter = mqtt_data_doc["variables"][i]["counter"];

      Serial.println(String(i) + " \t " + variableFullName.substring(0,5) + " \t\t " + variable.substring(0,10) + " \t " + variableType.substring(0,5) + " \t\t " + String(counter).substring(0,10) + " \t\t " + lastMsg);
    }
    Serial.println( "\n\n Last Incomming Msg -> " + last_received_msg);
  }
}