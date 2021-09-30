#include <Arduino.h>

//CONEXION CON IoTPROJECTS
String dId = "0034";
String webhook_pass = "hNzMCc6g9I";
String webhook_endpoint = "http://3.142.89.107:3001/api/getdevicecredentials";
const char *mqtt_server= "app.iotcostarica.ml";


//CONFIGURACION DE WiFi
const char *wifi_ssid = "IoTCR";
const char *wifi_password = "Projects17$";

const long sendDBInterval = 60000;
