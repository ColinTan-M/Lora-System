#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(0, 5);

const char *ssid = "*****";
const char *password = "*****";
const char *mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char *mqtt_id = "*****";

uint8_t uart_transmit[20];

String temp;
uint8_t check_uart;
uint8_t check;
uint8_t Auto;
uint8_t Bump;
uint8_t Fan;
uint8_t Led;
uint8_t Temp;
uint8_t Humi;
uint16_t Lux;
uint8_t Lux_H;
uint8_t Lux_L;
uint8_t HumiSoil;
char buf[10];

long lastMeasure = 0;
long pub = 0;
long Tick_Button = 0;

WiFiClient client;
PubSubClient mqtt_client(client);

void callback(char* topic, byte* payload, unsigned int length) {
  
  String topic_temp = String((char*)topic);
  String message;
  
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  if(topic_temp == "CtrlLora/Auto"){
    uart_transmit[0] = (uint8_t)(message.toInt());
    Tick_Button = millis();
  }
  if(topic_temp == "CtrlLora/Bump"){
    uart_transmit[1] = (uint8_t)(message.toInt());
    Tick_Button = millis();
  }
  if(topic_temp == "CtrlLora/Fan"){
    uart_transmit[2] = (uint8_t)(message.toInt());
    Tick_Button = millis();
  }
  if(topic_temp == "CtrlLora/Led"){
    uart_transmit[3] = (uint8_t)(message.toInt());
    Tick_Button = millis();
  }
  if(topic_temp == "CtrlLora/TempUpper"){
    uart_transmit[4] = (uint8_t)(message.toInt());
  }
  if(topic_temp == "CtrlLora/TempLower"){
    uart_transmit[5] = (uint8_t)(message.toInt());
  }
  if(topic_temp == "CtrlLora/HumiUpper"){
    uart_transmit[6] = (uint8_t)(message.toInt());
  }
  if(topic_temp == "CtrlLora/HumiLower"){
    uart_transmit[7] = (uint8_t)(message.toInt());
  }
  if(topic_temp == "CtrlLora/LuxUpper"){
    uart_transmit[8] = (uint8_t)((message.toInt()) >> 8);
    uart_transmit[9] = (uint8_t)((message.toInt()) & 0xff);
  }
  if(topic_temp == "CtrlLora/LuxLower"){
    uart_transmit[10] = (uint8_t)((message.toInt()) >> 8);
    uart_transmit[11] = (uint8_t)((message.toInt()) & 0xff);
  }
  if(topic_temp == "CtrlLora/HumiSoilUpper"){
    uart_transmit[12] = (uint8_t)(message.toInt());
  }
  if(topic_temp == "CtrlLora/HumiSoilLower"){
    uart_transmit[13] = (uint8_t)(message.toInt());
  }
   Serial.print(topic_temp); Serial.print(": "); 
   Serial.println(message);
   check_uart = 1;
}

void setup_wifi()
{
  Serial.print("Connecting to Wifi... ");
  Serial.print(ssid);
  Serial.println();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.print("Connected to Wifi ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.println();
  delay(1000);
}

void reconnect()
{
  Serial.println("Connecting to mqtt... ");
  while (!mqtt_client.connect(mqtt_id))
  {
    delay(500);
  }
  Serial.println("Connected to mqtt ");
  mqtt_client.subscribe("CtrlLora/Auto");
  mqtt_client.subscribe("CtrlLora/Bump");
  mqtt_client.subscribe("CtrlLora/Fan");
  mqtt_client.subscribe("CtrlLora/Led");
  mqtt_client.subscribe("CtrlLora/TempUpper");
  mqtt_client.subscribe("CtrlLora/TempLower");
  mqtt_client.subscribe("CtrlLora/HumiUpper");
  mqtt_client.subscribe("CtrlLora/HumiLower");
  mqtt_client.subscribe("CtrlLora/LuxUpper");
  mqtt_client.subscribe("CtrlLora/LuxLower");
  mqtt_client.subscribe("CtrlLora/HumiSoilUpper");
  mqtt_client.subscribe("CtrlLora/HumiSoilLower");
}

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  setup_wifi();
  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(callback);
  reconnect(); 
  
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    setup_wifi();
  }
  if (!mqtt_client.connect(mqtt_id))
  {
    reconnect();  
  }

  
  if (millis() - pub > 700) {
    pub = millis();
    if(mySerial.available()){//Read from  STM module and send to serial monitor
      check = mySerial.read();
      if(check == 255)
      {
        Auto = mySerial.read();
        Bump = mySerial.read();
        Fan = mySerial.read();
        Led = mySerial.read();
        Temp = mySerial.read();
        Humi = mySerial.read();
        Lux_H = mySerial.read();
        Lux_L = mySerial.read();
        HumiSoil = mySerial.read();

        if((Temp != 255 )&&(Humi < 100)&&(HumiSoil < 100)){
          if(millis() - Tick_Button > 5500){
            sprintf(buf, "%d", Auto);
            mqtt_client.publish("LORA/AUTO", buf);
            sprintf(buf, "%d", Bump);
            mqtt_client.publish("LORA/BUMP", buf);
            
            sprintf(buf, "%d", Fan);
            mqtt_client.publish("LORA/FAN", buf);
            sprintf(buf, "%d", Led);
            mqtt_client.publish("LORA/LED", buf);
          }
            
          sprintf(buf, "%d", Temp);
          mqtt_client.publish("LORA/TEMP", buf);
          sprintf(buf, "%d", Humi);
          mqtt_client.publish("LORA/HUMI", buf);
          
          Lux = (((uint16_t)Lux_H) << 8) | ((uint16_t)Lux_L);    
          sprintf(buf, "%d", Lux);
          mqtt_client.publish("LORA/LUX", buf);
          sprintf(buf, "%d", HumiSoil);
          mqtt_client.publish("LORA/HUMISOIL", buf);
          
          Serial.print("Auto: "); Serial.println(Auto);
          Serial.print("Bump: "); Serial.println(Bump);
          Serial.print("Led: "); Serial.println(Led);
          Serial.print("Fan: "); Serial.println(Fan);
          Serial.print("Temp: "); Serial.println(Temp);
          Serial.print("Humi: "); Serial.println(Humi);
          Serial.print("Lux_H: "); Serial.println(Lux_H);
          Serial.print("Lux_L: "); Serial.println(Lux_L);
          Serial.print("HumiSoil: "); Serial.println(HumiSoil);  
        }
      }
      else temp = mySerial.readString();
    }
  }
  
  if (millis() - lastMeasure > 200) {
    lastMeasure = millis();
    if(check_uart == 1)
    {
      check_uart = 0;
      uart_transmit[14] = 255;
      for (int i=0; i<=14; i++) { 
        mySerial.write(uart_transmit[i]);
      }
    }
    
  }
  
  mqtt_client.loop();
}
