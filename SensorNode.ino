#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <BH1750FVI.h>
#include <Adafruit_BMP280.h>
//#include <BLEDevice.h>

const char* ssid = "Your-Network";
const char* password = "Network-Pasword";
const char* mqtt_server = "MQTT-Broker-Address";


Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);
Adafruit_BMP280 bmp;

float x; float y; float z;
char ax[10]; char ay[10]; char az[10];

float lux;
char LightLux[5];

float temperatura; float pressao; float altitude;
char temp[7]; char pressure[9]; char alt[7];

long startTime; long long currentTime; long long elapsedTime1 = 0; char elapsedTime2[100];

long timeFlag1 = -10000000;

int aux1 = 0; int aux2 = 0; int aux3 = 0; int aux4 = 0;
int dataAmount = 1;


WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// LED Pin
#define LED 2

void accelerometer(){
  
  sensors_event_t event;
  accel.getEvent(&event);

  x = event.acceleration.x;
  y = event.acceleration.y;
  z = event.acceleration.z;
}

void lightsensor(){

  lux = LightSensor.GetLightIntensity();

}

void TemperaturaPressaoAltitude(){

  temperatura = bmp.readTemperature();
  pressao = bmp.readPressure();
  altitude = bmp.readAltitude(1013.25);
  
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  //WiFi.setSleep(0); //Linha mágica

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "LEDswitch") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(LED, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(LED, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    client.connect("ESP32Client");
    
//    Serial.print("Attempting MQTT connection...");
//    // Attempt to connect
//    if (client.connect("ESP32Client")) {
//      Serial.println("connected");
//      // Subscribe
//      client.subscribe("LEDswitch");
//    } else {
//      Serial.print("failed, rc=");
//      Serial.print(client.state());
//      Serial.println(" try again in 5 seconds");
//      // Wait 5 seconds before retrying
//      delay(500);
//    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  //WiFi.mode(WIFI_OFF);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  accel.begin(); 
  accel.setRange(ADXL345_RANGE_16_G);
  /*accel.setDataRate(ADXL345_DATARATE_0_10_HZ);*/
  accel.setDataRate(ADXL345_DATARATE_3200_HZ);

  LightSensor.begin();

  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,Adafruit_BMP280::STANDBY_MS_500);

  startTime = millis();

  

}

void loop(){
  
//    if (!client.connected()) {
//      reconnect();
//    }
//    client.loop();

    if(WiFi.status() != WL_CONNECTED){
      while (WiFi.status() != WL_CONNECTED) {
        Serial.println("No loop WIFI, tentando reconectar");
        WiFi.begin(ssid, password);
        delay(1000);
      }
    }
  
    if (!client.connected()) {
      Serial.println("No loop MQTT, tentando reconectar");
      reconnect();
    }
    client.loop();
    
    
    //Serial.println("rodando");
    
    currentTime = millis();
    elapsedTime1 = currentTime - startTime;

    /*accelerometer();
    lightsensor();
    TemperaturaPressaoAltitude();*/
  
    if((currentTime - timeFlag1) >= 1000){

//      setup_wifi();
//      client.connect("ESP32Client");
//      if (!client.connected()) {
//        reconnect();
//      }

      //Serial.println("conectado e vou enviar dados");
      
      accelerometer();
      lightsensor();
      TemperaturaPressaoAltitude();
      
      timeFlag1 = currentTime;

//      dtostrf(elapsedTime1, 100, 0, elapsedTime2);
//      client.publish("tempoS", elapsedTime2);
  
      dtostrf(x, 9, 2, ax);
      dtostrf(y, 9, 2, ay);
      dtostrf(z, 9, 2, az);

      while(aux1 < dataAmount){
        client.publish("AX", ax);
        client.publish("AY", ay);
        client.publish("AZ", az);
        aux1 = aux1 + 1;
      }
      
  
      dtostrf(lux, 5, 0, LightLux);

      while(aux2 < dataAmount){
        client.publish("Lux", LightLux);
        aux2 = aux2 + 1;
      }
      
      dtostrf(temperatura, 6, 2, temp);
      dtostrf(pressao, 8, 2, pressure);
      dtostrf(altitude, 6, 2, alt);

      while(aux3 < dataAmount){
        client.publish("bmpT", temp);
        client.publish("bmpP", pressure);
        client.publish("bmpA", alt);
        aux3 = aux3 + 1;
      }
  
      dtostrf(elapsedTime1, 100, 0, elapsedTime2);
      
      while(aux4 < dataAmount){
        client.publish("tempoS", elapsedTime2);
        aux4 = aux4 + 1;
      }

      aux1 = 0;
      aux2 = 0;
      aux3 = 0;
      aux4 = 0;
      
      delay(500);
      
//      WiFi.mode(WIFI_OFF);

//      client.disconnect();
//      WiFi.disconnect();
//      WiFi.mode(WIFI_OFF);
      
    }
}
