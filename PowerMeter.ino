#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

float busvoltage = 0;  float current_mA = 0; float power_mW = 0;
char BusV[9]; char Current[9]; char Power[9];
float vMax = 0; float vMin = 20; float iMax = 0; float iMin = 999; float pMax = 0; float pMin = 9999;
char vMaxS[9]; char vMinS[9]; char iMaxS[9]; char iMinS[9]; char pMaxS[9]; char pMinS[9];

long startTime; long long currentTime; long long elapsedTime1 = 0; char elapsedTime2[100];

long long currentTimeAnt; int flag1 = 0; float consumoAcumuladoA = 0; float consumoAcumuladoP = 0; float deltaT = 0; 
float auxConsumoA = 0; float auxConsumoP = 0; char consumoTotalA[25]; char consumoTotalP[25];

long timeFlag1 = -10000000; long timeFlag2 = -10000000;

float vAcumulado = 0; float iAcumulado = 0; float pAcumulado = 0; float contador = 0;
float vMedia = 0; float iMedia = 0; float pMedia = 0;



// Update these with values suitable for your network.

const char* ssid = "Your-Network";
const char* password = "Network-Pasword";
const char* mqtt_server = "MQTT-Broker-Address";


WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void battery(){
  
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  
}

void batteryMaxMinCheck(){
  
  if(busvoltage > vMax) {vMax = busvoltage; dtostrf(vMax, 8, 2, vMaxS); client.publish("BatVmax", vMaxS);}
  if(current_mA > iMax) {iMax = current_mA; dtostrf(iMax, 8, 2, iMaxS); client.publish("BatImax", iMaxS);}
  if(power_mW > pMax)   {pMax = power_mW;   dtostrf(pMax, 8, 2, pMaxS); client.publish("BatPmax", pMaxS);}
  if(busvoltage < vMin) {vMin = busvoltage; dtostrf(vMin, 8, 2, vMinS); client.publish("BatVmin", vMinS);}
  if(current_mA < iMin) {iMin = current_mA; dtostrf(iMin, 8, 2, iMinS); client.publish("BatImin", iMinS);}
  if(power_mW < pMin)   {pMin = power_mW;   dtostrf(pMin, 8, 2, pMinS); client.publish("BatPmin", pMinS);}
  
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  uint32_t currentFrequency;
  ina219.begin();
  ina219.setCalibration_16V_400mA();

  startTime = millis();
                  
}

void loop() {

//while(elapsedTime1 <= 900000){
  
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
    
    currentTime = millis();
    elapsedTime1 = currentTime - startTime;

    battery();
    batteryMaxMinCheck();

    if (flag1 == 0){
        currentTimeAnt = currentTime;
        flag1 = 1;
    }

    if (flag1 == 1){
      
        deltaT = currentTime - currentTimeAnt; //Calculates time between each data aquisition
        deltaT = deltaT/(1000*60*60);          //Converts ms to h
        
        auxConsumoA = current_mA * deltaT;     //Multiplies the current reading with deltaT
        auxConsumoP = power_mW * deltaT;
        
        consumoAcumuladoA = consumoAcumuladoA + auxConsumoA; //Adds the new vlaue to the total
        consumoAcumuladoP = consumoAcumuladoP + auxConsumoP;
        
        currentTimeAnt = currentTime;
    }

    vAcumulado = vAcumulado + busvoltage;
    iAcumulado = iAcumulado + current_mA;
    pAcumulado = pAcumulado + power_mW;

    contador = contador + 1;
  
    if(contador >= 100){

      vMedia = vAcumulado/contador;
      iMedia = iAcumulado/contador;
      pMedia = pAcumulado/contador;
      
      dtostrf(vMedia, 8, 2, BusV);
      dtostrf(iMedia, 8, 2, Current);
      dtostrf(pMedia, 8, 2, Power);
  
      client.publish("BatV", BusV);
      client.publish("BatC", Current);
      client.publish("BatP", Power);
  
      dtostrf(elapsedTime1, 100, 0, elapsedTime2);
      client.publish("tempoB", elapsedTime2);

      dtostrf(consumoAcumuladoA, 25, 15, consumoTotalA);
      client.publish("ConsumoA", consumoTotalA);

      dtostrf(consumoAcumuladoP, 25, 15, consumoTotalP);
      client.publish("ConsumoP", consumoTotalP);

      vAcumulado = 0;
      iAcumulado = 0;
      pAcumulado = 0;
      contador = 0;

      delay(500);
      
    }
//  }
}
