//MASTER MAC: 30:AE:A4:FF:3E:6C
//SLAVE MAC: 30:AE:A4:FE:3C:30

#include "DHT.h"
#define DHTPIN 4     // Digital pin connected to the DHT sensor

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);
const int motionSensor = 27;
const int RELE = 21;
bool bandera_sensar=true;
bool sanitizador_on=false;
float h=0;
float t=0;

uint32_t t_uv_start =0;
uint8_t timer_uv_running = false;

uint32_t t_sens_start =0;
uint8_t t_sens_running = false;
int t_sens_min=3000;

int tiempo=20000;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0xFF, 0x3E, 0x6C};

// Define variables to store BME280 readings to be sent
float temperature=0;
float humidity=0;
String status_ok="";

// Define variables to store incoming readings
float incomingTime;
String incomingTrama;

// Variable to store if sending data was successful
String success;


/*
 * INTERRUPCION DE IR 
 */
 
/*/ Checks if motion was detected, sets LED HIGH and starts a timer/*
void IRAM_ATTR detectsMovement() {
  Serial.println("MOTION DETECTED!!!");
  digitalWrite(RELE, HIGH);
  bandera_sensar=false;
  sanitizador_on=false;
}*/

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float temp;
    float hum;
    String st;
} struct_message;

typedef struct struct_order{
  int time_on;
  String trama;
} struct_order;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message Sensor_Readings;

// Create a struct_message to hold incoming sensor readings
struct_order incomingReadings;


void read_sensores(){  
  h = dht.readHumidity();
    // Read temperature as Celsius (the default)
  t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    //float f = dht.readTemperature(true);
      
    // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

}



// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingTime = incomingReadings.time_on;
  incomingTrama = incomingReadings.trama;
  Serial.print(incomingTime);
  Serial.print(" ");
  Serial.println(incomingTrama);

  if(incomingTrama=="continuar" && sanitizador_on==true){
  t_uv_start=millis();
  timer_uv_running=true;  
  }
  
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(motionSensor, INPUT_PULLUP);
  pinMode(RELE, OUTPUT);
  digitalWrite(RELE, HIGH); 
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);


  
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  //attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);
  
  dht.begin();
}
 
void loop() {
  if(incomingTrama=="iniciar"){
    Serial.println("INICIANDO SATINIZADOR PELIGRO!!");
    sanitizador_on=true;
    digitalWrite(RELE, LOW);  
    status_ok = "on";
    Sensor_Readings.temp = temperature;
    Sensor_Readings.hum = humidity;
    Sensor_Readings.st = status_ok;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Sensor_Readings, sizeof(Sensor_Readings));
    delay(5000);

    bool bandera=true;
    while(bandera){
      if(incomingTrama=="apagar" || sanitizador_on==false){
      Serial.println("SANITIZADOR OFF");
      sanitizador_on=false;
      bandera=false;
      }

      if (!t_sens_running){
      // Timer is not already running, so capture the start time
        t_sens_start = millis();
        t_sens_running = true;
      }
    
      if (t_sens_running==true && (millis()-t_sens_start)>t_sens_min){
        read_sensores();
        t_sens_running = false;
      }

      

      temperature = t;
      humidity = h;
  

        
      if(analogRead(motionSensor)>5){
        Serial.println("MOTION DETECTED!!!");
        digitalWrite(RELE, HIGH);
        bandera_sensar=false;
        sanitizador_on=false;
      }
        



      if (timer_uv_running==true && (millis()-t_uv_start)>tiempo){
        Serial.println("SATINIZADOR OFF");
        sanitizador_on=false;
        timer_uv_running=false;
        bandera=false;
      }

      if(sanitizador_on){
        digitalWrite(RELE, LOW);
        status_ok = "on";
      } 
      else{
        status_ok = "off";
        digitalWrite(RELE, HIGH); 
        bandera=false;  
      } 
           
      // Set values to send
      Sensor_Readings.temp = temperature;
      Sensor_Readings.hum = humidity;
      Sensor_Readings.st = status_ok;

      // Send message via ESP-NOW
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Sensor_Readings, sizeof(Sensor_Readings));
   
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
      delay(2000);  
    }
  }
  else{

          if (!t_sens_running){
      // Timer is not already running, so capture the start time
        t_sens_start = millis();
        t_sens_running = true;
      }
    
      if (t_sens_running==true && (millis()-t_sens_start)>t_sens_min){
        read_sensores();
        t_sens_running = false;
      }
        // Set values to send
    Sensor_Readings.temp = temperature;
    Sensor_Readings.hum = humidity;
    Sensor_Readings.st = status_ok;

      // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Sensor_Readings, sizeof(Sensor_Readings));
   
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
      delay(1500);  
    }
}


  
