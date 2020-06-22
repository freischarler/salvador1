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
const int led = 26;
bool bandera_sensar=true;
float h=0;
float t=0;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0xFF, 0x3E, 0x6C};

// Define variables to store BME280 readings to be sent
float temperature;
float humidity;
String status_ok;

// Define variables to store incoming readings
float incomingTime;
String incomingTrama;

// Variable to store if sending data was successful
String success;


/*
 * INTERRUPCION DE IR 
 */
 
// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
  Serial.println("MOTION DETECTED!!!");
  digitalWrite(led, LOW);
  bandera_sensar=false;
}

void read_sensores(){
  if (bandera_sensar==false){
    delay(10000);
    bandera_sensar=true;
  }
  if (bandera_sensar==true){
    h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    //float f = dht.readTemperature(true);
      delay(3000);
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }  
  } 
}

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float temp;
    float hum;
    String st;
} struct_message;

typedef struct struct_order{
  long time_on;
  String trama;
} struct_order;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message Sensor_Readings;

// Create a struct_message to hold incoming sensor readings
struct_order incomingReadings;

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
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

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

  pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);
  
  dht.begin();
}
 
void loop() {
  getReadings();
 
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
  //updateDisplay();
  delay(5000);
}

void getReadings(){
  temperature = 30;
  humidity = 20;
  status_ok = "hola";
}
