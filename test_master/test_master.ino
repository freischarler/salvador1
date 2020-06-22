//MASTER MAC: 30:AE:A4:FF:3E:6C
//SLAVE MAC: 30:AE:A4:FE:4D:30

#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>


#define pulsador_iniciar 36
#define pulsador_inc 32
#define pulsador_dec 39
#define pulsador_fin 34
//#define RELE 23

uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0xFE, 0x4D, 0x30};


// Define variables to store BME280 readings to be sent
float temperature;
float humidity;
String state;

// Define variables to store incoming readings
float incomingTemp;
float incomingHum;
String incomingState;

// Variable to store if sending data was successful
String success;

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
const int switchGPIOs[] = {36, 39, 34, 32};
const int totalSwitches = sizeof(switchGPIOs)/sizeof(switchGPIOs[0]);
int value=0;
bool accionar=false;
bool bool_pulsado=false;
bool control=false; //constante para controlar la comunicacion
bool sanitizador_on=false; //constante si esta on o off el sanitizador
int k=30;

long timer = 1000*1;
uint32_t t_start = 0;
uint32_t t_uv_start =0;
uint32_t t_min = 2000;
uint32_t d_finish=5000; //tiempo mostrando que finalizo el SANITIZADOR
uint8_t timer_running = false;
uint8_t timer_uv_running = false;

typedef struct struct_message {
    float temp;
    float hum;
    String st;
} struct_message;

typedef struct struct_order {
    int time_on;
    String trama;
} struct_order;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

struct_order sendOrder;

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
  incomingTemp = incomingReadings.temp;
  incomingHum = incomingReadings.hum;
  incomingState = incomingReadings.st;
  Serial.print(incomingTemp);
  Serial.print(" ");
  Serial.println(incomingHum);
  Serial.println(incomingState);
}

void initGPIOs() {
  for (int i = 0; i < totalSwitches; i++) {
    pinMode(switchGPIOs[i], INPUT);
  }
  //pinMode(RELE, OUTPUT);
  //digitalWrite(RELE,LOW);
}

void configuracion_personalizada();
void iniciar_sanitizador(int tiempo);

void configuracion_rapida(){
  iniciar_sanitizador(timer*k);
}

void show_Menu() {
   lcd.clear(); lcd.setCursor(0,0);
   Serial.println("Iniciando...");
   delay(500);
   int menu=1;
   int old_menu=0;
   while(1)
    {
        if(old_menu!=menu || accionar==true){
          old_menu=menu;   
          switch (menu)
          {
            
            case 1:
                Serial.println("1-Prueba");  
                lcd.clear(); lcd.setCursor(0,0);
                lcd.print("1-Prueba");
                delay(100);
                break;

            case 2:
                Serial.println("2-Inicio Rapido");
                lcd.clear(); lcd.setCursor(0,0);
                lcd.print("2-Inicio Rapido");
                if (accionar==true) {
                  configuracion_rapida();
                  lcd.clear(); lcd.setCursor(0,0);
                  lcd.print("FINALIZADO");
                  old_menu=99;
                  delay(d_finish);
                }  
                delay(100);  
                break;

            case 3:
                Serial.println("3-Inicio personalizado");
                lcd.clear(); lcd.setCursor(0,0);
                lcd.print("3-Inicio personalizado");
                if (accionar==true) {
                  configuracion_personalizada();
                  lcd.clear(); lcd.setCursor(0,0);
                  lcd.print("FINALIZADO");
                  old_menu=99;
                  delay(d_finish);
                }  
                delay(100);
                break;
          }
          accionar=false;
        }
        
        for (int i = 0; i < totalSwitches; i++) {
          value = digitalRead(switchGPIOs[i]);
          if(value>0){
            if(i==3){
              if(menu==3){
                old_menu=menu;
                menu=1;
              }else{
                old_menu=menu;
                menu++;
              }
            }
            if(i==0) accionar=true;
            //if(i==2) accionar=true;
            if(i==1) {
              if(menu==1){
                old_menu=menu;
                menu=3;
              }else{
                old_menu=menu;
                menu--;
              }   
            }
            delay(100);
          }
        //digitalWrite(ssrGPIOs[i], value);
        }  
    } 
}

void apagar(){
  sendOrder.time_on=0;
  sendOrder.trama="apagar";
  bool auxiliar=false;
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendOrder, sizeof(sendOrder));

  while(!auxiliar){
    if (result == ESP_OK) {
      Serial.println("Sent with success"); 
      auxiliar=true;
      control=false;
    }
    else {
      Serial.println("Error sending the data");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ERROR APAGAR");
      delay(5000);
    }     
  }
}

void iniciar_sanitizador(int tiempo){ 
  if(control==false){
    sendOrder.time_on=tiempo;
    sendOrder.trama="iniciar";
  
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendOrder, sizeof(sendOrder));
   
    if (result == ESP_OK) {
      Serial.println("Sent with success");
      control=true;
    }
    else {
      Serial.println("Error sending the data");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ERROR SEND");
      delay(5000);
    } 
  }

  if(sanitizador_on==false){
    while(!sanitizador_on){  //espero confirmacion que esta prendido
      if(incomingState=="on"){
        sanitizador_on=true;
        delay(500);
      }          
    }
  }

  while(sanitizador_on){
    // ON SANITIZADOR 
    if(!timer_uv_running){
      //digitalWrite(RELE,HIGH);
        t_uv_start=millis();
        timer_uv_running=true;
    }

    // OFF SANITIZADOR
    if (timer_uv_running==true && (millis()-t_uv_start)>tiempo){
      apagar();
      //digitalWrite(RELE,LOW);
      timer_uv_running=false;
      timer_running=false;
      sanitizador_on=false;
      return;
    }

    if (incomingState=="off"){
      return;
    }

  // DISPLAY DE VARIABLES HUMEDAD Y TEMPERATURA
    if (!timer_running) {
      // Timer is not already running, so capture the start time
      t_start = millis();
      timer_running = true;
    }
  
    if (timer_running==true && (millis()-t_start)>t_min){
      timer_running = false;
    }

    // OFF UV EN CASO DE APRETAR EL BOTON DE FIN
    if (digitalRead(pulsador_fin)==HIGH){
      apagar();
      //digitalWrite(RELE,LOW);
      timer_uv_running=false;
      timer_running=false;
      sanitizador_on=false;
      return;
    }

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print((millis()-t_uv_start));
    lcd.setCursor(0,1);
    lcd.print(incomingTemp);
    lcd.print(" ");
    lcd.print(incomingHum);

    delay(2000);
  
  
    sendOrder.time_on=tiempo;
    sendOrder.trama="continuar";
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendOrder, sizeof(sendOrder));   
  }
  return;
}


bool botones_on(){
  for (int i = 0; i < totalSwitches; i++) {
          value = digitalRead(switchGPIOs[i]);
          if(value>0){
            //delay(10);
            return true;
            }
  }
  return false;
}


void configuracion_personalizada(){
  while(botones_on())
  {}
  
  int contador=0;
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Ingrese t");
  delay(3000);
  bool bandera=true;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Ingrese t");
  lcd.setCursor(0,1);
  lcd.print("t="); lcd.print(contador);

  while(bandera){
    

    bool_pulsado=botones_on();
    
    if (digitalRead(pulsador_iniciar) == HIGH && bool_pulsado==true)     //Pregunta si el pulsador está presionado
      {
        long tiempo=timer*contador;
        Serial.println(tiempo);
        iniciar_sanitizador(tiempo);
        return;
      }


    if (digitalRead(pulsador_inc) == HIGH && bool_pulsado==true){     //Pregunta si el pulsador está presionado
        if(contador>=2){
          contador+= -1;    //La variable vuelve a su valor original
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Ingrese t");
          lcd.setCursor(0,1);
          lcd.print("t="); lcd.print(contador);
        }
    }
  
    if (digitalRead(pulsador_dec) == HIGH && bool_pulsado==true){     //Pregunta si el pulsador está presionado
        //Realiza la acción deseada
          contador+= 1;    //La variable vuelve a su valor original
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Ingrese t");
          lcd.setCursor(0,1);
          lcd.print("t="); lcd.print(contador);
    }
     
    if (digitalRead(pulsador_fin) == HIGH && bool_pulsado==true){
    return;
    }
    bool_pulsado=false;   
    while(botones_on()) //espera hasta que esten en OFF los botones
    {}   
  }
  
}

int show=0;



void setup()
{
  Serial.begin(115200);
  lcd.init();                      // initialize the lcd 
  initGPIOs();
  // Print a message to the LCD.
  lcd.backlight();
  
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
}
 
void loop()
{
  show_Menu();
}
