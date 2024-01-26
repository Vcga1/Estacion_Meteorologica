#include <DHT.h>                //Libreria para el sensor de humedad
#include <Adafruit_BMP280.h>    //Libreria para el sensor de presion
#include <RTClib.h>             //Libreria para el reloj de tiempo real
#include <Wire.h>               //Libreria para la comunicacion I2C
#include <SD.h>                 //Libreria para la comunicacion con la tarjeta SD
#include <HTTPClient.h>         //Libreria para la comunicacion con el servidor web
#include <WiFi.h>               //Libreria para la conexion por WiFi

//File myFile;

/*-----Para el sensor de temperatura-----*/

#define pinT 34  

float T;         

/*-------Para el sensor de humedad-------*/

#define pinH 27

float H;
DHT dht(pinH,DHT11); 

/*-------Para el sensor de presion-------*/

float P;
Adafruit_BMP280 bmp;

/*-----Para el anemometro-----*/

#define pinAne 39

float anemometro;

/*-------Para la veleta----------*/

#define pinVel 36

float V1;
float Voltaje_max = 1.9; // El valor se encuentra en unidades de volts (V)
float angulo_max = 360;  // El valor se encuentra en unidades de grados (°)
float angulo;

/*----------Para el pluviometro---------*/

#define pluviometro 14

int contPulso = 0;
float vol = 4;           //Volumen de agua que hace que la cubeta caiga
float precp = 0;

/*---------Para el panel solar-----------*/

#define pinPan 32

float Pow;
float R1 = 10; // 10.000 ohm
float R2 = 13.3;
float a = 0.0064; // 64 centimetros cuadrados
float rad;
float R_V;
float Req;
float Vin;
float Vreal;

/*------Para la RTC--------*/

RTC_DS1307 rtc;

/*-----Para la conexion wifi-------*/

const char *ssid = "MikroTik";                                              // Nombre de la red WiFi
const char *password = "STVultur*.";                                        // Contraseña de la red WiFi
const char *apiEndpoint = "http://192.168.40.246:8000/api/medidas2/insert"; // URL a la que se envia la solicitud

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10       /* Time ESP32 will go to sleep (in seconds) */
#define latch 2

void setup(){
  Serial.begin(9600);
  pinMode(latch, OUTPUT);
  digitalWrite(latch,HIGH);
  delay(3000); //Take some time to open up the Serial Monitor

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Hibernar por " + String(TIME_TO_SLEEP) +
  " Segundos");
  
  pinMode(pluviometro, INPUT);                                             //Inicializacion de el pin digital que va a contar las caidas de la cubeta
  attachInterrupt(digitalPinToInterrupt(pluviometro), contar, FALLING);    //Inicializacion de la interrupcion del pluviometro
  
  dht.begin();                                                             //Inicializacion del sensor DHT11
  delay(3000);
  
  if (! rtc.begin()) {
    Serial.println("No se puede detectar el módulo RTC");
    while (1);
  }                                                                         
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));                          //Inicializacion del reloj de tiempo real
  
  R_V = (R2 / ( R1 + R2 ));
  Req = R1 + R2;                                                           //Calculo de las resistencias para el calculo de la potencia generada por el panel solar
  
  if (!SD.begin(5)) {
    Serial.println("No se pudo inicializar");
    return;
  }
  Serial.println("inicializacion exitosa");
  
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }                                                                          //Inicializacion del sensor BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Modo de operación */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Presion oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtrado. */
                Adafruit_BMP280::STANDBY_MS_500); /* Tiempo Standby. */ 
                
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a la red WiFi");                                 //Conexion a la red WiFi
  
  
}

void loop(){

  
  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  esp_deep_sleep_start();
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}
