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


void setup() {

  Serial.begin(9600);                                                      //Inicializacion del puerto serial
  
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
  
  /*if (!SD.begin(4)) {
    Serial.println("No se pudo inicializar");
    return;
  }
  Serial.println("inicializacion exitosa");*/
  
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

void loop() {
/*
  

  Serial.print("Radiacion:");
  
  Serial.println(rad);*/
  DateTime now = rtc.now();

    if ((now.second()%10) == 0){

      /*El codigo esta configurado para tomar medidas cada 10 segundos para las pruebas, si se cambia now.second() 
      por now.minute() se puede configurar para que las medidas sean cada 10 minutos*/
      
      //T = (analogRead(pinT)*3300.0/4095)/10;
      //T = dht.readTemperature();
      T = bmp.readTemperature();
      
      P = bmp.readPressure();
      
      H = dht.readHumidity();

      anemometro = analogRead(pinAne)*3300.0/4095;
      
      V1 = analogRead(pinVel)*3.3/4095;
      if (V1>Voltaje_max)
        V1=Voltaje_max;
      // Convertimos de voltaje a grados.
      angulo = ((V1)*angulo_max)/Voltaje_max;
      
      Vin = analogRead(pinPan)*3.3/4095;
      Vreal = (R_V)*Vin;                 //se multiplica el voltaje medido por la relacion del divisor de tension
      Pow = (Vreal*Vreal)/Req;           //Se calcula la potencia disipada por el arreglo de resistencias
      rad = Pow/a;                       //Se divide la potencia entre el area del panel solar
      
      precp = contPulso*vol;             //Se multiplica la cantidad de pulsos del pluviometro por el volumen previamente definido

      if (WiFi.status() != WL_CONNECTED){
        
        /*Si no hay conexion WiFi se guardan los valores medidos en la terjeta SD*/
        
        /*myFile = SD.open("log.txt", FILE_WRITE);//abrimos  el archivo
       
        if (myFile) { 
            Serial.println("Escribiendo informacion en SD...");
            myFile.print(now.day());
            myFile.print(',');
            myFile.print(now.month());
            myFile.print(',');
            myFile.print(now.year());
            myFile.print(",");
            myFile.print(now.hour());
            myFile.print(',');
            myFile.print(now.minute());
            myFile.print(',');
            myFile.print(now.second());
            myFile.print(",");
            myFile.print(T);
            myFile.print(",");
            myFile.print(P);
            myFile.print(",");
            myFile.print(H);
            myFile.print(",");
            myFile.print(anemometro);
            myFile.print(",");
            myFile.print(angulo);
            myFile.print(",");
            myFile.print(rad);
            myFile.print(",");
            myFile.println(precp);
            
            myFile.close(); //cerramos el archivo
        }
        else {
        Serial.println("Error al abrir el archivo");
      }*/
    }
      
    else{
      //Si hay conexion WiFi disponible se envia la informacion al servidor web
      sendPostRequest(now.day(),now.month(),now.year(),now.hour(), now.minute(), now.second(), T, P, H, anemometro, angulo, rad, precp);
    }            
  }
    delay(1000);   //Espera por un segundo para volver a verificar la hora
}

/*Funcion para enviar la informacion al servidor web*/

void sendPostRequest(int D, int M, int A, int H, int mm, int s, float T, float P, float hum, float V, float dir, float rad, float pre) {
  HTTPClient http;

  // Configura la solicitud POST
  http.begin(apiEndpoint);
  http.addHeader("Content-Type", "application/json");

  // Construye el cuerpo JSON de la solicitud
  String jsonBody = "{\"id\":1, \"d\":\"" + String(D) + "\", \"m\":\"" + String(M) + "\", \"a\":\"" + String(A) + "\", \"h\":\"" + String(H) + "\", \"min\":\"" + String(mm) + "\", \"s\":\"" + String(s) + "\", \"t\":\"" + String(T) + "\", \"p\":\"" + String(P) + "\", \"hum\":\"" + String(hum) + "\", \"v\":\"" + String(V) + "\", \"dir\":\"" + String(dir) + "\", \"rad\":\"" + String(rad) + "\", \"pre\":\"" + String(pre) + "\"}";

  // Realiza la solicitud POST
  int httpResponseCode = http.POST(jsonBody);

  // Maneja la respuesta del servidor
  if (httpResponseCode > 0) {
    Serial.print("Respuesta del servidor: ");
    Serial.println(httpResponseCode);
  }
  else {
    Serial.print("Error en la solicitud: ");
    Serial.println(httpResponseCode);
  }

  // Libera los recursos
  http.end();
}

/*Funcion para contar la cantidad de interrupciones causadas por el pluviometro*/

void contar() {
  static unsigned long ultimo_tiempo_interrupcion=0;
  unsigned long tiempo_interrupcion=millis();
  if(tiempo_interrupcion - ultimo_tiempo_interrupcion >500){      //condicion para evitar rebotes
    contPulso++;
    Serial.println(contPulso);
  } 
   ultimo_tiempo_interrupcion=tiempo_interrupcion;
}
