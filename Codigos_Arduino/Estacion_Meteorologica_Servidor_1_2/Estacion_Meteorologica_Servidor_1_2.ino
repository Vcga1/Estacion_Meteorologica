#include <DHT.h>                //Libreria para el sensor de humedad
#include <Adafruit_BMP280.h>    //Libreria para el sensor de presion
#include <RTClib.h>             //Libreria para el reloj de tiempo real
#include <Wire.h>               //Libreria para la comunicacion I2C
#include <SD.h>                 //Libreria para la comunicacion con la tarjeta SD
#include <FS.h>
#include <SPI.h>
#include <HTTPClient.h>         //Libreria para la comunicacion con el servidor web
#include <WiFi.h>               //Libreria para la conexion por WiFi

File myFile;

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

#define Conec_WiFi 33
#define WiFi_ok 25
#define Err_Modulo 26
#define Err_SD 12
#define server_OK 17
#define server_Err 4
#define server_Indef 16


void setup() {

  Serial.begin(9600);                                                      //Inicializacion del puerto serial
  delay(1000);

  pinMode(Conec_WiFi, OUTPUT);
  pinMode(WiFi_ok, OUTPUT);
  pinMode(Err_Modulo, OUTPUT);
  pinMode(Err_SD, OUTPUT);
  pinMode(server_OK, OUTPUT);
  pinMode(server_Err, OUTPUT);
  pinMode(server_Indef, OUTPUT);
  
  pinMode(pluviometro, INPUT);                                             //Inicializacion de el pin digital que va a contar las caidas de la cubeta
  attachInterrupt(digitalPinToInterrupt(pluviometro), contar, FALLING);    //Inicializacion de la interrupcion del pluviometro
  
  dht.begin();                                                             //Inicializacion del sensor DHT11
  delay(3000);
  
  if (! rtc.begin()) {
    Serial.println("No se puede detectar el módulo RTC");
    digitalWrite(Err_Modulo, HIGH);
    while (1);
  }                                                                         
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));                          //Inicializacion del reloj de tiempo real
  
  R_V = (R2 / ( R1 + R2 ));
  Req = R1 + R2;                                                           //Calculo de las resistencias para el calculo de la potencia generada por el panel solar
  
  if (!SD.begin(5)) {
    Serial.println("No se pudo inicializar");
    digitalWrite(Err_SD, HIGH);
    return;
  }
  Serial.println("inicializacion exitosa");
  digitalWrite(Err_SD, LOW);
  
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    digitalWrite(Err_Modulo, HIGH);
    while (1);
  }                                                                          //Inicializacion del sensor BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Modo de operación */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Presion oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtrado. */
                Adafruit_BMP280::STANDBY_MS_500); /* Tiempo Standby. */ 
                
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(Conec_WiFi,HIGH);
    delay(100);
    digitalWrite(Conec_WiFi,LOW);
    delay(100);
    digitalWrite(Conec_WiFi,HIGH);
    delay(100);
    digitalWrite(Conec_WiFi,LOW);
    delay(100);
    digitalWrite(Conec_WiFi,HIGH);
    delay(100);
    digitalWrite(Conec_WiFi,LOW);
    delay(100);
    digitalWrite(Conec_WiFi,HIGH);
    delay(100);
    digitalWrite(Conec_WiFi,LOW);
    delay(100);
    digitalWrite(Conec_WiFi,HIGH);
    delay(100);
    digitalWrite(Conec_WiFi,LOW);
    delay(100);
    digitalWrite(Conec_WiFi,HIGH);
    delay(100);
    digitalWrite(Conec_WiFi,LOW);
    delay(100);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a la red WiFi");                                 //Conexion a la red WiFi
  digitalWrite(Conec_WiFi,LOW);
  digitalWrite(WiFi_ok,HIGH); 
}

void loop() {
  digitalWrite(Err_SD, LOW);

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

      String mensaje = "d:" + String(now.day()) + ", m:" + String(now.month()) + ", a:" + String(now.year()) + ", h:" + String(now.hour()) + ", min:" + String(now.minute()) + ", s:" + String(now.second()) + ", t:" + String(T) + ", p:" + String(P) + ", hum:" + String(H) + ", v:" + String(anemometro) + ", dir:" + String(angulo) + ", rad:" + String(rad) + ", pre:" + String(precp) + "\n";   

      myFile = SD.open("/log.txt", FILE_APPEND);

      if (myFile) {
        myFile.print(mensaje);
        myFile.close();
      } 
      else {
        Serial.print("Error al escribir en la tarjeta SD");
        digitalWrite(Err_SD, HIGH);
      }

      sendPostRequest(now.day(),now.month(),now.year(),now.hour(), now.minute(), now.second(), T, P, H, anemometro, angulo, rad, precp);         
  }
    delay(1000);   //Espera por un segundo para volver a verificar la hora
}

/*Funcion para enviar la informacion al servidor web*/

void sendPostRequest(int D, int M, int A, int H, int mm, int s, float T, float P, float hum, float V, float dir, float rad, float pre) {
  HTTPClient http;

  digitalWrite(server_OK, LOW);
  digitalWrite(server_Indef, LOW);
  digitalWrite(server_Err, LOW);

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
    if (httpResponseCode == 201)
      digitalWrite(server_OK, HIGH);
    else
      digitalWrite(server_Indef, HIGH);
  }
  else {
    Serial.print("Error en la solicitud: ");
    Serial.println(httpResponseCode);
    digitalWrite(server_Err, HIGH);
  }

  // Libera los recursos
  http.end();
}

/*Funcion para contar la cantidad de interrupciones causadas por el pluviometro*/

void contar() {
  static unsigned long ultimo_tiempo_interrupcion=0;
  unsigned long tiempo_interrupcion=millis();
  if(tiempo_interrupcion - ultimo_tiempo_interrupcion >600){      //condicion para evitar rebotes
    contPulso++;
    Serial.println(contPulso);
  } 
   ultimo_tiempo_interrupcion=tiempo_interrupcion;
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}
