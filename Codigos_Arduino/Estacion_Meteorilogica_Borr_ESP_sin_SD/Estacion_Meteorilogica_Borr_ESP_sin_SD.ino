#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_BMP280.h>
#include <RTClib.h>
#include <Wire.h>
#include <TimeAlarms.h>
#include <SD.h>
#include <HTTPClient.h>
#include <WiFi.h>

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
//float sumP;
//float Pprom;
Adafruit_BMP280 bmp;

/*-----Para el anemometro-----*/
float anemometro;
#define pinAne 39

/*-------Para la veleta----------*/
#define pinVel 36
float V1;
float Voltaje_max = 1.9; // El valor se encuentra en unidades de volts (V).
float angulo_max = 360; // El valor se encuentra en unidades de grados (°).
float angulo;

/*----------Para el pluviometro---------*/
#define pluviometro 14
int contPulso = 0;
float vol = 4;
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

const char *ssid = "MikroTik";          // Nombre de tu red WiFi
const char *password = "STVultur*."; // Contraseña de tu red WiFi
const char *apiEndpoint = "http://192.168.40.246:8000/api/medidas2/insert"; // Cambia la URL según tu configuración


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pluviometro, INPUT);
  attachInterrupt(digitalPinToInterrupt(pluviometro), contar, FALLING);
  dht.begin();
  delay(3000);
  if (! rtc.begin()) {
  Serial.println("No hay un módulo RTC");
  while (1);
 }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  R_V = (R2 / ( R1 + R2 ));
  Req = R1 + R2;
  /*if (!SD.begin(4)) {
    Serial.println("No se pudo inicializar");
    return;
  }
  Serial.println("inicializacion exitosa");*/
  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Modo de operación */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Presion oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtrado. */
                Adafruit_BMP280::STANDBY_MS_500); /* Tiempo Standby. */ 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a la red WiFi");
}

void loop() {
  int Nmed = 0;
/*
  

  Serial.print("Radiacion:");
  
  Serial.println(rad);*/
  DateTime now = rtc.now();

    if ((now.second()%10) == 0){
    //T = (analogRead(pinT)*3300.0/4095)/10;
    //T = ((T*5000)/1023)/10;
    //sumT = sumT + T;
    //T = dht.readTemperature();
    T = bmp.readTemperature();
    P = bmp.readPressure();
    //sumP = sumP + P;
    H = dht.readHumidity();
    //sumH = sumH + H;
    //Nmed++;
    anemometro = analogRead(pinAne)*3300.0/4095;
    V1 = analogRead(pinVel);
    // Convertimos de los valores del analogico a voltaje medido.
    V1 = V1*3.3/4095;
    if (V1>Voltaje_max)
      V1=Voltaje_max;
    // Convertimos de voltaje a grados.
    angulo = ((V1)*angulo_max)/Voltaje_max;
    Vin = analogRead(pinPan);
    Vreal = (R_V)*Vin*3.3/4095;
    Pow = (Vreal*Vreal)/Req;
    rad = Pow/a;
    precp = contPulso*vol;
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
  sendPostRequest(now.day(),now.month(),now.year(),now.hour(), now.minute(), now.second(), T, P, H, anemometro, angulo, rad, precp);
  }
  delay(1000);
}

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
  } else {
    Serial.print("Error en la solicitud: ");
    Serial.println(httpResponseCode);
  }

  // Libera los recursos
  http.end();
}

void contar() {
  static unsigned long ultimo_tiempo_interrupcion=0;
  unsigned long tiempo_interrupcion=millis();
  if(tiempo_interrupcion - ultimo_tiempo_interrupcion >200){
    contPulso++;
    Serial.println(contPulso);
  } 
   ultimo_tiempo_interrupcion=tiempo_interrupcion;
}
