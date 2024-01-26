#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_BMP280.h>
#include <RTClib.h>
#include <Wire.h>
#include <TimeAlarms.h>
#include <SD.h>

File myFile;

/*-----Para el sensor de temperatura-----*/
float T;
float sumT;
float Tprom;

/*-------Para el sensor de humedad-------*/
#define pinH 3
float H;
float sumH;
float Hprom;

DHT dht(pinH,DHT11);

/*-------Para el sensor de presion-------*/
float P;
//float sumP;
//float Pprom;
Adafruit_BMP280 bmp;

/*-----Para el anemometro-----*/
float anemometro;

/*-------Para la veleta----------*/
float V1;
float Voltaje_max = 1.9; // El valor se encuentra en unidades de volts (V).
float angulo_max = 360; // El valor se encuentra en unidades de grados (°).
float angulo;

/*----------Para el pluviometro---------*/
#define pluviometro 2
int contPulso = 0;
float vol = 3.5;
float precp = 0;
/*---------Para el panel solar-----------*/

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

int Nmed = 0;

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
  if (!SD.begin(4)) {
    Serial.println("No se pudo inicializar");
    return;
  }
  Serial.println("inicializacion exitosa");
if (!bmp.begin()) {
  Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  while (1);
}

bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Modo de operación */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Presion oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtrado. */
                Adafruit_BMP280::STANDBY_MS_500); /* Tiempo Standby. */
}


void loop() {
  int Nmed = 0;
/*
  

  Serial.print("Radiacion:");
  
  Serial.println(rad);*/
  DateTime now = rtc.now();

    if ((now.second()%10) == 0){
    //T = analogRead(A0);
    //T = ((T*5000)/1023)/10;
    //sumT = sumT + T;
    T = dht.readTemperature();
    P = bmp.readPressure ();
    //P = 0.0;
    //sumP = sumP + P;
    H = dht.readHumidity();
    //sumH = sumH + H;
    //Nmed++;
    myFile = SD.open("log.txt", FILE_WRITE);//abrimos  el archivo
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
        anemometro = (analogRead(A1)*5000.00)/1023;
        myFile.print(anemometro);
        myFile.print(",");
        V1 = analogRead(A0);
        // Convertimos de los valores del analogico a voltaje medido.
        V1 = (V1 * 5.00)/1023;
        if (V1>Voltaje_max)
          V1=Voltaje_max;
        // Convertimos de voltaje a grados.
        angulo = ((V1)*angulo_max)/Voltaje_max;
        myFile.print(angulo);
        myFile.print(",");
        Vin = analogRead(A2);
        Vreal = (R_V)*Vin*5.00/1023;
        Pow = (Vreal*Vreal)/Req;
        rad = Pow/a;
        myFile.print(rad);
        myFile.print(",");
        precp = contPulso*vol;
        myFile.println(precp);

        myFile.close(); //cerramos el archivo
    }
    else {
    Serial.println("Error al abrir el archivo");
  }
  }
  delay(1000);
}


void contar() {
  contPulso++;
}
