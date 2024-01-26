#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_BMP085.h>
#include <RTClib.h>
#include <Wire.h>
#include <TimeAlarms.h>

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
float sumP
float Pprom;
Adafruit_BMP085 bmp;

/*-----Para el anemometro-----*/
float anemometro;

/*-------Para la veleta----------*/
float V1;
float Voltaje_max = 1.9; // El valor se encuentra en unidades de volts (V).
float angulo_max = 360; // El valor se encuentra en unidades de grados (°).
float angulo;

/*----------Para el pluviometro---------*/
#define pluviometro 5
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
}

void loop() {
  int Nmed = 0;
/*
  

  Serial.print("Radiacion:");
  
  Serial.println(rad);*/
  DateTime now = rtc.now();
  if (now.second() == 0){
    if (Nmed != 0){
      Tprom = sumT/Nmed;
      Pprom = sumP/Nmed;
      Hprom = sumH/Nmed;
      Nmed = 0;
    }
    else {
      Tprom = analogRead(A0);
      Tprom = ((Tprom*5000)/1023)/10;
      Pprom = bmp.readPressure ();
      Hprom = dht.readHumidity();
    }
    Serial.print(now.day());
    Serial.print(',');
    Serial.print(now.month());
    Serial.print(',');
    Serial.print(now.year());
    Serial.print(",");
    Serial.print(now.hour());
    Serial.print(',');
    Serial.print(now.minute());
    Serial.print(',');
    Serial.print(now.second());
    Serial.print(",");
    Serial.print(Tprom);
    Serial.print(",");
    Serial.print(Pprom);
    Serial.print(",");
    Serial.print(Hprom);
    Serial.print(",");
    anemometro = (analogRead(A1)*5000)/1023
    Serial.print(anemometro);
    Serial.print(",");
    V1 = analogRead(A0);
    // Convertimos de los valores del analogico a voltaje medido.
    V1 = (V1 * 5.00)/1023;
    if (V1>Voltaje_max)
      V1=Voltaje_max;
    // Convertimos de voltaje a grados.
    angulo = ((V1)*angulo_max)/Voltaje_max;
    Serial.print(angulo);
    Serial.print(",");
    Vin = analogRead(A2);
    Vreal = (R_V)*Vin*5.00/1023;
    Pow = (Vreal*Vreal)/Req;
    rad = Pow/a;
    Serial.print(rad);
    Serial.print(",");
    precp = contPulso*vol;
    Serial.print(precp);
    sumT = 0;
    sumP = 0;
    sumH = 0;
    T = analogRead(A0);
    T = ((T*5000)/1023)/10;
    sumT = sumT + T;
    P = bmp.readPressure ();
    sumP = sumP + P;
    H = dht.readHumidity();
    sumH = sumH + H;
    Nmed++;
  }
  else if ((now.second()%10) == 0){
    T = analogRead(A0);
    T = ((T*5000)/1023)/10;
    sumT = sumT + T;
    P = bmp.readPressure ();
    sumP = sumP + P;
    H = dht.readHumidity();
    sumH = sumH + H;
    Nmed++;
    Serial.print(now.day());
    Serial.print(',');
    Serial.print(now.month());
    Serial.print(',');
    Serial.print(now.year());
    Serial.print(",");
    Serial.print(now.hour());
    Serial.print(',');
    Serial.print(now.minute());
    Serial.print(',');
    Serial.print(now.second());
    Serial.print(",");
    Serial.print(Tprom);
    Serial.print(",");
    Serial.print(Pprom);
    Serial.print(",");
    Serial.print(Hprom);
    Serial.print(",");
    anemometro = (analogRead(A1)*5000)/1023
    Serial.print(anemometro);
    Serial.print(",");
    V1 = analogRead(A0);
    // Convertimos de los valores del analogico a voltaje medido.
    V1 = (V1 * 5.00)/1023;
    if (V1>Voltaje_max)
      V1=Voltaje_max;
    // Convertimos de voltaje a grados.
    angulo = ((V1)*angulo_max)/Voltaje_max;
    Serial.print(angulo);
    Serial.print(",");
    Vin = analogRead(A2);
    Vreal = (R_V)*Vin*5.00/1023;
    Pow = (Vreal*Vreal)/Req;
    rad = Pow/a;
    Serial.print(rad);
    Serial.print(",");
    precp = contPulso*vol;
    Serial.print(precp);
  }
  delay(1000);
}


void contar() {
  contPulso++;
}
