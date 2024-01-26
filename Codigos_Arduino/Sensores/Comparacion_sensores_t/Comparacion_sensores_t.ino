#include <DHT.h>                //Libreria para el sensor de humedad
#include <Adafruit_BMP280.h>    //Libreria para el sensor de presion
#include <Wire.h>               //Libreria para la comunicacion I2C


#define pinT 34  

#define pinH 27
DHT dht(pinH,DHT11); 

Adafruit_BMP280 bmp;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  delay(2000); 
  dht.begin();  

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }                                                                          //Inicializacion del sensor BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Modo de operación */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Presion oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtrado. */
                Adafruit_BMP280::STANDBY_MS_500); /* Tiempo Standby. */ 

}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print("LM35: ");
  Serial.println((analogRead(pinT)*3300.0/4095)/10);
  Serial.print("DHT11: ");
  Serial.println(dht.readTemperature());
  Serial.print("BMP280: ");
  Serial.println(bmp.readTemperature());
  delay(1000);

}
