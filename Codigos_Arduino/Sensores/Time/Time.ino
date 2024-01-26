#include <RTClib.h>
#include <Wire.h>
#include <TimeLib.h>

RTC_DS1307 rtc;
void setup() {
  Serial.begin(9600);
  delay(3000);
  if (! rtc.begin()) {
  Serial.println("No hay un módulo RTC");
  while (1);
 }
 rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

}

void loop () {
 DateTime now = rtc.now();

 Serial.print(now.day());
 Serial.print('/');
 Serial.print(now.month());
 Serial.print('/');
 Serial.print(now.year());
 Serial.print(" ");
 Serial.print(now.hour());
 Serial.print(':');
 Serial.print(now.minute());
 Serial.print(':');
 Serial.print(now.second());
 Serial.println();
 delay(3000);
}
