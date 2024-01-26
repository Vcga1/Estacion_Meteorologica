/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-sd-card
 */

#include <SD.h>
#include <SPI.h>

#define PIN_SPI_CS 5 // The ESP32 pin GPIO5

void setup() {
  Serial.begin(9600);
  delay(1000);

  if (!SD.begin(PIN_SPI_CS)) {
    Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
    while (1); // stop the program
  }

  Serial.println(F("SD CARD INITIALIZED."));

  // open file for writing
  File myFile = SD.open("/esp32.txt", FILE_WRITE);

  if (myFile) {
    myFile.println("Created by esp32io.com"); // write a line to esp32.txt
    myFile.println("Learn ESP32 and SD Card"); // write another line to esp32.txt
    myFile.close();
  } else {
    Serial.print(F("SD Card: Issue encountered while attempting to open the file esp32.txt"));
  }

}

void loop() {
}
