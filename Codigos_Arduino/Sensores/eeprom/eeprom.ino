// 24LC256 I2C EEPROM test code.

#include <24LC256.h>
uint8_t data[10];
// The constructor.
//E24LC256 MyEEPROM;                                          // Initialise EEPROM at default I2C address: 0x50
E24LC256 MyEEPROM(0x51);                                    // Initialise EEPROM at the given address (0x50-0x57).
union Data{
  float Temp;
  int   dia;
  unsigned char x[4];
}MyUni;
int pos = 0;
int posT,posd;


void setup() {
  Serial.begin(115200);
  Serial.println(F("Looking for EEPROM chip..."));
  MyEEPROM.init();
  switch (MyEEPROM.getStatus()) {
    case  MyEEPROM.EEPROM_FOUND:
      Serial.println(F("EEPROM found."));
      break;

    case MyEEPROM.EEPROM_NOT_FOUND:
      Serial.println(F("EEPROM not found. Halting operations."));
      while (true) {}
      break;

    default:
      Serial.println(F("Unkown status. Halting operations."));
      while (true) {}
      break;
  }

  Serial.println(F("Se escribe la temperatura en la EEPROM: "));
  MyUni.Temp = -100.50;
  posT=pos;
  for (uint8_t i = 0; i < 4; i++) {
    MyEEPROM.write(i+posT, MyUni.x[i]);
    Serial.print(MyUni.x[i], HEX);
    Serial.print(' ');    
    pos++;
  }
  Serial.println(' ');

  Serial.println(F("Se escribe el dia en la EEPROM: "));
  MyUni.dia = 24;
  posd=pos;
  for (uint8_t i = 0; i < 2; i++) {
    MyEEPROM.write(i+posd, MyUni.x[i]);
    Serial.print(MyUni.x[i], HEX);
    Serial.print(' '); 
    pos++;   
  }
  Serial.println(' ');
  
  Serial.print(F("Leyendo temperatura: "));
  MyUni.Temp=0;
  MyUni.dia=0;
  for (uint8_t i = 0; i < 4; i++) {
    MyUni.x[i] = MyEEPROM.read(i+posT);  
  }
  Serial.print(MyUni.Temp);
  Serial.print(' ');  
  
  Serial.println();
  Serial.print(F("Leyendo Dia: "));
  for (uint8_t i = 0; i < 2; i++) {
    MyUni.x[i] = MyEEPROM.read(i+posd);  
  }
  Serial.print(MyUni.dia);
  Serial.print(' '); 
}

void loop() {}
