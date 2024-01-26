#include <SD.h>

File myFile;
void setup()
{
  Serial.begin(9600);
  Serial.print("Iniciando SD ...");
  if (!SD.begin(5)) {
    Serial.println("No se pudo inicializar");
    return;
  }
  Serial.println("inicializacion exitosa");
}

void loop()
{
  myFile = SD.open("log.txt", FILE_WRITE);//abrimos  el archivo
  
  if (myFile) { 
        Serial.print("Escribiendo SD: ");
        myFile.print("Tiempo(ms)=");
        myFile.println(millis());
        
        myFile.close(); //cerramos el archivo
                           
  
  } else {
    Serial.println("Error al abrir el archivo");
  }
  delay(10000);
}
