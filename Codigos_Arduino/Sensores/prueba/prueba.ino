#include <SD.h>
#include <SPI.h>

const int chipSelect = 5; // El pin CS de la tarjeta SD puede variar, ajusta según sea necesario

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  // Inicializar la comunicación con la tarjeta SD
  if (!SD.begin(chipSelect)) {
    Serial.println("Error al inicializar la tarjeta SD.");
    return;
  }

  // Nombre del archivo
  String fileName = "/archivo.txt";

  // Cadena de texto que deseas agregar al final del archivo
  String textoAAgregar = "\n otra linea";

  // Llamar a la función para agregar texto al final del archivo
  agregarTextoAlFinal(fileName, textoAAgregar);
}

void loop() {
  // No hay nada que hacer en el bucle principal
}

void agregarTextoAlFinal(String fileName, String data) {
  // Abrir el archivo en modo de escritura al final
  File file = SD.open(fileName, FILE_WRITE);

  // Verificar si se pudo abrir el archivo correctamente
  if (file) {
    // Agregar la cadena de texto al final del archivo
    file.println(data);

    // Cerrar el archivo
    file.close();
    Serial.println("Texto agregado al final del archivo con éxito.");
  } else {
    // Si no se pudo abrir el archivo, imprimir un mensaje de error
    Serial.println("Error al abrir el archivo.");
  }
}
