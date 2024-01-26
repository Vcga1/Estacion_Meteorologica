// Potenciometro conectado a GPIO 35
const int portPin = 35;
// Almacenamiento del valor de puerto (Rango de 0-4095)
float potValor = 0;
void setup() {
  Serial.begin(9600);
  delay(1000);
}
void loop() {
  // Lectura del valor en cada vuelta del bucle
  potValor = analogRead(portPin)*3.3/4095;
  Serial.println(potValor);  //Envío del valor al puerto serie
  delay(1000);
}
