int S1;
int S2;
float V1;


void setup() {
  Serial.begin(9600);
  //Las entradas analogicas no requieren ser inicializadas.

}

void loop() {
  S1 = analogRead(A2);
  V1 = ((S1 * 5000.0)/1024);
  Serial.println(V1);
  delay(1000);
}
