#define pluviometro 14

int contPulso = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pluviometro, INPUT);
  attachInterrupt(digitalPinToInterrupt(pluviometro), contar, FALLING);

}

void loop(){
  
}

  void contar() {
    static unsigned long ultimo_tiempo_interrupcion=0;
    unsigned long tiempo_interrupcion=millis();
    if(tiempo_interrupcion - ultimo_tiempo_interrupcion >200){
      contPulso++;
      Serial.println(contPulso);
    } 
     ultimo_tiempo_interrupcion=tiempo_interrupcion;
  }
