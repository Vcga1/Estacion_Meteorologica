// Código del anemometro y de la veleta, donde el anemometro es el medidor de velocidad
//del viento y la veleta es el encargado de indicar la dirección del viento.


float anemometro;
float rev;
float V_0 = 0;

float V1;
float Voltaje_max = 1.9; // El valor se encuentra en unidades de volts (V).
float angulo_max = 360; // El valor se encuentra en unidades de grados (°).
float angulo;

void setup() {
  Serial.begin(9600);

}

void loop() {

  //Solo se mide el voltaje registrado por el movimiento de las aspas.
  anemometro = analogRead(A1);
  // Convertimos de los valores del analogico a voltaje medido.
  anemometro = (anemometro * 5.00)/1024;
  Serial.print("Voltaje medido por el anemometro: ");
  Serial.print(anemometro);
  Serial.println(" mV");


  // Determinamos el ángulo de rotación, con el cual se indica la orientación del viento.
  V1 = analogRead(A0);
  // Convertimos de los valores del analogico a voltaje medido.
  V1 = (V1 * 5.00)/1024;
  if (V1>Voltaje_max)
    V1=Voltaje_max;
  //Serial.println(V1);
  // Convertimos de voltaje a grados.
  angulo = ((V1)*angulo_max)/Voltaje_max;
  Serial.print("El ángulo del viento es: ");
  Serial.print(angulo);
  Serial.println(" °");
  
delay(1000); 
}
