#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define ON_Board_LED 2 //LED en tarjeta

const char* ssid = "MikroTik";
const char* password = "STVultur*.";

//String url = "http://192.168.3.114:8000/api/personas/insert/";
String url = "http://192.168.40.246:8000/api/personas/1";

int id = 1;
//String nombre = "Gabriel";
//String cedula = "123456";

//char salidaJson[128];
 void httpConnetion() {

    HTTPClient client;

    client.begin( url );
    int httpCode = client.GET();

    Serial.println("El codigo de HTTP es " + String(httpCode));

    if ( httpCode > 0) {
      String payload = client.getString();
      Serial.println("El payload es " + payload);

      // Convertimos a un objeto JSON nuestros datos de la base de datos.
      DynamicJsonDocument doc(500);
      DeserializationError error= deserializeJson(doc,payload);

      if (error) {
        Serial.print("Error al analizar JSON: ");
        Serial.println(error.c_str());
        return;
      }
   }
 }
      
void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println();
  Serial.println("-------------");
  Serial.print("Connecting");

  int connecting_process_timed_out = 20;
  connecting_process_timed_out = connecting_process_timed_out * 2;

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");

    digitalWrite(ON_Board_LED, HIGH);
    delay(250);
    digitalWrite(ON_Board_LED, LOW);
    delay(250);

    if (connecting_process_timed_out > 0) connecting_process_timed_out--;
    if (connecting_process_timed_out == 0) {
      delay(1000);
      ESP.restart();
    }
  }
  Serial.println(WiFi.localIP());

}

void loop() {
  // put your main code here, to run repeatedly:
  /*if ((WiFi.status()== WL_CONNECTED)) {
    Serial.print("Conectado a Wi-fi");
    Serial.println();

    HTTPClient client;

    client.begin(url);
    client.addHeader("Content-Type","application/json");

    const size_t capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;

    JsonObject object = doc.to<JsonObject>();
    object ["id"] = id;
    object ["nombre"] = nombre;
    object ["cedula"] = cedula;

    id++;

    serializeJson (doc, salidaJson);

    int httpCode = client.POST(String(salidaJson));

   if ( httpCode > 0){
    Serial.println("\nStatuscode: "+ String(httpCode));
    client.end();
   }
   else{
    Serial.println("Error en la solicitud http");
   }
   
   }
   else{
    Serial.println("Sin Conexion");
   }
   */
   if ((WiFi.status()== WL_CONNECTED)) {
    Serial.print("Conectado a Wi-fi");
    Serial.println();
    httpConnetion();
   }
   delay(1000);
}
