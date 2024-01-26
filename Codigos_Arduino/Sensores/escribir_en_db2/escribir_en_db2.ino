#include <HTTPClient.h>
#include <WiFi.h>

const char *ssid = "MikroTik";          // Nombre de tu red WiFi
const char *password = "STVultur*."; // Contraseña de tu red WiFi
const char *apiEndpoint = "http://192.168.40.246:8000/api/medidas2/insert"; // Cambia la URL según tu configuración

String nombre = "gabriel";
String cedula = "27310388";

void setup() {
  Serial.begin(115200);
  delay(100);

  // Conéctate a la red WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a la red WiFi");

  // Realiza la solicitud POST
  sendPostRequest(8,12,2023,11,9,30,20.00,81353.53/1000,32.00,4.89,240.78,3.11,3.50);
}

void sendPostRequest(int D, int M, int A, int H, int mm, int s, float T, float P, float hum, float V, float dir, float rad, float pre) {
  HTTPClient http;

  // Configura la solicitud POST
  http.begin(apiEndpoint);
  http.addHeader("Content-Type", "application/json");

  // Construye el cuerpo JSON de la solicitud
 String jsonBody = "{\"id\":1, \"d\":\"" + String(D) + "\", \"m\":\"" + String(M) + "\", \"a\":\"" + String(A) + "\", \"h\":\"" + String(H) + "\", \"min\":\"" + String(mm) + "\", \"s\":\"" + String(s) + "\", \"t\":\"" + String(T) + "\", \"p\":\"" + String(P) + "\", \"hum\":\"" + String(hum) + "\", \"v\":\"" + String(V) + "\", \"dir\":\"" + String(dir) + "\", \"rad\":\"" + String(rad) + "\", \"pre\":\"" + String(pre) + "\"}";

  // Realiza la solicitud POST
  int httpResponseCode = http.POST(jsonBody);

  // Maneja la respuesta del servidor
  if (httpResponseCode > 0) {
    Serial.print("Respuesta del servidor: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("Error en la solicitud: ");
    Serial.println(httpResponseCode);
  }

  // Libera los recursos
  http.end();
}

void loop() {
  // Puedes agregar aquí otras tareas si es necesario
  
}
