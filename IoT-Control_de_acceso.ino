#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_PN532.h>
#include <ESP32Servo.h>

// =====================================================
// 1. CONFIGURACIÓN WIFI Y BACKEND
// =====================================================

static const char* WIFI_SSID = "POMA";
static const char* WIFI_PASSWORD = "andresP23";

// Cambiar por la IP LAN de tu PC Fedora.
// Ejemplo: http://192.168.1.50:8000/api/v1
static const char* API_BASE_URL = "http://192.168.0.29:8000/api/v1";
static const char* NFC_ENDPOINT = "/acceso/nfc-scan";

// Actualmente el backend no exige API key.
// Se deja preparado para producción.
static const char* DEVICE_API_KEY = "";

// =====================================================
// 2. PINES DEL SISTEMA
// =====================================================

// PN532 en SPI
static const uint8_t PN532_SCK  = 18;
static const uint8_t PN532_MISO = 19;
static const uint8_t PN532_MOSI = 23;
static const uint8_t PN532_SS   = 27;

// Servo MG995
static const uint8_t SERVO_PIN = 21;

// LED acceso autorizado
static const uint8_t LED_GREEN_PIN = 26;

// PIR HC-SR501
static const uint8_t PIR_PIN = 25;

// LED encendido mientras el PIR detecta movimiento
static const uint8_t LED_PIR_PIN = 32;

// Buzzer activo para acceso denegado
static const uint8_t BUZZER_PIN = 33;

// =====================================================
// 3. CONFIGURACIÓN DEL SERVO
// =====================================================

Servo servoAcceso;

static const int SERVO_CERRADO_GRADOS = 90;
static const int SERVO_ABIERTO_GRADOS = 180;

static const int SERVO_PULSO_MIN_US = 500;
static const int SERVO_PULSO_MAX_US = 2400;

// =====================================================
// 4. CONFIGURACIÓN DE TIEMPOS
// =====================================================

static const unsigned long TIEMPO_MAX_APERTURA_SIN_PIR_MS = 10000;

// Modificado: antes 2000 ms, ahora 1000 ms.
static const unsigned long TIEMPO_ESPERA_CIERRE_TRAS_PIR_LOW_MS = 1000;

static const unsigned long INTERVALO_LECTURA_NFC_MS = 250;
static const unsigned long COOLDOWN_MISMA_TARJETA_MS = 2000;
static const unsigned long TIEMPO_ESTABILIZACION_PIR_MS = 30000;

static const unsigned long DURACION_BUZZER_DENEGADO_MS = 1000;
static const unsigned long INTERVALO_REINTENTO_WIFI_MS = 5000;
static const unsigned long INTERVALO_REINTENTO_PN532_MS = 5000;
static const unsigned long HTTP_TIMEOUT_MS = 3000;

// =====================================================
// 5. CONFIGURACIÓN PN532
// =====================================================

Adafruit_PN532 nfc(PN532_SS);

// true  = UID tipo "04:A1:B2:C3"
// false = UID tipo "04A1B2C3"
static const bool UID_CON_DOS_PUNTOS = true;

// =====================================================
// 6. ESTADOS DEL SISTEMA
// =====================================================

enum class EstadoAcceso {
  CERRADO,
  ABIERTO_ESPERANDO_PIR,
  ABIERTO_CON_USUARIO_DETECTADO,
  ABIERTO_ESPERANDO_CIERRE
};

struct RespuestaBackend {
  bool comunicacionOk;
  bool accesoConcedido;
  int httpCode;
  String mensaje;
  String motivoDenegacion;
};

EstadoAcceso estadoActual = EstadoAcceso::CERRADO;

// =====================================================
// 7. VARIABLES DE CONTROL
// =====================================================

unsigned long tiempoInicioSistema = 0;
unsigned long tiempoApertura = 0;
unsigned long tiempoInicioPIRLow = 0;

unsigned long ultimaLecturaNFC = 0;
unsigned long ultimaTarjetaLeidaMs = 0;
String ultimoUIDLeido = "";

unsigned long ultimoIntentoWifi = 0;
unsigned long ultimoIntentoPN532 = 0;

unsigned long tiempoInicioBuzzer = 0;
bool buzzerActivo = false;

bool pn532Disponible = false;
int anguloServoActual = -1;

// =====================================================
// 8. UTILIDADES DE SALIDA
// =====================================================

void moverServoSiEsNecesario(int nuevoAngulo) {
  if (anguloServoActual == nuevoAngulo) return;

  servoAcceso.write(nuevoAngulo);
  anguloServoActual = nuevoAngulo;

  Serial.print("[SERVO] Angulo: ");
  Serial.println(nuevoAngulo);
}

void encenderLedVerde(bool encendido) {
  digitalWrite(LED_GREEN_PIN, encendido ? HIGH : LOW);
}

void encenderLedPIR(bool encendido) {
  digitalWrite(LED_PIR_PIN, encendido ? HIGH : LOW);
}

void iniciarBuzzerDenegado() {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerActivo = true;
  tiempoInicioBuzzer = millis();
}

void actualizarBuzzer() {
  if (!buzzerActivo) return;

  if (millis() - tiempoInicioBuzzer >= DURACION_BUZZER_DENEGADO_MS) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActivo = false;
  }
}

// =====================================================
// 9. PIR
// =====================================================

bool pirEstaEstabilizado() {
  return millis() - tiempoInicioSistema >= TIEMPO_ESTABILIZACION_PIR_MS;
}

bool leerPIR() {
  if (!pirEstaEstabilizado()) return false;
  return digitalRead(PIR_PIN) == HIGH;
}

// =====================================================
// 10. WIFI
// =====================================================

void iniciarWiFi() {
  Serial.print("[WIFI] Conectando a ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  ultimoIntentoWifi = millis();
}

void actualizarWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long ahora = millis();

  if (ahora - ultimoIntentoWifi < INTERVALO_REINTENTO_WIFI_MS) return;

  Serial.println("[WIFI] Reintentando conexion.");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  ultimoIntentoWifi = ahora;
}

bool wifiDisponible() {
  return WiFi.status() == WL_CONNECTED;
}

// =====================================================
// 11. UID NFC
// =====================================================

String uidAString(const uint8_t* uid, uint8_t uidLength) {
  String resultado;

  for (uint8_t i = 0; i < uidLength; i++) {
    if (uid[i] < 0x10) resultado += "0";
    resultado += String(uid[i], HEX);

    if (UID_CON_DOS_PUNTOS && i < uidLength - 1) {
      resultado += ":";
    }
  }

  resultado.toUpperCase();
  return resultado;
}

// =====================================================
// 12. PN532
// =====================================================

bool inicializarPN532() {
  Serial.println("[PN532] Inicializando.");

  SPI.begin(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);
  nfc.begin();
  delay(100);

  uint32_t versiondata = nfc.getFirmwareVersion();

  if (!versiondata) {
    Serial.println("[PN532] No disponible. Se reintentara sin bloquear.");
    return false;
  }

  Serial.print("[PN532] Firmware: ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print(".");
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  nfc.SAMConfig();

  // Evita bloqueos largos en readPassiveTargetID().
  nfc.setPassiveActivationRetries(0x01);

  Serial.println("[PN532] Listo.");
  return true;
}

void actualizarPN532() {
  if (pn532Disponible) return;

  unsigned long ahora = millis();

  if (ahora - ultimoIntentoPN532 < INTERVALO_REINTENTO_PN532_MS) return;

  ultimoIntentoPN532 = ahora;
  pn532Disponible = inicializarPN532();
}

// =====================================================
// 13. BACKEND
// =====================================================

RespuestaBackend consultarBackendNFC(const String& nfcUid) {
  RespuestaBackend respuesta;
  respuesta.comunicacionOk = false;
  respuesta.accesoConcedido = false;
  respuesta.httpCode = -1;
  respuesta.mensaje = "Sin respuesta del backend";
  respuesta.motivoDenegacion = "Error de comunicacion";

  if (!wifiDisponible()) {
    respuesta.mensaje = "WiFi no conectado";
    return respuesta;
  }

  WiFiClient client;
  HTTPClient http;

  String url = String(API_BASE_URL) + String(NFC_ENDPOINT);

  if (!http.begin(client, url)) {
    respuesta.mensaje = "No se pudo iniciar HTTP";
    return respuesta;
  }

  http.setTimeout(HTTP_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/json");

  if (strlen(DEVICE_API_KEY) > 0) {
    http.addHeader("X-Device-Key", DEVICE_API_KEY);
  }

  StaticJsonDocument<128> requestDoc;
  requestDoc["nfc_uid"] = nfcUid;

  String requestBody;
  requestBody.reserve(80);
  serializeJson(requestDoc, requestBody);

  Serial.print("[HTTP] POST ");
  Serial.println(url);
  Serial.print("[HTTP] Body: ");
  Serial.println(requestBody);

  int httpCode = http.POST(requestBody);
  respuesta.httpCode = httpCode;

  String responseBody = http.getString();
  http.end();

  Serial.print("[HTTP] Code: ");
  Serial.println(httpCode);
  Serial.print("[HTTP] Response: ");
  Serial.println(responseBody);

  if (httpCode < 200 || httpCode >= 300) {
    respuesta.mensaje = "Backend respondio error HTTP";
    return respuesta;
  }

  StaticJsonDocument<512> responseDoc;
  DeserializationError error = deserializeJson(responseDoc, responseBody);

  if (error) {
    respuesta.mensaje = "JSON invalido desde backend";
    return respuesta;
  }

  respuesta.comunicacionOk = true;
  respuesta.accesoConcedido = responseDoc["acceso_concedido"] | false;
  respuesta.mensaje = responseDoc["mensaje"] | "";
  respuesta.motivoDenegacion = responseDoc["motivo_denegacion"] | "";

  return respuesta;
}

// =====================================================
// 14. ACCIONES DEL SISTEMA
// =====================================================

void abrirAcceso(const String& mensaje) {
  Serial.print("[ACCESO] Autorizado: ");
  Serial.println(mensaje);

  moverServoSiEsNecesario(SERVO_ABIERTO_GRADOS);
  encenderLedVerde(true);

  tiempoApertura = millis();
  estadoActual = EstadoAcceso::ABIERTO_ESPERANDO_PIR;
}

void denegarAcceso(const String& motivo) {
  Serial.print("[ACCESO] Denegado: ");
  Serial.println(motivo);

  moverServoSiEsNecesario(SERVO_CERRADO_GRADOS);
  encenderLedVerde(false);
  iniciarBuzzerDenegado();

  estadoActual = EstadoAcceso::CERRADO;
}

void cerrarAcceso(const char* motivo) {
  Serial.print("[ACCESO] Cerrando: ");
  Serial.println(motivo);

  moverServoSiEsNecesario(SERVO_CERRADO_GRADOS);
  encenderLedVerde(false);

  estadoActual = EstadoAcceso::CERRADO;
}

// =====================================================
// 15. LECTURA NFC
// =====================================================

void leerPN532SiCorresponde() {
  if (estadoActual != EstadoAcceso::CERRADO) return;
  if (!pn532Disponible) return;

  unsigned long ahora = millis();

  if (ahora - ultimaLecturaNFC < INTERVALO_LECTURA_NFC_MS) return;

  ultimaLecturaNFC = ahora;

  uint8_t uid[7] = {0};
  uint8_t uidLength = 0;

  bool tarjetaDetectada = nfc.readPassiveTargetID(
    PN532_MIFARE_ISO14443A,
    uid,
    &uidLength
  );

  if (!tarjetaDetectada) return;

  String uidTexto = uidAString(uid, uidLength);

  Serial.print("[PN532] UID: ");
  Serial.println(uidTexto);

  if (uidTexto == ultimoUIDLeido &&
      ahora - ultimaTarjetaLeidaMs < COOLDOWN_MISMA_TARJETA_MS) {
    Serial.println("[PN532] UID ignorado por cooldown.");
    return;
  }

  ultimoUIDLeido = uidTexto;
  ultimaTarjetaLeidaMs = ahora;

  RespuestaBackend respuesta = consultarBackendNFC(uidTexto);

  if (!respuesta.comunicacionOk) {
    denegarAcceso(respuesta.mensaje);
    return;
  }

  if (respuesta.accesoConcedido) {
    abrirAcceso(respuesta.mensaje);
  } else {
    String motivo = respuesta.motivoDenegacion.length() > 0
      ? respuesta.motivoDenegacion
      : respuesta.mensaje;

    denegarAcceso(motivo);
  }
}

// =====================================================
// 16. CONTROL POR PIR
// =====================================================

void actualizarEstadoAcceso() {
  bool pirActivo = leerPIR();
  encenderLedPIR(pirActivo);

  unsigned long ahora = millis();

  switch (estadoActual) {
    case EstadoAcceso::CERRADO:
      moverServoSiEsNecesario(SERVO_CERRADO_GRADOS);
      encenderLedVerde(false);
      break;

    case EstadoAcceso::ABIERTO_ESPERANDO_PIR:
      if (pirActivo) {
        Serial.println("[PIR] Movimiento detectado.");
        estadoActual = EstadoAcceso::ABIERTO_CON_USUARIO_DETECTADO;
        break;
      }

      if (ahora - tiempoApertura >= TIEMPO_MAX_APERTURA_SIN_PIR_MS) {
        cerrarAcceso("timeout sin deteccion PIR");
      }
      break;

    case EstadoAcceso::ABIERTO_CON_USUARIO_DETECTADO:
      if (pirActivo) break;

      Serial.println("[PIR] Sin movimiento. Esperando cierre.");
      tiempoInicioPIRLow = ahora;
      estadoActual = EstadoAcceso::ABIERTO_ESPERANDO_CIERRE;
      break;

    case EstadoAcceso::ABIERTO_ESPERANDO_CIERRE:
      if (pirActivo) {
        Serial.println("[PIR] Movimiento nuevamente. Cierre cancelado.");
        estadoActual = EstadoAcceso::ABIERTO_CON_USUARIO_DETECTADO;
        break;
      }

      if (ahora - tiempoInicioPIRLow >= TIEMPO_ESPERA_CIERRE_TRAS_PIR_LOW_MS) {
        cerrarAcceso("PIR libre durante 1 segundo");
      }
      break;
  }
}

// =====================================================
// 17. SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  tiempoInicioSistema = millis();

  Serial.println();
  Serial.println("====================================");
  Serial.println("ESP32 + PN532 + PIR + Servo + Backend");
  Serial.println("====================================");

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_PIR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);

  encenderLedVerde(false);
  encenderLedPIR(false);
  digitalWrite(BUZZER_PIN, LOW);

  servoAcceso.setPeriodHertz(50);
  servoAcceso.attach(SERVO_PIN, SERVO_PULSO_MIN_US, SERVO_PULSO_MAX_US);
  moverServoSiEsNecesario(SERVO_CERRADO_GRADOS);

  iniciarWiFi();

  ultimoIntentoPN532 = millis() - INTERVALO_REINTENTO_PN532_MS;
  actualizarPN532();

  Serial.println("[SISTEMA] Inicializado.");
  Serial.println("[SISTEMA] El acceso depende del backend.");
  Serial.println("[PIR] Ignorando lecturas iniciales por estabilizacion.");
}

// =====================================================
// 18. LOOP
// =====================================================

void loop() {
  actualizarWiFi();
  actualizarPN532();
  actualizarBuzzer();
  actualizarEstadoAcceso();
  leerPN532SiCorresponde();
}