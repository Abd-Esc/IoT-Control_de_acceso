#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <ESP32Servo.h>

// =====================================================
// 1. PINES DEL SISTEMA
// =====================================================

// PN532 en modo SPI
static const uint8_t PN532_SCK  = 18;
static const uint8_t PN532_MISO = 19;
static const uint8_t PN532_MOSI = 23;
static const uint8_t PN532_SS   = 27;

// Servo MG995
static const uint8_t SERVO_PIN = 21;

// LED verde de acceso autorizado
static const uint8_t LED_GREEN_PIN = 26;

// PIR HC-SR501
// Pin asumido porque no fue especificado por el usuario.
static const uint8_t PIR_PIN = 25;

// =====================================================
// 2. CONFIGURACIÓN DEL SERVO
// =====================================================

Servo servoAcceso;

static const int SERVO_CERRADO_GRADOS = 90;
static const int SERVO_ABIERTO_GRADOS = 0;

// Ajustes típicos para MG995.
// Si tu servo no llega bien a los extremos, ajustar estos valores.
static const int SERVO_PULSO_MIN_US = 500;
static const int SERVO_PULSO_MAX_US = 2400;

// =====================================================
// 3. CONFIGURACIÓN DE TIEMPOS
// =====================================================

// Si se autoriza acceso y nadie es detectado por el PIR,
// se cierra automáticamente después de 10 segundos.
static const unsigned long TIEMPO_MAX_APERTURA_SIN_PIR_MS = 10000;

// Cuando el PIR deja de detectar movimiento,
// se espera 2 segundos antes de cerrar.
static const unsigned long TIEMPO_ESPERA_CIERRE_TRAS_PIR_LOW_MS = 2000;

// Intervalo entre intentos de lectura del PN532.
// No conviene consultar el lector en cada ciclo del loop.
static const unsigned long INTERVALO_LECTURA_NFC_MS = 250;

// Evita aceptar la misma tarjeta muchas veces seguidas.
static const unsigned long COOLDOWN_TARJETA_MS = 2000;

// Tiempo inicial para ignorar lecturas del PIR al encender.
// Algunos HC-SR501 generan falsas detecciones al iniciar.
static const unsigned long TIEMPO_ESTABILIZACION_PIR_MS = 30000;

// =====================================================
// 4. CONFIGURACIÓN PN532
// =====================================================

// Usaremos SPI por hardware.
// En ESP32 los pines VSPI típicos son:
// SCK  = GPIO18
// MISO = GPIO19
// MOSI = GPIO23
// SS puede ser definido, aquí usamos GPIO27.
Adafruit_PN532 nfc(PN532_SS);

// Para prueba inicial:
// true  = acepta cualquier tarjeta detectada.
// false = solo acepta UIDs configurados en AUTHORIZED_UIDS.
static const bool ACEPTAR_CUALQUIER_TARJETA = true;

// Ejemplo de UIDs autorizados.
// Para producción, cambia estos valores por los UID reales de tus tarjetas.
// Si ACEPTAR_CUALQUIER_TARJETA = true, esta lista no se usa.
static const uint8_t AUTHORIZED_UIDS[][7] = {
  {0x04, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6}
};

static const uint8_t AUTHORIZED_UID_LENGTHS[] = {
  7
};

static const size_t AUTHORIZED_UID_COUNT =
  sizeof(AUTHORIZED_UID_LENGTHS) / sizeof(AUTHORIZED_UID_LENGTHS[0]);

// =====================================================
// 5. ESTADOS DEL SISTEMA
// =====================================================

enum class EstadoAcceso {
  CERRADO,
  ABIERTO_ESPERANDO_PIR,
  ABIERTO_CON_USUARIO_DETECTADO,
  ABIERTO_ESPERANDO_CIERRE
};

EstadoAcceso estadoActual = EstadoAcceso::CERRADO;

// =====================================================
// 6. VARIABLES DE CONTROL
// =====================================================

unsigned long tiempoInicioSistema = 0;
unsigned long tiempoApertura = 0;
unsigned long tiempoInicioPIRLow = 0;
unsigned long ultimaLecturaNFC = 0;
unsigned long ultimaTarjetaAceptada = 0;

int anguloServoActual = -1;

// =====================================================
// 7. UTILIDADES
// =====================================================

void moverServoSiEsNecesario(int nuevoAngulo) {
  if (anguloServoActual == nuevoAngulo) {
    return;
  }

  servoAcceso.write(nuevoAngulo);
  anguloServoActual = nuevoAngulo;

  Serial.print("[SERVO] Nuevo angulo: ");
  Serial.println(nuevoAngulo);
}

void encenderLedVerde(bool encendido) {
  digitalWrite(LED_GREEN_PIN, encendido ? HIGH : LOW);
}

bool pirEstaEstabilizado() {
  return millis() - tiempoInicioSistema >= TIEMPO_ESTABILIZACION_PIR_MS;
}

bool leerPIR() {
  if (!pirEstaEstabilizado()) {
    return false;
  }

  return digitalRead(PIR_PIN) == HIGH;
}

void imprimirUID(const uint8_t *uid, uint8_t uidLength) {
  for (uint8_t i = 0; i < uidLength; i++) {
    if (uid[i] < 0x10) {
      Serial.print("0");
    }

    Serial.print(uid[i], HEX);

    if (i < uidLength - 1) {
      Serial.print(":");
    }
  }
}

bool uidAutorizado(const uint8_t *uid, uint8_t uidLength) {
  if (ACEPTAR_CUALQUIER_TARJETA) {
    return true;
  }

  for (size_t i = 0; i < AUTHORIZED_UID_COUNT; i++) {
    if (uidLength != AUTHORIZED_UID_LENGTHS[i]) {
      continue;
    }

    bool coincide = true;

    for (uint8_t j = 0; j < uidLength; j++) {
      if (uid[j] != AUTHORIZED_UIDS[i][j]) {
        coincide = false;
        break;
      }
    }

    if (coincide) {
      return true;
    }
  }

  return false;
}

// =====================================================
// 8. ACCIONES DEL SISTEMA
// =====================================================

void abrirAcceso() {
  Serial.println("[ACCESO] Autorizado. Abriendo servo.");

  moverServoSiEsNecesario(SERVO_ABIERTO_GRADOS);
  encenderLedVerde(true);

  tiempoApertura = millis();
  estadoActual = EstadoAcceso::ABIERTO_ESPERANDO_PIR;
}

void cerrarAcceso(const char *motivo) {
  Serial.print("[ACCESO] Cerrando. Motivo: ");
  Serial.println(motivo);

  moverServoSiEsNecesario(SERVO_CERRADO_GRADOS);
  encenderLedVerde(false);

  estadoActual = EstadoAcceso::CERRADO;
}

// =====================================================
// 9. LECTURA DEL PN532
// =====================================================

void leerPN532SiCorresponde() {
  // Solo leer tarjetas cuando el sistema está cerrado.
  // Si ya está abierto, ignoramos nuevas tarjetas para evitar estados ambiguos.
  if (estadoActual != EstadoAcceso::CERRADO) {
    return;
  }

  unsigned long ahora = millis();

  if (ahora - ultimaLecturaNFC < INTERVALO_LECTURA_NFC_MS) {
    return;
  }

  ultimaLecturaNFC = ahora;

  uint8_t uid[7] = {0};
  uint8_t uidLength = 0;

  bool tarjetaDetectada = nfc.readPassiveTargetID(
    PN532_MIFARE_ISO14443A,
    uid,
    &uidLength
  );

  if (!tarjetaDetectada) {
    return;
  }

  Serial.print("[PN532] Tarjeta detectada. UID: ");
  imprimirUID(uid, uidLength);
  Serial.println();

  if (ahora - ultimaTarjetaAceptada < COOLDOWN_TARJETA_MS) {
    Serial.println("[PN532] Tarjeta ignorada por cooldown.");
    return;
  }

  if (!uidAutorizado(uid, uidLength)) {
    Serial.println("[PN532] Tarjeta NO autorizada.");
    return;
  }

  ultimaTarjetaAceptada = ahora;
  abrirAcceso();
}

// =====================================================
// 10. CONTROL DEL SERVO SEGÚN PIR Y TIMEOUT
// =====================================================

void actualizarEstadoAcceso() {
  bool pirActivo = leerPIR();
  unsigned long ahora = millis();

  switch (estadoActual) {
    case EstadoAcceso::CERRADO:
      // Estado seguro por defecto.
      moverServoSiEsNecesario(SERVO_CERRADO_GRADOS);
      encenderLedVerde(false);
      break;

    case EstadoAcceso::ABIERTO_ESPERANDO_PIR:
      // El acceso fue autorizado y el servo está abierto.
      // Si el usuario entra en el área del PIR, mantenemos abierto.
      if (pirActivo) {
        Serial.println("[PIR] Movimiento detectado. Manteniendo abierto.");
        estadoActual = EstadoAcceso::ABIERTO_CON_USUARIO_DETECTADO;
        break;
      }

      // Si nadie pasó después de 10 segundos, cerrar por seguridad.
      if (ahora - tiempoApertura >= TIEMPO_MAX_APERTURA_SIN_PIR_MS) {
        cerrarAcceso("timeout de 10 segundos sin deteccion PIR");
      }
      break;

    case EstadoAcceso::ABIERTO_CON_USUARIO_DETECTADO:
      // Mientras el PIR siga detectando movimiento,
      // el servo NO debe cambiar de ángulo.
      if (pirActivo) {
        break;
      }

      // El PIR dejó de detectar movimiento.
      // No cerramos inmediatamente: esperamos 2 segundos.
      Serial.println("[PIR] Sin movimiento. Iniciando espera de cierre.");
      tiempoInicioPIRLow = ahora;
      estadoActual = EstadoAcceso::ABIERTO_ESPERANDO_CIERRE;
      break;

    case EstadoAcceso::ABIERTO_ESPERANDO_CIERRE:
      // Si vuelve a detectar movimiento dentro de los 2 segundos,
      // cancelamos el cierre.
      if (pirActivo) {
        Serial.println("[PIR] Movimiento detectado nuevamente. Cancelando cierre.");
        estadoActual = EstadoAcceso::ABIERTO_CON_USUARIO_DETECTADO;
        break;
      }

      // Si pasan 2 segundos sin movimiento, cerramos.
      if (ahora - tiempoInicioPIRLow >= TIEMPO_ESPERA_CIERRE_TRAS_PIR_LOW_MS) {
        cerrarAcceso("PIR libre durante 2 segundos");
      }
      break;
  }
}

// =====================================================
// 11. SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  tiempoInicioSistema = millis();

  Serial.println();
  Serial.println("====================================");
  Serial.println("Sistema ESP32 + PN532 SPI + PIR + Servo");
  Serial.println("====================================");

  // LED
  pinMode(LED_GREEN_PIN, OUTPUT);
  encenderLedVerde(false);

  // PIR
  pinMode(PIR_PIN, INPUT);

  // Servo
  servoAcceso.setPeriodHertz(50);
  servoAcceso.attach(SERVO_PIN, SERVO_PULSO_MIN_US, SERVO_PULSO_MAX_US);
  moverServoSiEsNecesario(SERVO_CERRADO_GRADOS);

  // SPI para PN532
  SPI.begin(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

  // PN532
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();

  if (!versiondata) {
    Serial.println("[ERROR] No se encontro PN532.");
    Serial.println("Revisa:");
    Serial.println("- Modo SPI del modulo PN532");
    Serial.println("- SCK GPIO18");
    Serial.println("- MISO GPIO19");
    Serial.println("- MOSI GPIO23");
    Serial.println("- SS GPIO27");
    Serial.println("- VCC y GND");
    while (true) {
      encenderLedVerde(true);
      delay(150);
      encenderLedVerde(false);
      delay(150);
    }
  }

  Serial.print("[PN532] Chip encontrado. Firmware: ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print(".");
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  // Configura el PN532 en modo lector.
  nfc.SAMConfig();

  // Evita que readPassiveTargetID() bloquee indefinidamente.
  // 0xFF = esperar para siempre.
  // Valores bajos = menos bloqueo y loop más fluido.
  nfc.setPassiveActivationRetries(0x01);

  Serial.println("[SISTEMA] Inicializado correctamente.");
  Serial.println("[SISTEMA] Servo en 90 grados.");
  Serial.println("[SISTEMA] Esperando tarjeta NFC/RFID.");
  Serial.println("[PIR] Ignorando lecturas iniciales durante estabilizacion.");
}

// =====================================================
// 12. LOOP PRINCIPAL
// =====================================================

void loop() {
  actualizarEstadoAcceso();
  leerPN532SiCorresponde();
}