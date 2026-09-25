/*
  ============================================================================
  ZÉFIRO STRIX - TETRA MAIN BOARD - FIRMWARE V4.0.0
  ============================================================================
  Dispositivo: ESP32-S3-WROOM-1U-N16 + PCM1808 (ADC) + PCM5102A (DAC)
               + micrófono digital externo opcional (I2S) + doble microSD

  ============================================================================
  SUPUESTOS Y PENDIENTES A VERIFICAR EN BANCO (leer antes de flashear)
  ============================================================================
  1. Se asume que las pistas de playback en CARD4 son archivos .WAV
     (mono, 16-bit PCM, SAMPLE_RATE definido abajo), NO .mp3 reales.
     Si tus pistas actuales son .mp3, hay que convertirlas antes de copiarlas.
  2. El formato de bits del PCM1808 (cómo vienen alineados los datos dentro
     de la trama I2S de 32 bits) se copió del mismo shift (>>11) que usaba
     el INMP441 en el firmware DIY. Puede necesitar ajuste tras la primera
     grabación de prueba si el audio sale muy bajo, saturado o con ruido.
  3. Polaridad de MIC_SEL_PIN (GPIO9): asumo, según datasheet estándar del
     SN74HC157, que nivel BAJO selecciona el micrófono digital externo
     (canal A) y nivel ALTO selecciona el PCM1808/mic analógico (canal B).
     Verificar con un multímetro o con una grabación de prueba de cada modo.
  4. Ambas tarjetas SD comparten bus SPI (MOSI/MISO/SCLK) y se montan de a
     una por vez (nunca simultáneas), porque el ciclo de trabajo real es
     secuencial: reproducir desde CARD4 → desmontar → montar CARD3 → grabar.
     Esto evita la complejidad de manejar dos instancias de sistema de
     archivos SD activas al mismo tiempo.
  5. La ganancia de playback es una constante fija de firmware
     (PLAYBACK_GAIN_PCT, ver #define), aplicada a las muestras antes de
     enviarlas por I2S. Calibrado en banco 25-sep: pot AP1 al maximo sin
     saturar, incluso en pistas ruidosas.
  6. Librerías requeridas (Gestor de Bibliotecas de Arduino IDE):
     RTClib, ArduinoJson, NimBLE-Arduino, Adafruit NeoPixel, SD, SPI
     (DFRobotDFPlayerMini YA NO se usa, puedes desinstalarla si quieres).

  HARDWARE:
  - ESP32-S3-WROOM-1U-N16 (Tetra Main Board)
  - PCM1808 (ADC, mic analógico vía CN6)  + PCM5102A (DAC, salida vía CN1→PAM→parlante)
  - Mic digital externo opcional (JST U24, MIC1-D) — selección por SN74HC157 (MICSW)
  - CARD3: config.json + sesiones.csv + grabaciones de respuesta
  - CARD4: pistas de playback (.wav) de llamadas de aves
  - RTC DS3231 (I2C)
  - BME280 opcional vía conector I2C externo (U20) — NO implementado en esta
    versión, mismo bus I2C que el RTC, se puede agregar después sin tocar pines.
  - 8x WS2812B (GPIO47) — ver leds.h
  - IP2326 (carga), BAT_STAT digital en GPIO7 (sin ADC de voltaje en esta
    revisión de PCB — ver notas de v2)
  - Botón "stealth" de estado en GPIO6

  PINES (confirmados contra el esquemático EasyEDA, sesión de auditoría):
  - I2S BCK=18  WS(LRCK)=8   MCLK=17(→PCM1808 SCKI)
  - I2S DATA_OUT=4 (→PCM5102A DIN)   I2S DATA_IN=5 (←mux MICSW)
  - MIC_SEL(MICSW)=9
  - SPI MOSI=11  MISO=13  SCLK=12
  - SD_CS_MAIN(CARD3)=14   SD_CS_PLAYBACK(CARD4)=1
  - I2C SDA=19  SCL=20
  - BOTON_ESTADO(TACT)=6   BAT_STAT=7   LEDC(WS2812B)=10
  ============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <sys/time.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_adc_cal.h>
#include <driver/adc.h>
#include <driver/rtc_io.h>

// NOTA: el límite de tamaño de característica BLE (BLE_ATT_ATTR_MAX_LEN) se
// edita directamente en la librería instalada: NimBLEAttValue.h (512→2048).
// Cualquiera que compile este proyecto en otro computador debe aplicar ese
// mismo cambio manualmente en su copia de la librería.
#include <NimBLEDevice.h>
#include "portal.h"
#include "leds.h"

// ============================================================================
// PINES
// ============================================================================
// --- Audio I2S (bus compartido entre PCM1808 y PCM5102A) ---
#define I2S_BCK           18
#define I2S_WS            8
#define I2S_MCLK          17
#define I2S_DATA_OUT      4    // hacia PCM5102A (playback)
#define I2S_DATA_IN       5    // desde el mux MICSW (grabación)
#define PLAYBACK_GAIN_PCT  6  // ganancia digital de playback (0-30,
                              // formula original: gananciaDigital =
                              // PLAYBACK_GAIN_PCT/20.0). Calibrado en
                              // banco 25-sep: pot AP1 al maximo sin
                              // saturar, incluso en pistas ruidosas.
#define MIC_SEL_PIN       9    // SN74HC157: LOW=mic digital externo, HIGH=PCM1808 analógico
#define MIC_SOURCE_DIGITAL LOW
#define MIC_SOURCE_ANALOG  HIGH

// --- SPI / SD (bus compartido, dos chip-select) ---
#define SPI_MOSI          11
#define SPI_MISO          13
#define SPI_SCLK          12
#define SD_CS_MAIN        14   // CARD3: config, CSV, grabaciones
#define SD_CS_PLAYBACK    1    // CARD4: pistas de playback (.wav)

// --- I2C (RTC + sensores externos) ---
#define I2C_SDA           19
#define I2C_SCL           20

// --- Otros ---
#define BOTON_ESTADO_PIN  6    // botón "stealth"
#define BAT_STAT_PIN      7    // IP2326: trickle vs carga normal (digital, sin voltaje)
#define VBAT_ADC_PIN            GPIO_NUM_3   // modificacion v1.0: divisor externo (no en esquematico original)
#define VBAT_ADC_CHANNEL        ADC1_CHANNEL_2
#define VBAT_RATIO              29.4f        // ratio real calibrado en banco (modulo sensor + divisor R1=3.3k/R2=10k)
#define VBAT_CUTOFF_V           9.6f         // 3.2V/celda, pack 3S - entra a modo proteccion
#define VBAT_RESUME_V           10.2f        // histeresis - no reactivar hasta este voltaje
#define VBAT_SAMPLES            32
#define BATTERY_PROTECTION_SLEEP_US  (30ULL * 60 * 1000000ULL)  // 30 min en modo proteccion

// ============================================================================
// CONSTANTES DE FIRMWARE
// ============================================================================
#define FW_VERSION        "4.0.0-tetra"
#define AP_SSID           "ZefiroStrix-Config"
#define AP_PASS           ""
#define AP_IP             "192.168.4.1"
#define PORTAL_TIMEOUT_S  60
#define CONFIG_PATH       "/config.json"
#define CSV_PATH          "/sesiones.csv"
#define REC_DIR_ROOT      "/REC"

// ── Protocolo BLE Zéfiro Strix v1 (ver docs/zefiro_ble_protocol_v1.md) ─────
#define BLE_BOARD_TYPE       "tetra-pcb"
#define BLE_SERVICE_UUID     "4d617b4f-4320-4e1b-b6c0-1e6a52a81ba9"
#define BLE_CONFIG_UUID      "770440e9-947e-4983-a405-3fdd67dd43db"
#define BLE_STATUS_UUID      "babdcdd4-83aa-45da-9444-1737d5ff6a2e"
#define BLE_TIMESYNC_UUID    "398eaab7-1b17-4529-ab0d-d2ccedce80fe"
#define BLE_COMMAND_UUID     "62b3db56-e022-4efc-a2e7-af19c4f69a3f"
#define BLE_TRACKS_PAGE_UUID "ee9249c7-eb47-43ef-a8b8-132e9f24b7ed"
#define BLE_TRACKS_DATA_UUID "9d181ffd-a6ae-497b-96b2-719fb223531d"
#define TRACKS_PAGE_SIZE     5
#define MAX_TRACKS           30

const char* NTP_SSID = "CHUCAO2-2.4";
const char* NTP_PASS = "ANANUKA8";

// ============================================================================
// CONSTANTES DE AUDIO
// ============================================================================
#define SAMPLE_RATE     8000
#define BITS_PER_SAMPLE 16
#define CHUNK_SAMPLES   2048

// Desplazamiento de bits para llevar la muestra I2S (32 bits) a rango de
// 16 bits. Es DISTINTO segun la fuente de microfono, porque son chips con
// formatos de salida diferentes - no compartir un solo valor entre ambos.
//
// DIGITAL (mic externo custom de Rod, fabricado en JLCPCB): chip
// identificado por su esquematico = Knowles/Syntiant SPH0655LM4H-1-8.
// Pinout (DATA/SELECT/CLOCK + nets SD/BCK/LRK) es identico al de su primo
// mejor documentado, el SPH0645LM4H-1: I2S, 24 bits, MSB primero. Un dato
// de 24 bits justificado a la izquierda en una trama de 32 bits requiere
// >>16 para llegar a 16 bits utiles. Con la medicion real de +26.1 dBFS
// usando >>11, este valor calcula a ~-4 dBFS, razonable para mic cerca de
// una fuente de sonido.
// ADVERTENCIA: la familia SPH06xx de Knowles tiene un defecto de
// alineacion de bits conocido en la comunidad (el SPH0645 corre el dato
// 1 bit respecto al datasheet). Si el audio suena con crujido/distorsion
// rara (no solo muy fuerte/debil), probar >>15 o >>17.
#define MIC_BIT_SHIFT_DIGITAL   16
//
// ANALOGICO (PCM1808, TI): NO VERIFICADO AUN EN BANCO. El PCM1808 entrega
// 24 bits justificados a la izquierda en una trama I2S de 32 bits, dato de
// datasheet - shift teorico >>16 para llegar a 16 bits utiles (8 para
// extraer los 24 bits validos, 8 mas para bajar de 24 a 16). Tratar como
// punto de partida, no como valor confirmado - probar igual que se hizo
// con el digital.
#define MIC_BIT_SHIFT_ANALOG    16

// ============================================================================
// ESTRUCTURA DE CONFIGURACIÓN (igual al DIY + micSource nuevo)
// ============================================================================
struct TrackConfig {
  int    order;
  char   species[48];
  bool   active;
};

struct Config {
  char stationName[48];
  char projectName[48];
  char researcher[48];
  char unitName[16];
  char utmZone[6];
  long utmEaste;
  long utmNorte;
  int  morningStartH, morningStartM;
  int  morningEndH,   morningEndM;
  int  nightStartH,   nightStartM;
  int  nightEndH,     nightEndM;
  int  recTimeSeg;
  int  pauseMs;
  int  gainFactor;   // ganancia de grabación (igual que antes)
  int  micSource;    // 0 = digital externo (default), 1 = analógico PCM1808
  TrackConfig tracks[MAX_TRACKS];
  int trackCount;
  long totalSessions;
  long totalRecordings;
};

// ============================================================================
// OBJETOS GLOBALES
// ============================================================================
RTC_DS3231           rtc;
Adafruit_BME280      bme;
bool                 bmeDisponible = false;  // sensor opcional, no bloquea el arranque si falta
WebServer            server(80);
DNSServer            dnsServer;
bool                 portalShutdown = false;
bool                 shutdownPending = false;
unsigned long        shutdownRequestedAt = 0;
bool                 timerActivo    = true;
Config               cfg;
bool                 sdMainMontada  = false;
bool                 sdPlaybackMontada = false;

// Sobrevive al deep sleep; solo se resetea con un power-cycle real.
RTC_DATA_ATTR bool    batteryProtectionActive = false;

NimBLEServer*        bleServer          = nullptr;
NimBLECharacteristic* bleConfigChar     = nullptr;
NimBLECharacteristic* bleStatusChar     = nullptr;
NimBLECharacteristic* bleTimeSyncChar   = nullptr;
NimBLECharacteristic* bleCommandChar    = nullptr;
NimBLECharacteristic* bleTracksPageChar = nullptr;
NimBLECharacteristic* bleTracksDataChar = nullptr;
int                   bleTracksSelectedPage = 0;
volatile bool         bleClienteConectado = false;
uint16_t              bleConnHandle = 0;
bool                  clienteActivo       = false;

// ============================================================================
// PROTOTIPOS
// ============================================================================
void     cargarConfigDefecto();
bool     cargarConfigSD();
bool     guardarConfigSD();
void     iniciarPortal();
void     manejarPortal();
void     setupEndpoints();
void     cerrarPortal();
bool     diagnosticoSistema();
bool     diagnosticoMicrofono();
bool     iniciarBME280();
bool     leerBME280(float& temperaturaC, float& humedadPct, float& presionHpa);
void     sincronizarHoraNTP();
bool     esHoraActiva(int hora, int minuto);
bool     dentroDeRango(int t, int ini, int fin);
void     ejecutarCiclo();
void     reproducirYGrabar(int trackFisico, int idxFisico);
bool     reproducirTrackWav(const char* filePath);
void     grabarRespuesta(int trackFisico, int idxFisico);
void     registrarCSV(const char* archivo, int trackNum, float rms, float durSeg);
bool     crearDirectorio(const char* path);
bool     montarSDPrincipal();
bool     montarSDPlayback();
void     desmontarSD();
void     initI2SFullDuplex();
void     seleccionarMicrofono(int fuente);
int      shiftMicActual();
void     writeWavHeader(File& f);
void     sellarWav(File& f, uint32_t dataSize);
bool     leerWavHeader(File& f, uint32_t& sampleRate, uint16_t& bits, uint16_t& canales);
void     iniciarBLE();
void     detenerBLE();
void     actualizarStatusBLE();
String   buildConfigJsonBLE();
bool     aplicarConfigJsonBLE(const String& body);
String   buildTracksPageJsonBLE(int page);
bool     aplicarTracksPageJsonBLE(const String& body);
void     revisarBotonEstado();
void     actualizarLedBateria();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n============================================");
  Serial.println("  ZEFIRO STRIX TETRA - V" FW_VERSION);
  Serial.println("  Monitor acustico de aves nocturnas");
  Serial.println("============================================\n");

  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.println("[BOOT] RF apagada.");

  pinMode(BOTON_ESTADO_PIN, INPUT_PULLUP);
  pinMode(BAT_STAT_PIN, INPUT);
  pinMode(MIC_SEL_PIN, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SD_CS_MAIN);

  initLeds();

  // Si el despertar fue por el boton de estado (deep sleep -> ext1), solo
  // mostramos el estado actual y volvemos a dormir, sin iniciar un ciclo
  // completo de monitoreo.
  esp_sleep_wakeup_cause_t causaDespertar = esp_sleep_get_wakeup_cause();
  if (causaDespertar == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.println("[BOOT] Despertar por boton de estado.");
    montarSDPrincipal();
    cargarConfigSD();
    desmontarSD();
    actualizarLedBateria();
    unsigned long t0 = millis();
    while (millis() - t0 < 3000) { updateLeds(); delay(10); }
    Serial.println("[BOOT] Volviendo a dormir.");
    configurarWakeupBoton();
    esp_sleep_enable_timer_wakeup(5ULL * 60 * 1000000ULL);
    esp_deep_sleep_start();
  }

  startLedsStartupAnimation();
  unsigned long tAnim = millis();
  while (millis() - tAnim < 1600) { updateLeds(); delay(5); }

  if (!diagnosticoSistema()) {
    Serial.println("\n[CRITICO] Sistema detenido. Corrige los fallos y reinicia.");
    showLedsStatus(COLOR_ROJO, 10);
    while (true) { updateLeds(); delay(10); }
  }

  montarSDPrincipal();
  if (!cargarConfigSD()) {
    Serial.println("[CONFIG] Usando configuracion por defecto.");
    cargarConfigDefecto();
    guardarConfigSD();
  }

  iniciarPortal();
  iniciarBLE();
  manejarPortal();
  cerrarPortal();

  DateTime now = rtc.now();
  struct timeval tv = { .tv_sec = (time_t)now.unixtime(), .tv_usec = 0 };
  settimeofday(&tv, NULL);

  desmontarSD();
  initI2SFullDuplex();
  seleccionarMicrofono(cfg.micSource);

  Serial.println("\n[BOOT] Sistema listo. Iniciando monitoreo...\n");
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  float vBat = leerVoltajeBateria();
  Serial.printf("[BAT] VBAT=%.2fV\n", vBat);

  if (batteryProtectionActive) {
    if (vBat >= VBAT_RESUME_V) {
      batteryProtectionActive = false;
      Serial.println("[BAT] Voltaje recuperado. Saliendo de modo proteccion.");
    } else {
      Serial.println("[BAT] Modo proteccion activo. Ciclo omitido, durmiendo 30 min.");
      configurarWakeupBoton();
      esp_sleep_enable_timer_wakeup(BATTERY_PROTECTION_SLEEP_US);
      esp_deep_sleep_start();
    }
  } else if (vBat < VBAT_CUTOFF_V) {
    batteryProtectionActive = true;
    Serial.println("[BAT] VBAT bajo umbral de seguridad. Entrando en modo proteccion.");
    configurarWakeupBoton();
    esp_sleep_enable_timer_wakeup(BATTERY_PROTECTION_SLEEP_US);
    esp_deep_sleep_start();
  }

  montarSDPrincipal();
  DateTime now   = rtc.now();
  int hora       = now.hour();
  int minuto     = now.minute();
  desmontarSD();

  if (esHoraActiva(hora, minuto)) {
    Serial.printf("\n[CICLO] Horario activo (%02d:%02d). Iniciando secuencia...\n",
                  hora, minuto);
    ejecutarCiclo();
    montarSDPrincipal();
    cfg.totalSessions++;
    guardarConfigSD();
    desmontarSD();
    Serial.println("[CICLO] Completado. Deep Sleep 5 min...");
  } else {
    Serial.printf("[SLEEP] Fuera de horario (%02d:%02d). Durmiendo 5 min...\n",
                  hora, minuto);
  }

  configurarWakeupBoton();
  esp_sleep_enable_timer_wakeup(5ULL * 60 * 1000000ULL);
  esp_deep_sleep_start();
}

// ============================================================================
// BATERIA: LECTURA DE VOLTAJE POR DIVISOR RESISTIVO (ADC1, GPIO3) - v1.0
// ============================================================================
float leerVoltajeBateria() {
  static bool adcInicializado = false;
  static esp_adc_cal_characteristics_t adc_chars_vbat;

  if (!adcInicializado) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(VBAT_ADC_CHANNEL, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                              1100, &adc_chars_vbat);
    adcInicializado = true;
  }

  uint32_t suma_mV = 0;
  for (int i = 0; i < VBAT_SAMPLES; i++) {
    int raw = adc1_get_raw(VBAT_ADC_CHANNEL);
    suma_mV += esp_adc_cal_raw_to_voltage(raw, &adc_chars_vbat);
    delay(2);
  }

  float vPin_V = (suma_mV / (float)VBAT_SAMPLES) / 1000.0f;
  return vPin_V * VBAT_RATIO;
}

// ============================================================================
// DEEP SLEEP: WAKE-UP POR BOTON DE ESTADO (GPIO6)
// ============================================================================
void configurarWakeupBoton() {
  uint64_t mask = 1ULL << BOTON_ESTADO_PIN;
  rtc_gpio_pullup_en((gpio_num_t)BOTON_ESTADO_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)BOTON_ESTADO_PIN);
  esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ALL_LOW);
}

// ============================================================================
// LED DE ESTADO SEGUN BATERIA (usa solo BAT_STAT, sin voltaje real - v1)
// ============================================================================
void actualizarLedBateria() {
  // BAT_STAT: HIGH normalmente = no cargando o carga normal, comportamiento
  // exacto (activo alto/bajo) a confirmar en banco con el IP2326 real.
  // Por ahora: solo mostramos un pulso verde de "equipo responde" al
  // presionar el boton, sin poder distinguir niveles reales de bateria
  // hasta la v2 del PCB (divisor ADC pendiente).
  showLedsStatus(COLOR_VERDE, 2);
}

// ============================================================================
// BOTON DE ESTADO (uso en operacion normal, fuera de deep sleep)
// ============================================================================
unsigned long ultimoBotonMs = 0;
void revisarBotonEstado() {
  if (digitalRead(BOTON_ESTADO_PIN) == LOW) {
    if (millis() - ultimoBotonMs > 400) {  // debounce simple
      ultimoBotonMs = millis();
      actualizarLedBateria();
    }
  }
}

// ============================================================================
// CONFIGURACIÓN: VALORES POR DEFECTO
// ============================================================================
void cargarConfigDefecto() {
  strlcpy(cfg.stationName, "Sin nombre",        sizeof(cfg.stationName));
  strlcpy(cfg.projectName, "Proyecto",          sizeof(cfg.projectName));
  strlcpy(cfg.researcher,  "Investigador",      sizeof(cfg.researcher));
  strlcpy(cfg.unitName,    "TETRA-01",          sizeof(cfg.unitName));
  strlcpy(cfg.utmZone,     "19S",               sizeof(cfg.utmZone));
  cfg.utmEaste = 0;
  cfg.utmNorte = 0;

  cfg.morningStartH = 6;  cfg.morningStartM = 22;
  cfg.morningEndH   = 7;  cfg.morningEndM   = 22;
  cfg.nightStartH   = 18; cfg.nightStartM   = 55;
  cfg.nightEndH     = 19; cfg.nightEndM     = 55;

  cfg.recTimeSeg  = 20;
  cfg.pauseMs     = 500;
  cfg.gainFactor  = 3;
  cfg.micSource   = 0;  // digital externo por defecto (mejor calidad)

  const char* especiesDefault[7] = {
    "Chuncho (Glaucidium nanum)",
    "Concon (Strix rufipes)",
    "Lechuza (Tyto alba)",
    "Tucuquere (Bubo magellanicus)",
    "Nuco (Asio flammeus)",
    "Especie 6",
    "Especie 7"
  };
  cfg.trackCount = 7;
  for (int i = 0; i < cfg.trackCount; i++) {
    cfg.tracks[i].order  = i + 1;
    cfg.tracks[i].active = true;
    strlcpy(cfg.tracks[i].species, especiesDefault[i], sizeof(cfg.tracks[i].species));
  }

  cfg.totalSessions   = 0;
  cfg.totalRecordings = 0;
}

// ============================================================================
// CONFIGURACIÓN: LEER DESDE SD (CARD3)
// ============================================================================
bool cargarConfigSD() {
  if (!SD.exists(CONFIG_PATH)) return false;

  File f = SD.open(CONFIG_PATH, FILE_READ);
  if (!f) return false;

  StaticJsonDocument<4096> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { Serial.printf("[CONFIG] JSON invalido: %s\n", err.c_str()); return false; }

  strlcpy(cfg.stationName, doc["stationName"] | "Sin nombre",   sizeof(cfg.stationName));
  strlcpy(cfg.projectName, doc["projectName"] | "Proyecto",     sizeof(cfg.projectName));
  strlcpy(cfg.researcher,  doc["researcher"]  | "Investigador", sizeof(cfg.researcher));
  strlcpy(cfg.unitName,    doc["unitName"]    | "TETRA-01",     sizeof(cfg.unitName));
  strlcpy(cfg.utmZone,     doc["utmZone"]     | "19S",          sizeof(cfg.utmZone));
  cfg.utmEaste = doc["utmEaste"] | 0;
  cfg.utmNorte = doc["utmNorte"] | 0;

  auto parseTime = [](const char* s, int& h, int& m) {
    if (!s || strlen(s) < 5) return;
    h = atoi(s);
    m = atoi(s + 3);
  };
  parseTime(doc["morningStart"] | "06:22", cfg.morningStartH, cfg.morningStartM);
  parseTime(doc["morningEnd"]   | "07:22", cfg.morningEndH,   cfg.morningEndM);
  parseTime(doc["nightStart"]   | "18:55", cfg.nightStartH,   cfg.nightStartM);
  parseTime(doc["nightEnd"]     | "19:55", cfg.nightEndH,     cfg.nightEndM);

  cfg.recTimeSeg  = doc["recTime"]    | 20;
  cfg.pauseMs     = doc["pauseMs"]    | 500;
  cfg.gainFactor  = doc["gainFactor"] | 3;
  cfg.micSource   = doc["micSource"]  | 0;

  JsonArray tracks = doc["tracks"];
  cfg.trackCount = doc["trackCount"] | (int)tracks.size();
  if (cfg.trackCount > MAX_TRACKS) cfg.trackCount = MAX_TRACKS;
  if (cfg.trackCount < 0) cfg.trackCount = 0;
  for (int i = 0; i < cfg.trackCount && i < (int)tracks.size(); i++) {
    cfg.tracks[i].order  = tracks[i]["order"]  | (i + 1);
    cfg.tracks[i].active = tracks[i]["active"] | true;
    strlcpy(cfg.tracks[i].species, tracks[i]["species"] | "Especie", sizeof(cfg.tracks[i].species));
  }

  cfg.totalSessions   = doc["totalSessions"]   | 0;
  cfg.totalRecordings = doc["totalRecordings"] | 0;

  Serial.println("[CONFIG] Configuracion cargada desde SD.");
  return true;
}

// ============================================================================
// CONFIGURACIÓN: ESCRIBIR EN SD (CARD3)
// ============================================================================
bool guardarConfigSD() {
  StaticJsonDocument<4096> doc;

  doc["stationName"] = cfg.stationName;
  doc["projectName"] = cfg.projectName;
  doc["researcher"]  = cfg.researcher;
  doc["unitName"]    = cfg.unitName;
  doc["utmZone"]     = cfg.utmZone;
  doc["utmEaste"]    = cfg.utmEaste;
  doc["utmNorte"]    = cfg.utmNorte;

  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.morningStartH, cfg.morningStartM);
  doc["morningStart"] = buf;
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.morningEndH,   cfg.morningEndM);
  doc["morningEnd"]   = buf;
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.nightStartH,   cfg.nightStartM);
  doc["nightStart"]   = buf;
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.nightEndH,     cfg.nightEndM);
  doc["nightEnd"]     = buf;

  doc["recTime"]    = cfg.recTimeSeg;
  doc["pauseMs"]    = cfg.pauseMs;
  doc["gainFactor"] = cfg.gainFactor;
  doc["micSource"]  = cfg.micSource;

  doc["trackCount"] = cfg.trackCount;
  JsonArray tracks = doc.createNestedArray("tracks");
  for (int i = 0; i < cfg.trackCount; i++) {
    JsonObject t  = tracks.createNestedObject();
    t["order"]    = cfg.tracks[i].order;
    t["species"]  = cfg.tracks[i].species;
    t["active"]   = cfg.tracks[i].active;
  }

  doc["totalSessions"]   = cfg.totalSessions;
  doc["totalRecordings"] = cfg.totalRecordings;

  File f = SD.open(CONFIG_PATH, FILE_WRITE);
  if (!f) { Serial.println("[CONFIG] Error abriendo config.json para escribir."); return false; }
  serializeJson(doc, f);
  f.close();
  return true;
}

// ============================================================================
// SD: MONTAR / DESMONTAR (bus compartido, una tarjeta a la vez)
// ============================================================================
bool montarSDPrincipal() {
  if (sdPlaybackMontada) { SD.end(); sdPlaybackMontada = false; }
  if (sdMainMontada) return true;
  if (!SD.begin(SD_CS_MAIN, SPI)) {
    Serial.println("[SD] Error montando CARD3 (principal).");
    return false;
  }
  sdMainMontada = true;
  return true;
}

bool montarSDPlayback() {
  if (sdMainMontada) { SD.end(); sdMainMontada = false; }
  if (sdPlaybackMontada) return true;
  if (!SD.begin(SD_CS_PLAYBACK, SPI)) {
    Serial.println("[SD] Error montando CARD4 (playback).");
    return false;
  }
  sdPlaybackMontada = true;
  return true;
}

void desmontarSD() {
  if (sdMainMontada || sdPlaybackMontada) {
    SD.end();
    sdMainMontada = false;
    sdPlaybackMontada = false;
  }
}

// ============================================================================
// I2S: FULL DUPLEX (una sola instalacion sirve para playback y grabacion,
// ya que BCK/WS son compartidos entre PCM1808 y PCM5102A por diseno de PCB)
// ============================================================================
void initI2SFullDuplex() {
  i2s_config_t cfg_i2s = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .mck_io_num = I2S_MCLK,
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_DATA_OUT,
    .data_in_num = I2S_DATA_IN
  };
  i2s_driver_install(I2S_NUM_0, &cfg_i2s, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// ============================================================================
// SELECCIONAR FUENTE DE MICROFONO (mux SN74HC157, GPIO9)
// ============================================================================
void seleccionarMicrofono(int fuente) {
  // fuente: 0 = digital externo, 1 = analogico (PCM1808)
  // VERIFICAR EN BANCO: polaridad asumida segun tabla de verdad estandar
  // del SN74HC157 (SEL=LOW -> canal A -> mic digital; SEL=HIGH -> canal B -> PCM1808).
  if (fuente == 0) {
    digitalWrite(MIC_SEL_PIN, MIC_SOURCE_DIGITAL);
    Serial.println("[MIC] Fuente: microfono digital externo.");
  } else {
    digitalWrite(MIC_SEL_PIN, MIC_SOURCE_ANALOG);
    Serial.println("[MIC] Fuente: microfono analogico (PCM1808).");
  }
}

// Devuelve el shift de bits correcto segun la fuente de microfono activa
// (ver MIC_BIT_SHIFT_DIGITAL / MIC_BIT_SHIFT_ANALOG mas arriba).
int shiftMicActual() {
  return (cfg.micSource == 0) ? MIC_BIT_SHIFT_DIGITAL : MIC_BIT_SHIFT_ANALOG;
}

// ============================================================================
// PORTAL: INICIAR ACCESS POINT
// ============================================================================
void iniciarPortal() {
  String ssid = String(AP_SSID);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid.c_str(), AP_PASS);
  delay(300);
  IPAddress ip;
  ip.fromString(AP_IP);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));

  dnsServer.start(53, "*", ip);
  setupEndpoints();
  server.begin();

  portalShutdown = false;
  timerActivo    = true;
  clienteActivo  = false;

  Serial.printf("[PORTAL] AP \"%s\" iniciado en %s\n", AP_SSID, AP_IP);
}

// ============================================================================
// PORTAL: ENDPOINTS HTTP
// ============================================================================
void setupEndpoints() {
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send_P(200, "text/html; charset=utf-8", PORTAL_HTML);
  });

  server.on("/config", HTTP_GET, []() {
    StaticJsonDocument<4096> doc;
    doc["stationName"] = cfg.stationName;
    doc["projectName"] = cfg.projectName;
    doc["researcher"]  = cfg.researcher;
    doc["unitName"]    = cfg.unitName;
    doc["utmZone"]     = cfg.utmZone;
    doc["utmEaste"]    = cfg.utmEaste;
    doc["utmNorte"]    = cfg.utmNorte;

    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", cfg.morningStartH, cfg.morningStartM);
    doc["morningStart"] = buf;
    snprintf(buf, sizeof(buf), "%02d:%02d", cfg.morningEndH, cfg.morningEndM);
    doc["morningEnd"]   = buf;
    snprintf(buf, sizeof(buf), "%02d:%02d", cfg.nightStartH, cfg.nightStartM);
    doc["nightStart"]   = buf;
    snprintf(buf, sizeof(buf), "%02d:%02d", cfg.nightEndH, cfg.nightEndM);
    doc["nightEnd"]     = buf;

    doc["recTime"]    = cfg.recTimeSeg;
    doc["gainFactor"] = cfg.gainFactor;
    doc["micSource"]  = cfg.micSource;

    doc["trackCount"] = cfg.trackCount;
    JsonArray tracks = doc.createNestedArray("tracks");
    for (int i = 0; i < cfg.trackCount; i++) {
      JsonObject t = tracks.createNestedObject();
      t["order"]   = cfg.tracks[i].order;
      t["species"] = cfg.tracks[i].species;
      t["active"]  = cfg.tracks[i].active;
    }

    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.on("/ping", HTTP_GET, []() {
    server.send(200, "text/plain", "ok");
  });

  server.on("/status", HTTP_GET, []() {
    DateTime now = rtc.now();
    char rtcBuf[20];
    snprintf(rtcBuf, sizeof(rtcBuf), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());

    uint64_t sdFree = (SD.totalBytes() - SD.usedBytes()) / 1024;

    StaticJsonDocument<320> doc;
    doc["version"]    = FW_VERSION;
    doc["boardType"]  = BLE_BOARD_TYPE;
    doc["unitName"]   = cfg.unitName;
    doc["rtcTime"]    = rtcBuf;
    doc["sdFreeMB"]   = (int)(sdFree / 1024);
    doc["sessions"]   = cfg.totalSessions;
    doc["recordings"] = cfg.totalRecordings;

    doc["bmeOk"] = bmeDisponible;
    float t, h, p;
    if (leerBME280(t, h, p)) {
      doc["tempC"]   = t;
      doc["humPct"]  = h;
      doc["presHpa"] = p;
    }

    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.on("/save", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"Sin body\"}");
      return;
    }

    StaticJsonDocument<4096> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"JSON invalido\"}");
      return;
    }

    strlcpy(cfg.stationName, doc["stationName"] | cfg.stationName, sizeof(cfg.stationName));
    strlcpy(cfg.projectName, doc["projectName"] | cfg.projectName, sizeof(cfg.projectName));
    strlcpy(cfg.researcher,  doc["researcher"]  | cfg.researcher,  sizeof(cfg.researcher));
    strlcpy(cfg.unitName,    doc["unitName"]    | cfg.unitName,    sizeof(cfg.unitName));
    strlcpy(cfg.utmZone,     doc["utmZone"]     | cfg.utmZone,     sizeof(cfg.utmZone));
    cfg.utmEaste = doc["utmEaste"] | cfg.utmEaste;
    cfg.utmNorte = doc["utmNorte"] | cfg.utmNorte;

    auto parseTime = [](const char* s, int& h, int& m) {
      if (!s || strlen(s) < 5) return;
      h = atoi(s); m = atoi(s + 3);
    };
    parseTime(doc["morningStart"] | "", cfg.morningStartH, cfg.morningStartM);
    parseTime(doc["morningEnd"]   | "", cfg.morningEndH,   cfg.morningEndM);
    parseTime(doc["nightStart"]   | "", cfg.nightStartH,   cfg.nightStartM);
    parseTime(doc["nightEnd"]     | "", cfg.nightEndH,     cfg.nightEndM);

    cfg.recTimeSeg = doc["recTime"] | cfg.recTimeSeg;
    cfg.micSource  = doc["micSource"] | cfg.micSource;

    JsonArray tracks = doc["tracks"];
    if (doc.containsKey("trackCount")) {
      int tc = doc["trackCount"];
      if (tc >= 0 && tc <= MAX_TRACKS) cfg.trackCount = tc;
    }
    for (int i = 0; i < cfg.trackCount && i < (int)tracks.size(); i++) {
      cfg.tracks[i].order  = tracks[i]["order"]  | cfg.tracks[i].order;
      cfg.tracks[i].active = tracks[i]["active"]  | cfg.tracks[i].active;
      strlcpy(cfg.tracks[i].species, tracks[i]["species"] | cfg.tracks[i].species,
              sizeof(cfg.tracks[i].species));
    }

    const char* sysDate = doc["sysDate"];
    const char* sysTime = doc["sysTime"];
    if (sysDate && sysTime && strlen(sysDate) >= 10 && strlen(sysTime) >= 8) {
      int y  = atoi(sysDate);
      int mo = atoi(sysDate + 5);
      int d  = atoi(sysDate + 8);
      int h  = atoi(sysTime);
      int mi = atoi(sysTime + 3);
      int s  = atoi(sysTime + 6);
      rtc.adjust(DateTime(y, mo, d, h, mi, s));
      Serial.printf("[PORTAL] RTC ajustado a %04d-%02d-%02d %02d:%02d:%02d\n",
                    y, mo, d, h, mi, s);
    }

    bool shutdown = doc["shutdown"] | false;
    bool ok = guardarConfigSD();
    server.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"Error SD\"}");

    if (ok) {
      showLedsStatus(COLOR_VERDE, 2);
      if (shutdown) {
        Serial.println("[PORTAL] Config guardada. Cerrando portal en 5 s...");
        shutdownRequestedAt = millis();
        shutdownPending = true;
      } else {
        Serial.println("[PORTAL] Config guardada.");
      }
    } else {
      showLedsStatus(COLOR_ROJO, 3);
    }
  });

  server.on("/csv", HTTP_GET, []() {
    if (!SD.exists(CSV_PATH)) {
      server.send(404, "text/plain", "CSV no encontrado aun.");
      return;
    }
    File f = SD.open(CSV_PATH, FILE_READ);
    if (!f) { server.send(500, "text/plain", "Error al abrir CSV."); return; }

    server.sendHeader("Content-Disposition", "attachment; filename=\"sesiones.csv\"");
    server.streamFile(f, "text/csv");
    f.close();
  });

  server.on("/hotspot-detect.html", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send_P(200, "text/html; charset=utf-8", PORTAL_HTML);
  });
  server.on("/library/test/success.html", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send_P(200, "text/html; charset=utf-8", PORTAL_HTML);
  });
  server.on("/generate_204", HTTP_GET, []() {
    server.sendHeader("Location", "http://" AP_IP "/", true);
    server.sendHeader("Cache-Control", "no-cache");
    server.send(302, "text/plain", "");
  });
  server.on("/gen_204", HTTP_GET, []() {
    server.sendHeader("Location", "http://" AP_IP "/", true);
    server.sendHeader("Cache-Control", "no-cache");
    server.send(302, "text/plain", "");
  });
  server.on("/connecttest.txt", HTTP_GET, []() {
    server.sendHeader("Location", "http://" AP_IP "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/ncsi.txt", HTTP_GET, []() {
    server.sendHeader("Location", "http://" AP_IP "/", true);
    server.send(302, "text/plain", "");
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "http://" AP_IP "/", true);
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send(302, "text/plain", "");
  });
}

// ============================================================================
// PORTAL: LOOP DE ATENCIÓN
// ============================================================================
void manejarPortal() {
  unsigned long tInicio = millis();
  unsigned long timeout = (unsigned long)PORTAL_TIMEOUT_S * 1000UL;
  unsigned long tUltimoNotify = 0;

  startLedsCharging();  // reutilizamos la respiracion naranja como "modo configuracion"
  // NOTA: si a futuro agregas deteccion de carga real via BAT_STAT, esta
  // llamada debe moverse para no confundir "modo config" con "cargando".

  Serial.printf("[PORTAL] Esperando conexion (max %d s)...\n", PORTAL_TIMEOUT_S);

  while (true) {
    dnsServer.processNextRequest();
    server.handleClient();
    updateLeds();
    revisarBotonEstado();
    delay(5);

    int wifiClientes = WiFi.softAPgetStationNum();

    if ((wifiClientes > 0 || bleClienteConectado) && !clienteActivo) {
      clienteActivo = true;
      timerActivo = false;
      Serial.println("[PORTAL] Cliente conectado. Portal permanece abierto hasta que el usuario finalice.");
    }

    if (!clienteActivo) {
      if (millis() - tInicio >= timeout) {
        Serial.println("[PORTAL] Sin conexion. Cerrando portal y reanudando operacion.");
        break;
      }
    }

    if (bleClienteConectado && millis() - tUltimoNotify >= 2000) {
      actualizarStatusBLE();
      tUltimoNotify = millis();
    }

    if (shutdownPending && millis() - shutdownRequestedAt >= 5000) {
      shutdownPending = false;
      portalShutdown = true;
    }

    if (portalShutdown) break;
  }

  stopLeds();
  Serial.println("[PORTAL] Cerrando portal.");
}

// ============================================================================
// PORTAL: CERRAR AP Y APAGAR RF
// ============================================================================
void cerrarPortal() {
  server.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  detenerBLE();  // NimBLEDevice::deinit(true) ya libera el controlador BT.
  // NO llamar btStop() aca: es un segundo apagado del mismo controlador ya
  // liberado por deinit(true) - causaba un crash real en banco (PC=0x0,
  // salto a puntero nulo) el 12-sep.
  Serial.println("[PORTAL] AP, DNS, WiFi, BLE y BT apagados.");
}

// ============================================================================
// BLE: SERVICIO ZÉFIRO STRIX (sin cambios de protocolo respecto al DIY)
// ============================================================================
class ZefiroServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* srv, NimBLEConnInfo& info) override {
    bleClienteConectado = true;
    bleConnHandle = info.getConnHandle();
    Serial.println("[BLE] Cliente conectado.");
  }
  void onDisconnect(NimBLEServer* srv, NimBLEConnInfo& info, int reason) override {
    bleClienteConectado = false;
    Serial.println("[BLE] Cliente desconectado. Reanudando advertising...");
    NimBLEDevice::startAdvertising();
  }
};

class ConfigCharCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    c->setValue(buildConfigJsonBLE());
  }
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    bool ok = aplicarConfigJsonBLE(String(c->getValue().c_str()));
    if (ok) {
      guardarConfigSD();
      Serial.println("[BLE] Config recibida y guardada.");
      showLedsStatus(COLOR_VERDE, 2);
    } else {
      Serial.println("[BLE] Config invalida, ignorada.");
      showLedsStatus(COLOR_ROJO, 3);
    }
  }
};

class TracksPageSelectCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    String val = String(c->getValue().c_str());
    int page = val.toInt();
    if (page < 0) page = 0;
    bleTracksSelectedPage = page;
  }
};

class TracksDataCharCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    c->setValue(buildTracksPageJsonBLE(bleTracksSelectedPage));
  }
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    bool ok = aplicarTracksPageJsonBLE(String(c->getValue().c_str()));
    if (ok) {
      guardarConfigSD();
      Serial.println("[BLE] Pagina de tracks recibida y guardada.");
    } else {
      Serial.println("[BLE] Pagina de tracks invalida, ignorada.");
    }
  }
};

class TimeSyncCharCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, c->getValue().c_str())) return;

    const char* sysDate = doc["sysDate"];
    const char* sysTime = doc["sysTime"];
    if (sysDate && sysTime && strlen(sysDate) >= 10 && strlen(sysTime) >= 8) {
      int y  = atoi(sysDate);
      int mo = atoi(sysDate + 5);
      int d  = atoi(sysDate + 8);
      int h  = atoi(sysTime);
      int mi = atoi(sysTime + 3);
      int s  = atoi(sysTime + 6);
      rtc.adjust(DateTime(y, mo, d, h, mi, s));
      Serial.printf("[BLE] RTC ajustado a %04d-%02d-%02d %02d:%02d:%02d\n",
                    y, mo, d, h, mi, s);
    }
  }
};

class CommandCharCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    StaticJsonDocument<64> doc;
    if (deserializeJson(doc, c->getValue().c_str())) return;

    bool shutdown = doc["shutdown"] | false;
    if (shutdown) {
      Serial.println("[BLE] Comando shutdown recibido. Cerrando portal en 5 s...");
      shutdownRequestedAt = millis();
      shutdownPending = true;
    }
  }
};

String buildConfigJsonBLE() {
  StaticJsonDocument<2048> doc;
  doc["stationName"] = cfg.stationName;
  doc["projectName"] = cfg.projectName;
  doc["researcher"]  = cfg.researcher;
  doc["unitName"]    = cfg.unitName;
  doc["utmZone"]     = cfg.utmZone;
  doc["utmEaste"]    = cfg.utmEaste;
  doc["utmNorte"]    = cfg.utmNorte;

  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.morningStartH, cfg.morningStartM);
  doc["morningStart"] = buf;
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.morningEndH, cfg.morningEndM);
  doc["morningEnd"]   = buf;
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.nightStartH, cfg.nightStartM);
  doc["nightStart"]   = buf;
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.nightEndH, cfg.nightEndM);
  doc["nightEnd"]     = buf;

  doc["recTime"]    = cfg.recTimeSeg;
  doc["pauseMs"]    = cfg.pauseMs;
  doc["gainFactor"] = cfg.gainFactor;
  doc["micSource"]  = cfg.micSource;
  doc["boardType"]  = BLE_BOARD_TYPE;

  doc["trackCount"] = cfg.trackCount;
  doc["totalSessions"]   = cfg.totalSessions;
  doc["totalRecordings"] = cfg.totalRecordings;

  String out;
  serializeJson(doc, out);
  return out;
}

bool aplicarConfigJsonBLE(const String& body) {
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) return false;

  strlcpy(cfg.stationName, doc["stationName"] | cfg.stationName, sizeof(cfg.stationName));
  strlcpy(cfg.projectName, doc["projectName"] | cfg.projectName, sizeof(cfg.projectName));
  strlcpy(cfg.researcher,  doc["researcher"]  | cfg.researcher,  sizeof(cfg.researcher));
  strlcpy(cfg.unitName,    doc["unitName"]    | cfg.unitName,    sizeof(cfg.unitName));
  strlcpy(cfg.utmZone,     doc["utmZone"]     | cfg.utmZone,     sizeof(cfg.utmZone));
  cfg.utmEaste = doc["utmEaste"] | cfg.utmEaste;
  cfg.utmNorte = doc["utmNorte"] | cfg.utmNorte;

  auto parseTime = [](const char* s, int& h, int& m) {
    if (!s || strlen(s) < 5) return;
    h = atoi(s); m = atoi(s + 3);
  };
  parseTime(doc["morningStart"] | "", cfg.morningStartH, cfg.morningStartM);
  parseTime(doc["morningEnd"]   | "", cfg.morningEndH,   cfg.morningEndM);
  parseTime(doc["nightStart"]   | "", cfg.nightStartH,   cfg.nightStartM);
  parseTime(doc["nightEnd"]     | "", cfg.nightEndH,     cfg.nightEndM);

  cfg.recTimeSeg = doc["recTime"] | cfg.recTimeSeg;
  cfg.gainFactor = doc["gainFactor"] | cfg.gainFactor;
  cfg.micSource  = doc["micSource"]  | cfg.micSource;

  return true;
}

String buildTracksPageJsonBLE(int page) {
  StaticJsonDocument<1024> doc;
  int start = page * TRACKS_PAGE_SIZE;
  JsonArray arr = doc.createNestedArray("tracks");
  for (int i = start; i < start + TRACKS_PAGE_SIZE && i < cfg.trackCount; i++) {
    JsonObject t = arr.createNestedObject();
    t["order"]   = cfg.tracks[i].order;
    t["species"] = cfg.tracks[i].species;
    t["active"]  = cfg.tracks[i].active;
  }
  int totalPages = cfg.trackCount == 0 ? 1 : (cfg.trackCount + TRACKS_PAGE_SIZE - 1) / TRACKS_PAGE_SIZE;
  doc["page"] = page;
  doc["totalPages"] = totalPages;
  doc["totalTracks"] = cfg.trackCount;
  String out;
  serializeJson(doc, out);
  return out;
}

bool aplicarTracksPageJsonBLE(const String& body) {
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) return false;

  int page = doc["page"] | bleTracksSelectedPage;
  int start = page * TRACKS_PAGE_SIZE;
  JsonArray tracks = doc["tracks"];
  for (int i = 0; i < (int)tracks.size() && (start + i) < MAX_TRACKS; i++) {
    int idx = start + i;
    cfg.tracks[idx].order  = tracks[i]["order"]  | cfg.tracks[idx].order;
    cfg.tracks[idx].active = tracks[i]["active"] | cfg.tracks[idx].active;
    strlcpy(cfg.tracks[idx].species, tracks[i]["species"] | cfg.tracks[idx].species,
            sizeof(cfg.tracks[idx].species));
    if (idx + 1 > cfg.trackCount) cfg.trackCount = idx + 1;
  }
  return true;
}

void actualizarStatusBLE() {
  if (!bleStatusChar) return;
  DateTime now = rtc.now();
  StaticJsonDocument<512> doc;
  char rtcBuf[20];
  snprintf(rtcBuf, sizeof(rtcBuf), "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  uint64_t sdFree = (SD.totalBytes() - SD.usedBytes()) / 1024;
  doc["unitName"]   = cfg.unitName;
  doc["sdFreeMB"]   = (int)(sdFree / 1024);
  doc["rtcTime"]    = rtcBuf;
  doc["sessions"]   = cfg.totalSessions;
  doc["recordings"] = cfg.totalRecordings;
  doc["version"]    = FW_VERSION;
  doc["boardType"]  = BLE_BOARD_TYPE;

  // BME280 opcional - nombres de campo confirmados contra status_model.dart
  // de la app (tempC/humPct/presHpa/bmeOk). bmeOk siempre viaja; los otros
  // tres solo si el sensor esta disponible.
  doc["bmeOk"] = bmeDisponible;
  float t, h, p;
  if (leerBME280(t, h, p)) {
    doc["tempC"]   = t;
    doc["humPct"]  = h;
    doc["presHpa"] = p;
  }

  float vBatStatus = leerVoltajeBateria();
  int batPct = (int)(((vBatStatus - VBAT_CUTOFF_V) / (12.6f - VBAT_CUTOFF_V)) * 100.0f);
  if (batPct < 0) batPct = 0;
  if (batPct > 100) batPct = 100;

  doc["batPct"] = batPct;
  doc["batV"]   = vBatStatus;
  doc["batLow"] = batteryProtectionActive;

  String out;
  serializeJson(doc, out);
  bleStatusChar->setValue(out);
  bleStatusChar->notify();
}

void iniciarBLE() {
  String nombreAdv = String("ZefiroStrix-") + cfg.unitName;

  NimBLEDevice::init(nombreAdv.c_str());
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ZefiroServerCallbacks());

  NimBLEService* service = bleServer->createService(BLE_SERVICE_UUID);

  bleConfigChar = service->createCharacteristic(
    BLE_CONFIG_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  bleConfigChar->setCallbacks(new ConfigCharCallbacks());

  bleStatusChar = service->createCharacteristic(
    BLE_STATUS_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  bleTimeSyncChar = service->createCharacteristic(
    BLE_TIMESYNC_UUID, NIMBLE_PROPERTY::WRITE);
  bleTimeSyncChar->setCallbacks(new TimeSyncCharCallbacks());

  bleCommandChar = service->createCharacteristic(
    BLE_COMMAND_UUID, NIMBLE_PROPERTY::WRITE);
  bleCommandChar->setCallbacks(new CommandCharCallbacks());

  bleTracksPageChar = service->createCharacteristic(
    BLE_TRACKS_PAGE_UUID, NIMBLE_PROPERTY::WRITE);
  bleTracksPageChar->setCallbacks(new TracksPageSelectCallbacks());

  bleTracksDataChar = service->createCharacteristic(
    BLE_TRACKS_DATA_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  bleTracksDataChar->setCallbacks(new TracksDataCharCallbacks());

  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN);
  advData.addServiceUUID(BLE_SERVICE_UUID);

  NimBLEAdvertisementData scanData;
  scanData.setName(nombreAdv.c_str());

  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->enableScanResponse(true);
  adv->start();

  Serial.printf("[BLE] Advertising como \"%s\".\n", nombreAdv.c_str());
}

void detenerBLE() {
  if (!bleServer) return;
  NimBLEDevice::stopAdvertising();

  if (bleClienteConectado) {
    Serial.println("[BLE] Desconectando cliente antes de apagar el stack...");
    bleServer->disconnect(bleConnHandle);
    unsigned long t0 = millis();
    while (bleClienteConectado && millis() - t0 < 2000) {
      delay(10);
    }
  }

  NimBLEDevice::deinit(true);
  bleServer        = nullptr;
  bleConfigChar    = nullptr;
  bleStatusChar    = nullptr;
  bleTimeSyncChar  = nullptr;
  bleCommandChar   = nullptr;
  bleClienteConectado = false;
}

// ============================================================================
// DIAGNÓSTICO COMPLETO DE HARDWARE
// ============================================================================
bool diagnosticoSistema() {
  bool ok = true;
  Serial.println("--- DIAGNOSTICO DE HARDWARE ---\n");

  Serial.print("[RTC]       DS3231... ");
  if (rtc.begin()) {
    if (rtc.lostPower()) {
      // El RTC perdio su respaldo (sin pila, o pila agotada/recien puesta).
      // Se sincroniza con la hora de compilacion como valor de partida
      // razonable - en terreno la app la corrige con la hora real del celular.
      // IMPORTANTE: esto NO se ejecuta en cada arranque, solo cuando
      // lostPower() es verdadero, para no resetear la hora real cada vez
      // que el equipo despierta de deep sleep en operacion normal.
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      DateTime ahora = rtc.now();
      Serial.printf("OK (perdio energia, ajustado a hora de compilacion: %04d/%02d/%02d %02d:%02d:%02d)\n",
        ahora.year(), ahora.month(), ahora.day(), ahora.hour(), ahora.minute(), ahora.second());
    } else {
      DateTime now = rtc.now();
      Serial.printf("OK - %04d/%02d/%02d %02d:%02d:%02d\n",
        now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    }
  } else {
    Serial.println("FALLO - No responde. Revisa SDA/SCL y alimentacion.");
    ok = false;
  }

  Serial.print("[SD CARD3]  Principal (config/CSV/grabaciones)... ");
  if (montarSDPrincipal()) {
    uint64_t libre = (SD.totalBytes() - SD.usedBytes()) / (1024ULL * 1024);
    uint64_t total = SD.totalBytes() / (1024ULL * 1024);
    Serial.printf("OK - %llu MB libres de %llu MB\n", libre, total);
  } else {
    Serial.println("FALLO - No detectada. Revisa CS1/GPIO14, SPI y formato FAT32.");
    ok = false;
  }
  desmontarSD();

  Serial.print("[SD CARD4]  Playback (.wav)... ");
  if (montarSDPlayback()) {
    uint64_t libre = (SD.totalBytes() - SD.usedBytes()) / (1024ULL * 1024);
    uint64_t total = SD.totalBytes() / (1024ULL * 1024);
    Serial.printf("OK - %llu MB libres de %llu MB\n", libre, total);
  } else {
    Serial.println("FALLO - No detectada. Revisa CS2/GPIO1, SPI y formato FAT32.");
    ok = false;
  }
  desmontarSD();

  Serial.print("[MICROFONO] I2S... ");
  if (!diagnosticoMicrofono()) {
    Serial.println("FALLO - Sin senal. Revisa MICSW, bus I2S y micSource configurado.");
    ok = false;
  }

  Serial.print("[BME280]    Sensor ambiental (opcional)... ");
  if (iniciarBME280()) {
    float t, h, p;
    leerBME280(t, h, p);
    Serial.printf("OK - %.1f C, %.0f%% HR, %.0f hPa\n", t, h, p);
  } else {
    Serial.println("no detectado (no critico, el equipo sigue funcionando sin estos datos).");
  }

  Serial.println();
  Serial.println(ok ? "[DIAG] Todos los componentes OK." : "[DIAG] Fallos criticos detectados.");
  Serial.println("-------------------------------\n");
  return ok;
}

// ============================================================================
// DIAGNÓSTICO DE MICRÓFONO (I2S + RMS)
// ============================================================================
bool diagnosticoMicrofono() {
  seleccionarMicrofono(cfg.micSource);

  i2s_config_t cfg_i2s = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 8,
    .dma_buf_len          = 128
  };
  i2s_pin_config_t pins = { .bck_io_num = I2S_BCK, .ws_io_num = I2S_WS,
                             .data_out_num = -1, .data_in_num = I2S_DATA_IN };
  i2s_driver_install(I2S_NUM_0, &cfg_i2s, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  delay(200);

  int64_t suma = 0;
  int     n    = 0;
  bool    hay  = false;

  unsigned long t0 = millis();
  while (millis() - t0 < 1500 && n < SAMPLE_RATE) {
    int32_t buf[64]; size_t leidos;
    i2s_read(I2S_NUM_0, buf, sizeof(buf), &leidos, 100);
    for (int i = 0; i < (int)(leidos / 4) && n < SAMPLE_RATE; i++) {
      int32_t v = buf[i] >> shiftMicActual();
      suma += (int64_t)v * v;
      if (v != 0) hay = true;
      n++;
    }
  }
  i2s_driver_uninstall(I2S_NUM_0);

  if (!hay || n == 0) return false;

  double rms  = sqrt((double)suma / n);
  double dbfs = 20.0 * log10(rms / 32767.0 + 1e-9);
  Serial.printf("OK - RMS %.0f (%.1f dBFS)\n", rms, dbfs);
  if (dbfs < -60.0) Serial.println("           [ADVERTENCIA] Nivel muy bajo.");
  if (dbfs > -10.0) Serial.println("           [ADVERTENCIA] Nivel alto, considera bajar gainFactor.");
  return true;
}

// ============================================================================
// BME280 - SENSOR AMBIENTAL (OPCIONAL, no critico para el arranque)
// ============================================================================
// Comparte el mismo bus I2C que el RTC (SDA=GPIO19, SCL=GPIO20), via el
// conector externo U20. Prueba ambas direcciones posibles (0x76 y 0x77,
// segun el estado del pin SDO del modulo) automaticamente.
bool iniciarBME280() {
  if (bme.begin(0x76)) { bmeDisponible = true; return true; }
  if (bme.begin(0x77)) { bmeDisponible = true; return true; }
  bmeDisponible = false;
  return false;
}

// Devuelve false si el sensor no esta disponible (deja los parametros sin
// tocar en ese caso - revisar bmeDisponible o el valor de retorno antes de
// usar los datos).
bool leerBME280(float& temperaturaC, float& humedadPct, float& presionHpa) {
  if (!bmeDisponible) return false;
  temperaturaC = bme.readTemperature();
  humedadPct   = bme.readHumidity();
  presionHpa   = bme.readPressure() / 100.0f;  // Pa -> hPa
  return true;
}

// ============================================================================
// HELPERS DE HORARIO
// ============================================================================
bool dentroDeRango(int t, int ini, int fin) {
  if (ini < fin) return t >= ini && t < fin;
  return t >= ini || t < fin;
}

bool esHoraActiva(int hora, int minuto) {
  int t      = hora * 60 + minuto;
  int iniMa  = cfg.morningStartH * 60 + cfg.morningStartM;
  int finMa  = cfg.morningEndH   * 60 + cfg.morningEndM;
  int iniNo  = cfg.nightStartH   * 60 + cfg.nightStartM;
  int finNo  = cfg.nightEndH     * 60 + cfg.nightEndM;
  return dentroDeRango(t, iniMa, finMa) || dentroDeRango(t, iniNo, finNo);
}

// ============================================================================
// CICLO PRINCIPAL
// ============================================================================
void ejecutarCiclo() {
  montarSDPrincipal();
  int orden[MAX_TRACKS];
  int nActivos = 0;
  for (int o = 1; o <= cfg.trackCount; o++) {
    for (int i = 0; i < cfg.trackCount; i++) {
      if (cfg.tracks[i].active && cfg.tracks[i].order == o) {
        orden[nActivos++] = i;
        break;
      }
    }
  }
  desmontarSD();
  if (nActivos == 0) { Serial.println("[CICLO] No hay tracks activos."); return; }

  int pos = 0;

  while (true) {
    int idxFisico   = orden[pos];
    int trackFisico = idxFisico + 1;

    montarSDPrincipal();
    DateTime now  = rtc.now();
    desmontarSD();
    bool activo   = esHoraActiva(now.hour(), now.minute());
    if (!activo && pos == 0) {
      Serial.println("[CICLO] Horario termino antes de iniciar. Saliendo.");
      break;
    }

    reproducirYGrabar(trackFisico, idxFisico);

    pos++;
    if (pos >= nActivos) pos = 0;

    montarSDPrincipal();
    now    = rtc.now();
    desmontarSD();
    activo = esHoraActiva(now.hour(), now.minute());
    if (!activo) {
      Serial.println("[CICLO] Horario terminado. Saliendo del ciclo.");
      break;
    }
  }
}

// ============================================================================
// REPRODUCIR UN TRACK (CARD4) Y GRABAR LA RESPUESTA (CARD3)
// ============================================================================
void reproducirYGrabar(int trackFisico, int idxFisico) {
  montarSDPrincipal();
  Serial.printf("\n[T%d] %s\n", trackFisico, cfg.tracks[idxFisico].species);
  desmontarSD();

  char nombrePista[24];
  snprintf(nombrePista, sizeof(nombrePista), "/T%d.wav", trackFisico);

  setLedsRecording(true);  // por si acaso, tambien durante playback: ningun LED debe encenderse

  montarSDPlayback();
  bool existePista = SD.exists(nombrePista);
  bool ok = existePista ? reproducirTrackWav(nombrePista) : false;
  desmontarSD();

  if (!existePista) {
    // No se encontro el archivo del track: se salta por completo (no tiene
    // sentido grabar una "respuesta" a un playback que nunca sono).
    Serial.printf("[T%d] SALTADO: no existe %s en CARD4.\n", trackFisico, nombrePista);
    setLedsRecording(false);
    delay(cfg.pauseMs);
    return;
  }
  if (!ok) {
    Serial.printf("[T%d] ADVERTENCIA: %s existe pero fallo la reproduccion (header WAV invalido?)\n",
                  trackFisico, nombrePista);
  }

  Serial.printf("[T%d] Reproduccion finalizada. Silenciando (400ms)...\n", trackFisico);
  delay(400);

  grabarRespuesta(trackFisico, idxFisico);
  setLedsRecording(false);

  delay(cfg.pauseMs);
}

// ============================================================================
// PLAYBACK: LEER .WAV DE CARD4 Y ENVIAR POR I2S
// ============================================================================
bool reproducirTrackWav(const char* filePath) {
  if (!SD.exists(filePath)) return false;
  File f = SD.open(filePath, FILE_READ);
  if (!f) return false;

  uint32_t sampleRate; uint16_t bits, canales;
  if (!leerWavHeader(f, sampleRate, bits, canales)) {
    Serial.println("[PLAYBACK] Header WAV invalido.");
    f.close();
    return false;
  }
  // NOTA: si el .wav no esta a SAMPLE_RATE/16-bit/mono, este reproductor
  // simple NO hace resample - se reproducira a velocidad/tono incorrectos.
  // Exporta las pistas ya en el formato correcto (8kHz, 16-bit, mono) para
  // evitar este problema.

  float gananciaDigital = constrain((float)PLAYBACK_GAIN_PCT / 20.0f, 0.0f, 1.5f);

  // static: evita que estos ~12KB vivan en el stack (causaba stack
  // overflow real y crash en banco - ver hallazgo del 12-sep). Con
  // "static" pasan a memoria fija (BSS), no al stack de la tarea.
  static int16_t buf16[CHUNK_SAMPLES];
  static int32_t buf32[CHUNK_SAMPLES];

  while (f.available()) {
    int leidos16 = f.read((uint8_t*)buf16, sizeof(buf16)) / 2;
    if (leidos16 <= 0) break;

    for (int i = 0; i < leidos16; i++) {
      int32_t muestra = (int32_t)(buf16[i] * gananciaDigital);
      if (muestra > 32767) muestra = 32767;
      if (muestra < -32768) muestra = -32768;
      buf32[i] = ((int32_t)muestra) << 16;  // alinear MSB-first a formato 32-bit del bus I2S (PCM5102A, I2S_COMM_FORMAT_STAND_I2S)
    }
    size_t escritos;
    i2s_write(I2S_NUM_0, buf32, leidos16 * 4, &escritos, portMAX_DELAY);
  }

  f.close();
  return true;
}

// ============================================================================
// GRABACIÓN WAV (CARD3) DESDE MICROFONO I2S
// ============================================================================
void grabarRespuesta(int trackFisico, int idxFisico) {
  montarSDPrincipal();
  DateTime now = rtc.now();

  char dirPath[36];
  sprintf(dirPath, "%s/%04d-%02d-%02d", REC_DIR_ROOT, now.year(), now.month(), now.day());
  crearDirectorio(REC_DIR_ROOT);
  crearDirectorio(dirPath);

  char especieCorta[16];
  strlcpy(especieCorta, cfg.tracks[idxFisico].species, sizeof(especieCorta));
  for (int k = 0; k < (int)strlen(especieCorta); k++) {
    if (especieCorta[k] == ' ' || especieCorta[k] == '(' || especieCorta[k] == ')') {
      especieCorta[k] = '_';
    }
  }

  char filePath[96];
  sprintf(filePath, "%s/%04d%02d%02d_%02d%02d%02d_T%d_%s.wav",
          dirPath,
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second(),
          trackFisico, especieCorta);

  Serial.printf("[T%d] Grabando -> %s\n", trackFisico, filePath);

  File f = SD.open(filePath, FILE_WRITE);
  if (!f) {
    Serial.printf("[T%d] ERROR: no se pudo crear el WAV.\n", trackFisico);
    desmontarSD();
    return;
  }

  writeWavHeader(f);

  uint32_t      dataSize = 0;
  unsigned long tIni     = millis();
  // static por el mismo motivo que en reproducirTrackWav(): evitar
  // stack overflow con buffers grandes en una funcion que se llama en
  // el loop principal.
  static int16_t ramBuf[CHUNK_SAMPLES];
  int           bufIdx   = 0;
  int64_t       sumaRMS  = 0;
  long          nMuestras = 0;
  bool          sdSinEspacio = false;

  while (millis() - tIni < (unsigned long)(cfg.recTimeSeg * 1000) && !sdSinEspacio) {
    int32_t i2sBuf[128]; size_t leidos;
    i2s_read(I2S_NUM_0, i2sBuf, sizeof(i2sBuf), &leidos, portMAX_DELAY);

    int n = leidos / 4;
    for (int i = 0; i < n; i++) {
      int32_t val = (i2sBuf[i] >> shiftMicActual()) * cfg.gainFactor;
      if (val >  32767) val =  32767;
      if (val < -32768) val = -32768;

      ramBuf[bufIdx++] = (int16_t)val;
      sumaRMS += (int64_t)val * val;
      nMuestras++;

      if (bufIdx >= CHUNK_SAMPLES) {
        size_t escritos = f.write((const uint8_t*)ramBuf, CHUNK_SAMPLES * 2);
        dataSize += escritos;
        if (escritos < (size_t)(CHUNK_SAMPLES * 2)) {
          // Escritura incompleta: lo mas probable es que la SD se quedo sin
          // espacio. Se corta la grabacion aqui en vez de seguir generando
          // un WAV corrupto/truncado sin avisar.
          Serial.printf("[T%d] ERROR: escritura SD incompleta (sin espacio?). Grabacion cortada.\n",
                        trackFisico);
          sdSinEspacio = true;
          break;
        }
        bufIdx = 0;
      }
    }
  }
  if (bufIdx > 0 && !sdSinEspacio) {
    size_t escritos = f.write((const uint8_t*)ramBuf, bufIdx * 2);
    dataSize += escritos;
    if (escritos < (size_t)bufIdx * 2) {
      Serial.printf("[T%d] ERROR: escritura SD incompleta en el cierre (sin espacio?).\n", trackFisico);
    }
  }

  sellarWav(f, dataSize);
  f.close();

  if (sdSinEspacio) {
    Serial.printf("[T%d] ADVERTENCIA: CARD3 sin espacio libre suficiente. Revisa y libera espacio pronto.\n",
                  trackFisico);
  }

  float rms    = nMuestras > 0 ? sqrt((double)sumaRMS / nMuestras) : 0;
  float durSeg = (float)dataSize / (SAMPLE_RATE * 2);
  float tamKB  = (float)(dataSize + 44) / 1024.0f;

  Serial.printf("[T%d] OK - %.1fs, %.1fKB, RMS=%.0f\n",
                trackFisico, durSeg, tamKB, rms);

  cfg.totalRecordings++;
  registrarCSV(filePath, trackFisico, rms, durSeg);
  desmontarSD();
}

// ============================================================================
// LOG CSV (CARD3)
// ============================================================================
void registrarCSV(const char* archivo, int trackNum, float rms, float durSeg) {
  bool nuevo = !SD.exists(CSV_PATH);
  File f = SD.open(CSV_PATH, FILE_APPEND);
  if (!f) return;

  if (nuevo) {
    f.println("fecha,hora,unidad,estacion,proyecto,investigador,"
              "utm_zona,utm_este,utm_norte,"
              "track,especie,archivo,duracion_seg,rms,dbfs,"
              "gain_factor,mic_source,temp_c,hum_pct,pres_hpa,fw_version");
  }

  DateTime now = rtc.now();

  double dbfs = 20.0 * log10((double)rms / 32767.0 + 1e-9);
  const char* micSourceTxt = (cfg.micSource == 0) ? "digital" : "analogico";

  float bmeT, bmeH, bmeP;
  bool bmeLeido = leerBME280(bmeT, bmeH, bmeP);
  char campoTemp[12], campoHum[12], campoPres[12];
  if (bmeLeido) {
    snprintf(campoTemp, sizeof(campoTemp), "%.1f", bmeT);
    snprintf(campoHum,  sizeof(campoHum),  "%.0f", bmeH);
    snprintf(campoPres, sizeof(campoPres), "%.0f", bmeP);
  } else {
    campoTemp[0] = campoHum[0] = campoPres[0] = '\0';
  }

  char linea[400];
  snprintf(linea, sizeof(linea),
    "%04d-%02d-%02d,%02d:%02d:%02d,"
    "%s,%s,%s,%s,"
    "%s,%ld,%ld,"
    "%d,%s,%s,%.1f,%.0f,%.1f,"
    "%d,%s,%s,%s,%s,%s",
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second(),
    cfg.unitName, cfg.stationName, cfg.projectName, cfg.researcher,
    cfg.utmZone, cfg.utmEaste, cfg.utmNorte,
    trackNum, cfg.tracks[trackNum - 1].species,
    archivo, durSeg, rms, dbfs,
    cfg.gainFactor, micSourceTxt, campoTemp, campoHum, campoPres, FW_VERSION);

  f.println(linea);
  f.close();
}

// ============================================================================
// HELPERS DE ARCHIVO
// ============================================================================
bool crearDirectorio(const char* path) {
  if (SD.exists(path)) return true;
  if (SD.mkdir(path))  return true;
  Serial.printf("[SD] Error creando directorio: %s\n", path);
  return false;
}

// ============================================================================
// WAV: HEADER ESTÁNDAR 44 BYTES (grabación)
// ============================================================================
void writeWavHeader(File& f) {
  const uint32_t sampleRate  = SAMPLE_RATE;
  const uint32_t byteRate    = SAMPLE_RATE * 1 * (BITS_PER_SAMPLE / 8);
  const uint16_t blockAlign  = 1 * (BITS_PER_SAMPLE / 8);
  const uint16_t bitsPerSamp = BITS_PER_SAMPLE;
  const uint16_t numChannels = 1;
  const uint16_t audioFmt    = 1;
  const uint32_t zero        = 0;
  const uint32_t fmtSize     = 16;

  f.write((uint8_t*)"RIFF",      4);
  f.write((uint8_t*)&zero,       4);
  f.write((uint8_t*)"WAVE",      4);
  f.write((uint8_t*)"fmt ",      4);
  f.write((uint8_t*)&fmtSize,    4);
  f.write((uint8_t*)&audioFmt,   2);
  f.write((uint8_t*)&numChannels,2);
  f.write((uint8_t*)&sampleRate, 4);
  f.write((uint8_t*)&byteRate,   4);
  f.write((uint8_t*)&blockAlign, 2);
  f.write((uint8_t*)&bitsPerSamp,2);
  f.write((uint8_t*)"data",      4);
  f.write((uint8_t*)&zero,       4);
}

void sellarWav(File& f, uint32_t dataSize) {
  uint32_t riffSize = dataSize + 36;
  f.seek(4);  f.write((uint8_t*)&riffSize, 4);
  f.seek(40); f.write((uint8_t*)&dataSize, 4);
}

// ============================================================================
// WAV: LEER HEADER (playback) - parser simple de 44 bytes estandar
// ============================================================================
bool leerWavHeader(File& f, uint32_t& sampleRate, uint16_t& bits, uint16_t& canales) {
  uint8_t hdr[44];
  if (f.read(hdr, 44) != 44) return false;
  if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) return false;

  memcpy(&canales,    hdr + 22, 2);
  memcpy(&sampleRate, hdr + 24, 4);
  memcpy(&bits,       hdr + 34, 2);
  return true;
}
