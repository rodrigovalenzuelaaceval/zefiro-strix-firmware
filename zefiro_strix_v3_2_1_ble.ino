/*
  ============================================================================
  ZÉFIRO STRIX V3.0.0 — FIRMWARE DE MONITOREO DE AVES NOCTURNAS
  ============================================================================
  Dispositivo: ESP32 DevModule + DFPlayer Mini + PAM8610 + INMP441 + DS3231

  NUEVO EN V3.0.0:
  - Portal de configuración web vía Access Point WiFi
  - Ventana de 60 s al encender: si nadie se conecta, cierra la red y opera
  - Configuración persistente en /config.json (MicroSD)
  - Log de sesiones en /sesiones.csv (MicroSD), descargable desde el portal
  - Parámetros configurables: horarios, tracks, especies, ganancia, volumen,
    duración de grabación, pausa entre tracks, coords UTM, proyecto, unidad
  - Hora del sistema sincronizable desde el portal (sin USB)
  - Endpoint /status para dashboard en tiempo real
  - Endpoint /csv para descarga directa del CSV desde el teléfono
  - Conversión GPS → UTM realizada en el navegador del teléfono (JS)

  HARDWARE:
  - ESP32 Dev Module
  - DFPlayer Mini → Amplificador PAM8610 → Parlante 3" 20W 4Ω
  - Micrófono INMP441 (I2S, bus dedicado)
  - RTC DS3231 (I2C)
  - MicroSD en lector SPI (grabaciones WAV + config + CSV)
  - 2× 18650 en serie (8.4V) → Mini360 5V → sistema
  - PAM8610 alimentado directo 8.4V desde 18650
  - INMP441 alimentado desde pin 3.3V del ESP32

  ARCHIVOS EN SD:
  - /config.json          → configuración activa
  - /sesiones.csv         → log georreferenciado de todas las grabaciones
  - /REC/YYYY-MM-DD/      → subcarpetas por fecha
    └ YYYYMMDD_HHMMSS_T<n>_<especie>.wav

  PINES:
  - SD_CS  = 5   I2S_WS = 33   I2S_SD = 35   I2S_SCK = 32
  - DF_RX  = 16  DF_TX  = 17   DF_BUSY = 4
  ============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <SPI.h>
#include <SD.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <sys/time.h>

// DFRobotDFPlayerMini.h define "#define Advertise 7" como una de sus
// constantes internas de comando. Esa macro choca con el enum
// NimBLETxPowerType de NimBLE-Arduino, que tiene un valor llamado
// literalmente "Advertise". Se anula aquí, justo antes de incluir NimBLE,
// para que el preprocesador no reemplace ese nombre dentro del enum.
#undef Advertise
// NOTA: el límite de tamaño de característica BLE (BLE_ATT_ATTR_MAX_LEN) se edita
// directamente en la librería instalada: NimBLEAttValue.h, línea ~44 (512 → 2048).
// No se define aquí porque el sketch y la librería se compilan como unidades
// separadas y el macro no se propaga. Cualquiera que compile este proyecto en otro
// computador debe aplicar ese mismo cambio manualmente en su copia de la librería.
#include <NimBLEDevice.h>
#include "portal.h"

// ============================================================================
// PINES
// ============================================================================
#define SD_CS     5
#define I2S_WS    33
#define I2S_SD    35
#define I2S_SCK   32
#define DF_RX     16
#define DF_TX     17
#define DF_BUSY   4

// ============================================================================
// CONSTANTES DE FIRMWARE
// ============================================================================
#define FW_VERSION        "3.2.1"
#define AP_SSID           "ZefiroStrix-Config"
#define AP_PASS           ""                   // Red abierta — sin contraseña
#define AP_IP             "192.168.4.1"
#define PORTAL_TIMEOUT_S  60                   // Segundos antes de cerrar el AP
#define CONFIG_PATH       "/config.json"
#define CSV_PATH          "/sesiones.csv"

// ── Protocolo BLE Zéfiro Strix v1 (ver docs/zefiro_ble_protocol_v1.md) ─────
// boardType para esta variante de hardware (ver sección 4 del protocolo)
#define BLE_BOARD_TYPE       "diy-wired"
#define BLE_SERVICE_UUID     "4d617b4f-4320-4e1b-b6c0-1e6a52a81ba9"
#define BLE_CONFIG_UUID      "770440e9-947e-4983-a405-3fdd67dd43db"
#define BLE_STATUS_UUID      "babdcdd4-83aa-45da-9444-1737d5ff6a2e"
#define BLE_TIMESYNC_UUID    "398eaab7-1b17-4529-ab0d-d2ccedce80fe"
#define BLE_COMMAND_UUID     "62b3db56-e022-4efc-a2e7-af19c4f69a3f"

// Credenciales NTP (solo si el usuario elige sincronizar por WiFi externo)
const char* NTP_SSID = "CHUCAO2-2.4";
const char* NTP_PASS = "ANANUKA8";

// ============================================================================
// CONSTANTES DE AUDIO
// ============================================================================
#define SAMPLE_RATE     8000
#define BITS_PER_SAMPLE 16
#define CHUNK_SAMPLES   2048

// ============================================================================
// ESTRUCTURA DE CONFIGURACIÓN
// ============================================================================
struct TrackConfig {
  int    order;
  char   species[48];
  bool   active;
};

struct Config {
  // Identidad
  char stationName[48];
  char projectName[48];
  char researcher[48];
  char unitName[16];
  // Georreferencia
  char utmZone[6];
  long utmEaste;
  long utmNorte;
  // Horarios
  int  morningStartH, morningStartM;
  int  morningEndH,   morningEndM;
  int  nightStartH,   nightStartM;
  int  nightEndH,     nightEndM;
  // Audio
  int  recTimeSeg;
  int  pauseMs;
  int  volume;
  int  gainFactor;
  // Tracks
  TrackConfig tracks[7];
  // Estadísticas
  long totalSessions;
  long totalRecordings;
};

// ============================================================================
// OBJETOS GLOBALES
// ============================================================================
RTC_DS3231           rtc;
HardwareSerial       dfSerial(2);
DFRobotDFPlayerMini  dfPlayer;
WebServer            server(80);
DNSServer            dnsServer;
bool                 portalShutdown = false;
bool                 timerActivo    = true;
Config               cfg;

// ── Estado BLE ──────────────────────────────────────────────────────────
NimBLEServer*        bleServer          = nullptr;
NimBLECharacteristic* bleConfigChar     = nullptr;
NimBLECharacteristic* bleStatusChar     = nullptr;
NimBLECharacteristic* bleTimeSyncChar   = nullptr;
NimBLECharacteristic* bleCommandChar    = nullptr;
volatile bool         bleClienteConectado = false;
bool                  clienteActivo       = false;  // true si hay cliente WiFi o BLE

// ============================================================================
// PROTOTIPOS
// ============================================================================
void     cargarConfigDefecto();
bool     cargarConfigSD();
bool     guardarConfigSD();
void     iniciarPortal();
void     manejarPortal();
void     setupEndpoints();
void     handleCaptivePortal();
void     cerrarPortal();
bool     diagnosticoSistema();
bool     diagnosticoMicrofono();
void     sincronizarHoraNTP();
bool     esHoraActiva(int hora, int minuto);
bool     dentroDeRango(int t, int ini, int fin);
void     ejecutarCiclo();
void     reproducirYGrabar(int trackPhysical, int trackLogical);
void     grabarRespuesta(int trackPhysical, int trackLogical);
void     registrarCSV(const char* archivo, int trackNum, float rms, float durSeg);
bool     crearDirectorio(const char* path);
void     initI2S();
void     writeWavHeader(File& f);
void     sellarWav(File& f, uint32_t dataSize);
void     iniciarBLE();
void     detenerBLE();
void     actualizarStatusBLE();
String   buildConfigJsonBLE();
bool     aplicarConfigJsonBLE(const String& body);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n============================================");
  Serial.println("  ZEFIRO STRIX V" FW_VERSION);
  Serial.println("  Monitor acústico de aves nocturnas");
  Serial.println("============================================\n");

  // Apagar WiFi y BT de inmediato
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.println("[BOOT] RF apagada.");

  pinMode(DF_BUSY, INPUT_PULLUP);

  // Diagnóstico de hardware (detiene el sistema si falla algo crítico)
  if (!diagnosticoSistema()) {
    Serial.println("\n[CRITICO] Sistema detenido. Corrige los fallos y reinicia.");
    while (true) delay(1000);
  }

  // Cargar configuración desde SD (o usar valores por defecto)
  if (!cargarConfigSD()) {
    Serial.println("[CONFIG] Usando configuración por defecto.");
    cargarConfigDefecto();
    guardarConfigSD();
  }

  // Portal de configuración (ventana de PORTAL_TIMEOUT_S segundos)
  iniciarPortal();
  iniciarBLE();      // Servicio BLE en paralelo al portal WiFi (ver protocolo)
  manejarPortal();   // Bloquea hasta timeout o hasta que el usuario guarda
  cerrarPortal();

  // Sincronizar reloj interno del ESP32 con el RTC
  DateTime now = rtc.now();
  struct timeval tv = { .tv_sec = (time_t)now.unixtime(), .tv_usec = 0 };
  settimeofday(&tv, NULL);

  // Inicializar I2S para grabación
  initI2S();

  Serial.println("\n[BOOT] Sistema listo. Iniciando monitoreo...\n");
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  DateTime now   = rtc.now();
  int hora       = now.hour();
  int minuto     = now.minute();

  if (esHoraActiva(hora, minuto)) {
    Serial.printf("\n[CICLO] Horario activo (%02d:%02d). Iniciando secuencia...\n",
                  hora, minuto);
    ejecutarCiclo();
    cfg.totalSessions++;
    guardarConfigSD();
    Serial.println("[CICLO] Completado. Deep Sleep 5 min...");
  } else {
    Serial.printf("[SLEEP] Fuera de horario (%02d:%02d). Durmiendo 5 min...\n",
                  hora, minuto);
  }

  esp_sleep_enable_timer_wakeup(5ULL * 60 * 1000000ULL);
  esp_deep_sleep_start();
}

// ============================================================================
// CONFIGURACIÓN: VALORES POR DEFECTO
// ============================================================================
void cargarConfigDefecto() {
  strlcpy(cfg.stationName, "Sin nombre",        sizeof(cfg.stationName));
  strlcpy(cfg.projectName, "Proyecto",          sizeof(cfg.projectName));
  strlcpy(cfg.researcher,  "Investigador",      sizeof(cfg.researcher));
  strlcpy(cfg.unitName,    "ZS-01",             sizeof(cfg.unitName));
  strlcpy(cfg.utmZone,     "19S",               sizeof(cfg.utmZone));
  cfg.utmEaste = 0;
  cfg.utmNorte = 0;

  cfg.morningStartH = 6;  cfg.morningStartM = 22;
  cfg.morningEndH   = 7;  cfg.morningEndM   = 22;
  cfg.nightStartH   = 18; cfg.nightStartM   = 55;
  cfg.nightEndH     = 19; cfg.nightEndM     = 55;

  cfg.recTimeSeg  = 20;
  cfg.pauseMs     = 500;
  cfg.volume      = 30;
  cfg.gainFactor  = 3;

  const char* especiesDefault[7] = {
    "Chuncho (Glaucidium nanum)",
    "Concon (Strix rufipes)",
    "Lechuza (Tyto alba)",
    "Tucuquere (Bubo magellanicus)",
    "Nuco (Asio flammeus)",
    "Especie 6",
    "Especie 7"
  };
  for (int i = 0; i < 7; i++) {
    cfg.tracks[i].order  = i + 1;
    cfg.tracks[i].active = true;
    strlcpy(cfg.tracks[i].species, especiesDefault[i], sizeof(cfg.tracks[i].species));
  }

  cfg.totalSessions   = 0;
  cfg.totalRecordings = 0;
}

// ============================================================================
// CONFIGURACIÓN: LEER DESDE SD
// ============================================================================
bool cargarConfigSD() {
  if (!SD.exists(CONFIG_PATH)) return false;

  File f = SD.open(CONFIG_PATH, FILE_READ);
  if (!f) return false;

  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { Serial.printf("[CONFIG] JSON inválido: %s\n", err.c_str()); return false; }

  strlcpy(cfg.stationName, doc["stationName"] | "Sin nombre",   sizeof(cfg.stationName));
  strlcpy(cfg.projectName, doc["projectName"] | "Proyecto",     sizeof(cfg.projectName));
  strlcpy(cfg.researcher,  doc["researcher"]  | "Investigador", sizeof(cfg.researcher));
  strlcpy(cfg.unitName,    doc["unitName"]    | "ZS-01",        sizeof(cfg.unitName));
  strlcpy(cfg.utmZone,     doc["utmZone"]     | "19S",          sizeof(cfg.utmZone));
  cfg.utmEaste = doc["utmEaste"] | 0;
  cfg.utmNorte = doc["utmNorte"] | 0;

  // Parsear horarios desde strings "HH:MM"
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
  cfg.volume      = doc["volume"]     | 30;
  cfg.gainFactor  = doc["gainFactor"] | 3;

  JsonArray tracks = doc["tracks"];
  for (int i = 0; i < 7 && i < (int)tracks.size(); i++) {
    cfg.tracks[i].order  = tracks[i]["order"]  | (i + 1);
    cfg.tracks[i].active = tracks[i]["active"] | true;
    strlcpy(cfg.tracks[i].species, tracks[i]["species"] | "Especie", sizeof(cfg.tracks[i].species));
  }

  cfg.totalSessions   = doc["totalSessions"]   | 0;
  cfg.totalRecordings = doc["totalRecordings"] | 0;

  Serial.println("[CONFIG] Configuración cargada desde SD.");
  return true;
}

// ============================================================================
// CONFIGURACIÓN: ESCRIBIR EN SD
// ============================================================================
bool guardarConfigSD() {
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
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.morningEndH,   cfg.morningEndM);
  doc["morningEnd"]   = buf;
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.nightStartH,   cfg.nightStartM);
  doc["nightStart"]   = buf;
  snprintf(buf, sizeof(buf), "%02d:%02d", cfg.nightEndH,     cfg.nightEndM);
  doc["nightEnd"]     = buf;

  doc["recTime"]    = cfg.recTimeSeg;
  doc["pauseMs"]    = cfg.pauseMs;
  doc["volume"]     = cfg.volume;
  doc["gainFactor"] = cfg.gainFactor;

  JsonArray tracks = doc.createNestedArray("tracks");
  for (int i = 0; i < 7; i++) {
    JsonObject t  = tracks.createNestedObject();
    t["order"]    = cfg.tracks[i].order;
    t["species"]  = cfg.tracks[i].species;
    t["active"]   = cfg.tracks[i].active;
  }

  doc["totalSessions"]   = cfg.totalSessions;
  doc["totalRecordings"] = cfg.totalRecordings;

  if (SD.exists(CONFIG_PATH)) SD.remove(CONFIG_PATH);
  File f = SD.open(CONFIG_PATH, FILE_WRITE);
  if (!f) { Serial.println("[CONFIG] Error al escribir config.json"); return false; }
  serializeJson(doc, f);
  f.close();
  return true;
}

// ============================================================================
// PORTAL: INICIAR ACCESS POINT
// ============================================================================
void iniciarPortal() {
  Serial.println("[PORTAL] Iniciando Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  IPAddress ip;
  ip.fromString(AP_IP);
  IPAddress gw(192, 168, 4, 1);
  IPAddress nm(255, 255, 255, 0);
  WiFi.softAPConfig(ip, gw, nm);

  delay(500);
  Serial.printf("[PORTAL] Red: %s — IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.printf("[PORTAL] Ventana de %d segundos para conectarse.\n", PORTAL_TIMEOUT_S);

  // Servidor DNS: resuelve CUALQUIER dominio a la IP del ESP32.
  // Sin esto, el teléfono no puede resolver ninguna URL y no abre el portal.
  dnsServer.setTTL(300);
  dnsServer.start(53, "*", ip);
  Serial.println("[PORTAL] DNS server activo (wildcard → 192.168.4.1).");

  setupEndpoints();
  server.begin();
}

// ============================================================================
// PORTAL: ENDPOINTS HTTP
// ============================================================================
void setupEndpoints() {

  // ── Página principal ────────────────────────────────────────────────────
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=utf-8", PORTAL_HTML);
  });

  // ── Configuración actual como JSON ─────────────────────────────────────
  server.on("/config.json", HTTP_GET, []() {
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
    doc["morningEnd"] = buf;
    snprintf(buf, sizeof(buf), "%02d:%02d", cfg.nightStartH, cfg.nightStartM);
    doc["nightStart"] = buf;
    snprintf(buf, sizeof(buf), "%02d:%02d", cfg.nightEndH, cfg.nightEndM);
    doc["nightEnd"] = buf;

    doc["recTime"]    = cfg.recTimeSeg;
    doc["pauseMs"]    = cfg.pauseMs;
    doc["volume"]     = cfg.volume;
    doc["gainFactor"] = cfg.gainFactor;

    JsonArray tracks = doc.createNestedArray("tracks");
    for (int i = 0; i < 7; i++) {
      JsonObject t = tracks.createNestedObject();
      t["order"]   = cfg.tracks[i].order;
      t["species"] = cfg.tracks[i].species;
      t["active"]  = cfg.tracks[i].active;
    }

    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  // ── Ping: señal de que hay un cliente activo → reinicia timeout ──────────
  server.on("/ping", HTTP_GET, []() {
    server.send(200, "text/plain", "ok");
  });

  // ── Estado del dispositivo ──────────────────────────────────────────────
  server.on("/status", HTTP_GET, []() {
    DateTime now = rtc.now();
    char rtcBuf[20];
    snprintf(rtcBuf, sizeof(rtcBuf), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());

    uint64_t sdFree = (SD.totalBytes() - SD.usedBytes()) / 1024;

    StaticJsonDocument<256> doc;
    doc["version"]    = FW_VERSION;
    doc["unitName"]   = cfg.unitName;
    doc["rtcTime"]    = rtcBuf;
    doc["sdFreeMB"]   = (int)(sdFree / 1024);
    doc["sessions"]   = cfg.totalSessions;
    doc["recordings"] = cfg.totalRecordings;

    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  // ── Guardar configuración ───────────────────────────────────────────────
  server.on("/save", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"Sin body\"}");
      return;
    }

    StaticJsonDocument<2048> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"JSON inválido\"}");
      return;
    }

    // Aplicar valores al struct Config
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

    JsonArray tracks = doc["tracks"];
    for (int i = 0; i < 7 && i < (int)tracks.size(); i++) {
      cfg.tracks[i].order  = tracks[i]["order"]  | cfg.tracks[i].order;
      cfg.tracks[i].active = tracks[i]["active"]  | cfg.tracks[i].active;
      strlcpy(cfg.tracks[i].species, tracks[i]["species"] | cfg.tracks[i].species,
              sizeof(cfg.tracks[i].species));
    }

    // Sincronizar RTC si vienen fecha y hora desde el portal
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

    // Aplicar volumen al DFPlayer en caliente
    dfPlayer.volume(cfg.volume);

    bool shutdown = doc["shutdown"] | false;
    bool ok = guardarConfigSD();
    server.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"Error SD\"}");

    if (ok) {
      if (shutdown) {
        Serial.println("[PORTAL] Config guardada. Cerrando portal en 5 s...");
        delay(5000);
        portalShutdown = true;
      } else {
        Serial.println("[PORTAL] Config guardada.");
      }
    }
  });

  // ── Descarga del CSV ────────────────────────────────────────────────────
  server.on("/csv", HTTP_GET, []() {
    if (!SD.exists(CSV_PATH)) {
      server.send(404, "text/plain", "CSV no encontrado aún.");
      return;
    }
    File f = SD.open(CSV_PATH, FILE_READ);
    if (!f) { server.send(500, "text/plain", "Error al abrir CSV."); return; }

    server.sendHeader("Content-Disposition", "attachment; filename=\"sesiones.csv\"");
    server.streamFile(f, "text/csv");
    f.close();
  });

  // ── Portal cautivo — estrategia correcta por SO ─────────────────────────
  //
  // El error anterior: responder "Success" o 204 a las URLs de detección
  // hace que el SO concluya que HAY internet y NO abre el webview del portal.
  //
  // Estrategia correcta:
  //   iOS/macOS  → /hotspot-detect.html: servir la página del portal (HTML 200).
  //                El SO detecta que el título NO es "Success" y abre el webview.
  //   Android    → /generate_204 y /gen_204: responder 302 al portal.
  //                Android abre el webview al recibir redirección en esa URL.
  //   Windows    → /connecttest.txt y /ncsi.txt: 302 al portal.
  //   Cualquier otra URL desconocida → 302 al portal.
  //
  // En todos los casos el webview aterriza en "/" que sirve el HTML completo.

  // iOS / macOS: NO responder "Success" — servir el portal directamente
  server.on("/hotspot-detect.html", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send_P(200, "text/html; charset=utf-8", PORTAL_HTML);
  });
  server.on("/library/test/success.html", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send_P(200, "text/html; charset=utf-8", PORTAL_HTML);
  });
  // Android: 302 → el webview sigue la redirección y carga el portal
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
  // Windows
  server.on("/connecttest.txt", HTTP_GET, []() {
    server.sendHeader("Location", "http://" AP_IP "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/ncsi.txt", HTTP_GET, []() {
    server.sendHeader("Location", "http://" AP_IP "/", true);
    server.send(302, "text/plain", "");
  });
  // Fallback: cualquier URL desconocida → portal
  server.onNotFound([]() {
    server.sendHeader("Location", "http://" AP_IP "/", true);
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send(302, "text/plain", "");
  });
}

// ============================================================================
// PORTAL: LOOP DE ATENCIÓN (bloquea hasta timeout o guardado)
// ============================================================================
void manejarPortal() {
  unsigned long tInicio = millis();
  unsigned long timeout = (unsigned long)PORTAL_TIMEOUT_S * 1000UL;
  unsigned long tUltimoNotify = 0;

  Serial.printf("[PORTAL] Esperando conexion (max %d s)...\n", PORTAL_TIMEOUT_S);

  while (true) {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(5);

    int wifiClientes = WiFi.softAPgetStationNum();

    if ((wifiClientes > 0 || bleClienteConectado) && !clienteActivo) {
      // Primera conexión (WiFi o BLE): cancelar el timeout de espera
      clienteActivo = true;
      timerActivo = false;
      Serial.println("[PORTAL] Cliente conectado (WiFi o BLE). Portal permanece abierto hasta que el usuario finalice.");
    }

    if (!clienteActivo) {
      // Nadie conectado aún: contar el timeout
      if (millis() - tInicio >= timeout) {
        Serial.println("[PORTAL] Sin conexión. Cerrando portal y reanudando operación.");
        break;
      }
    }

    // Refrescar el status BLE cada 2 s mientras haya un cliente BLE conectado
    if (bleClienteConectado && millis() - tUltimoNotify >= 2000) {
      actualizarStatusBLE();
      tUltimoNotify = millis();
    }

    // Si hay cliente: esperar hasta shutdown (enviado desde /save o característica Command)
    if (portalShutdown) break;
  }

  Serial.println("[PORTAL] Timeout alcanzado. Cerrando portal.");
}

// ============================================================================
// PORTAL: CERRAR AP Y APAGAR RF
// ============================================================================
void cerrarPortal() {
  server.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  detenerBLE();
  btStop();
  Serial.println("[PORTAL] AP, DNS, WiFi, BLE y BT apagados.");
}

// ============================================================================
// BLE: SERVICIO ZÉFIRO STRIX (ver docs/zefiro_ble_protocol_v1.md)
// ============================================================================
// Corre en paralelo al portal WiFi durante la misma ventana de configuración.
// No reemplaza al portal HTTP (que sigue intacto), es un transporte alternativo
// para la app Flutter. Reutiliza el mismo struct Config; NO reutiliza el
// endpoint /save de HTTP porque el protocolo separa Config / TimeSync / Command
// en características distintas (a diferencia del payload combinado de /save).

class ZefiroServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* srv, NimBLEConnInfo& info) override {
    bleClienteConectado = true;
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
      dfPlayer.volume(cfg.volume);
      Serial.println("[BLE] Config recibida y guardada.");
    } else {
      Serial.println("[BLE] Config inválida, ignorada.");
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
      delay(5000);
      portalShutdown = true;
    }
  }
};

// Arma el JSON de configuración actual (mismos campos que la sección 3 del
// protocolo, y los mismos que ya sirve el endpoint HTTP /config).
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
  doc["volume"]     = cfg.volume;
  doc["gainFactor"] = cfg.gainFactor;

  JsonArray tracks = doc.createNestedArray("tracks");
  for (int i = 0; i < 7; i++) {
    JsonObject t = tracks.createNestedObject();
    t["order"]   = cfg.tracks[i].order;
    t["species"] = cfg.tracks[i].species;
    t["active"]  = cfg.tracks[i].active;
  }

  doc["totalSessions"]   = cfg.totalSessions;
  doc["totalRecordings"] = cfg.totalRecordings;

  String out;
  serializeJson(doc, out);
  return out;
}

// Aplica un JSON de configuración recibido por BLE al struct Config.
// Solo toca campos de configuración (sección 3 del protocolo): a diferencia
// del endpoint HTTP /save, NO procesa sysDate/sysTime/shutdown aquí, esos
// viajan por las características TimeSync y Command respectivamente.
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

  cfg.recTimeSeg  = doc["recTime"]    | cfg.recTimeSeg;
  cfg.pauseMs     = doc["pauseMs"]    | cfg.pauseMs;
  cfg.volume      = doc["volume"]     | cfg.volume;
  cfg.gainFactor  = doc["gainFactor"] | cfg.gainFactor;

  JsonArray tracks = doc["tracks"];
  for (int i = 0; i < 7 && i < (int)tracks.size(); i++) {
    cfg.tracks[i].order  = tracks[i]["order"]  | cfg.tracks[i].order;
    cfg.tracks[i].active = tracks[i]["active"] | cfg.tracks[i].active;
    strlcpy(cfg.tracks[i].species, tracks[i]["species"] | cfg.tracks[i].species,
            sizeof(cfg.tracks[i].species));
  }

  return true;
}

// Arma y notifica el JSON de status (sección 4 del protocolo), incluyendo
// boardType ahora formalizado para distinguir esta placa DIY de la Tetra Main.
void actualizarStatusBLE() {
  if (!bleStatusChar) return;

  DateTime now = rtc.now();
  char rtcBuf[20];
  snprintf(rtcBuf, sizeof(rtcBuf), "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  uint64_t sdFree = (SD.totalBytes() - SD.usedBytes()) / 1024;

  StaticJsonDocument<256> doc;
  doc["version"]    = FW_VERSION;
  doc["unitName"]   = cfg.unitName;
  doc["rtcTime"]    = rtcBuf;
  doc["sdFreeMB"]   = (int)(sdFree / 1024);
  doc["sessions"]   = cfg.totalSessions;
  doc["recordings"] = cfg.totalRecordings;
  doc["boardType"]  = BLE_BOARD_TYPE;

  String out;
  serializeJson(doc, out);

  bleStatusChar->setValue(out);
  if (bleClienteConectado) bleStatusChar->notify();
}

// Inicia el servicio GATT y el advertising. Se llama junto a iniciarPortal().
void iniciarBLE() {
  Serial.println("[BLE] Iniciando servicio...");

  String nombreAdv = String("ZefiroStrix-") + cfg.unitName;
  NimBLEDevice::init(nombreAdv.c_str());
  NimBLEDevice::setMTU(247);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ZefiroServerCallbacks());

  NimBLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

  bleConfigChar = svc->createCharacteristic(
      BLE_CONFIG_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  bleConfigChar->setCallbacks(new ConfigCharCallbacks());

  bleStatusChar = svc->createCharacteristic(
      BLE_STATUS_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  bleTimeSyncChar = svc->createCharacteristic(
      BLE_TIMESYNC_UUID,
      NIMBLE_PROPERTY::WRITE);
  bleTimeSyncChar->setCallbacks(new TimeSyncCharCallbacks());

  bleCommandChar = svc->createCharacteristic(
      BLE_COMMAND_UUID,
      NIMBLE_PROPERTY::WRITE);
  bleCommandChar->setCallbacks(new CommandCharCallbacks());

  svc->start();

  actualizarStatusBLE();  // valor inicial disponible antes de la primera notificación

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();

  // ── Paquete de advertising principal (presupuesto de 31 bytes) ──────────
  // flags (3 bytes) + UUID de servicio de 128 bits (18 bytes) = 21 bytes.
  // El nombre NO cabe aquí: "ZefiroStrix-ZS-01" necesita 18 bytes y solo
  // quedan 10 libres, por eso se mueve al scan response (abajo).
  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN);   // LE General Discoverable
  advData.addServiceUUID(BLE_SERVICE_UUID);

  // ── Paquete de scan response (segundo paquete, presupuesto propio de 31 bytes) ──
  // Aquí va el nombre del dispositivo, que no cabía en el paquete principal.
  NimBLEAdvertisementData scanData;
  scanData.setName(nombreAdv.c_str());

  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->enableScanResponse(true);
  adv->start();

  Serial.printf("[BLE] Advertising como \"%s\" (nombre en scan response).\n", nombreAdv.c_str());
}

// Detiene advertising y libera el stack BLE por completo (llamado desde
// cerrarPortal(), antes de btStop()).
void detenerBLE() {
  if (!bleServer) return;
  NimBLEDevice::stopAdvertising();
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
  Serial.println("--- DIAGNÓSTICO DE HARDWARE ---\n");

  // RTC
  Serial.print("[RTC]       DS3231... ");
  if (rtc.begin()) {
    DateTime now = rtc.now();
    if (rtc.lostPower()) {
      Serial.println("OK (ADVERTENCIA: perdió energía, hora incorrecta)");
    } else {
      Serial.printf("OK — %04d/%02d/%02d %02d:%02d:%02d\n",
        now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    }
  } else {
    Serial.println("FALLO — No responde. Revisa SDA/SCL y alimentación.");
    ok = false;
  }

  // SD
  Serial.print("[SD]        MicroSD... ");
  if (SD.begin(SD_CS)) {
    uint64_t libre = (SD.totalBytes() - SD.usedBytes()) / (1024ULL * 1024);
    uint64_t total = SD.totalBytes() / (1024ULL * 1024);
    Serial.printf("OK — %llu MB libres de %llu MB\n", libre, total);
  } else {
    Serial.println("FALLO — No detectada. Revisa CS, SPI y formato FAT32.");
    ok = false;
  }

  // DFPlayer
  Serial.print("[DFPLAYER]  DFPlayer Mini... ");
  delay(1500);
  dfSerial.begin(9600, SERIAL_8N1, DF_RX, DF_TX);
  if (dfPlayer.begin(dfSerial, false, false)) {
    dfPlayer.volume(0);
    int n = dfPlayer.readFileCounts();
    if (n >= 7) {
      Serial.printf("OK — %d archivos en /mp3/\n", n);
    } else {
      Serial.printf("ADVERTENCIA — %d archivos (se esperan 7)\n", n);
    }
    dfPlayer.volume(cfg.volume > 0 ? cfg.volume : 30);
  } else {
    Serial.println("FALLO — No responde. Revisa TX/RX y alimentación.");
    ok = false;
  }

  // Micrófono
  Serial.print("[MICROFONO] INMP441 I2S... ");
  if (!diagnosticoMicrofono()) {
    Serial.println("FALLO — Sin señal. Revisa WS/SCK/SD y pin L/R a GND.");
    ok = false;
  }

  Serial.println();
  Serial.println(ok ? "[DIAG] Todos los componentes OK." : "[DIAG] Fallos críticos detectados.");
  Serial.println("-------------------------------\n");
  return ok;
}

// ============================================================================
// DIAGNÓSTICO DE MICRÓFONO (I2S + RMS)
// ============================================================================
bool diagnosticoMicrofono() {
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
  i2s_pin_config_t pins = { .bck_io_num = I2S_SCK, .ws_io_num = I2S_WS,
                             .data_out_num = -1, .data_in_num = I2S_SD };
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
      int32_t v = buf[i] >> 11;
      suma += (int64_t)v * v;
      if (v != 0) hay = true;
      n++;
    }
  }
  i2s_driver_uninstall(I2S_NUM_0);

  if (!hay || n == 0) return false;

  double rms  = sqrt((double)suma / n);
  double dbfs = 20.0 * log10(rms / 32767.0 + 1e-9);
  Serial.printf("OK — RMS %.0f (%.1f dBFS)\n", rms, dbfs);
  if (dbfs < -60.0) Serial.println("           [ADVERTENCIA] Nivel muy bajo — revisa pin L/R a GND.");
  if (dbfs > -10.0) Serial.println("           [ADVERTENCIA] Nivel alto — considera bajar gainFactor.");
  return true;
}

// ============================================================================
// I2S: INICIALIZAR PARA GRABACIÓN NORMAL
// ============================================================================
void initI2S() {
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
  i2s_pin_config_t pins = { .bck_io_num = I2S_SCK, .ws_io_num = I2S_WS,
                             .data_out_num = -1, .data_in_num = I2S_SD };
  i2s_driver_install(I2S_NUM_0, &cfg_i2s, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
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
// CICLO PRINCIPAL: construye secuencia ordenada y la ejecuta
// ============================================================================
void ejecutarCiclo() {
  // Construir lista de tracks activos ordenados por cfg.tracks[i].order
  int orden[7];
  int nActivos = 0;
  for (int o = 1; o <= 7; o++) {
    for (int i = 0; i < 7; i++) {
      if (cfg.tracks[i].active && cfg.tracks[i].order == o) {
        orden[nActivos++] = i;  // índice físico del track
        break;
      }
    }
  }
  if (nActivos == 0) { Serial.println("[CICLO] No hay tracks activos."); return; }

  int pos = 0;  // posición dentro de orden[]

  while (true) {
    int idxFisico  = orden[pos];
    int trackFisico = idxFisico + 1;  // 1-7 para DFPlayer

    DateTime now  = rtc.now();
    bool activo   = esHoraActiva(now.hour(), now.minute());
    if (!activo && pos == 0) {
      Serial.println("[CICLO] Horario terminó antes de iniciar. Saliendo.");
      break;
    }

    reproducirYGrabar(trackFisico, idxFisico);

    pos++;
    if (pos >= nActivos) pos = 0;

    now    = rtc.now();
    activo = esHoraActiva(now.hour(), now.minute());
    if (!activo) {
      Serial.println("[CICLO] Horario terminado. Saliendo del ciclo.");
      break;
    }
  }
}

// ============================================================================
// REPRODUCIR UN TRACK Y GRABAR LA RESPUESTA
// ============================================================================
void reproducirYGrabar(int trackFisico, int idxFisico) {
  Serial.printf("\n[T%d] %s\n", trackFisico, cfg.tracks[idxFisico].species);

  dfPlayer.play(trackFisico);
  delay(1500);

  unsigned long tRep = millis();
  while (digitalRead(DF_BUSY) == LOW) {
    delay(100);
    if (millis() - tRep > 600000UL) {
      Serial.println("[T] ADVERTENCIA: BUSY prolongado, continuando.");
      break;
    }
  }
  Serial.printf("[T%d] Reproducción finalizada. Silenciando amp (400ms)...\n", trackFisico);
  delay(400);

  grabarRespuesta(trackFisico, idxFisico);
  delay(cfg.pauseMs);
}

// ============================================================================
// GRABACIÓN WAV CON BUFFER DE RAM
// ============================================================================
void grabarRespuesta(int trackFisico, int idxFisico) {
  DateTime now = rtc.now();

  char dirPath[36];
  sprintf(dirPath, "/REC/%04d-%02d-%02d", now.year(), now.month(), now.day());
  crearDirectorio("/REC");
  crearDirectorio(dirPath);

  // Nombre: YYYYMMDD_HHMMSS_T<n>_<especie_corta>.wav
  // Especie: primeras 12 chars sin espacios especiales
  char especieCorta[16];
  strlcpy(especieCorta, cfg.tracks[idxFisico].species, sizeof(especieCorta));
  // Reemplazar espacios y paréntesis por guión bajo
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

  Serial.printf("[T%d] Grabando → %s\n", trackFisico, filePath);

  File f = SD.open(filePath, FILE_WRITE);
  if (!f) {
    Serial.printf("[T%d] ERROR: no se pudo crear el WAV.\n", trackFisico);
    return;
  }

  writeWavHeader(f);

  uint32_t      dataSize = 0;
  unsigned long tIni     = millis();
  int16_t       ramBuf[CHUNK_SAMPLES];
  int           bufIdx   = 0;
  int64_t       sumaRMS  = 0;
  long          nMuestras = 0;

  while (millis() - tIni < (unsigned long)(cfg.recTimeSeg * 1000)) {
    int32_t i2sBuf[128]; size_t leidos;
    i2s_read(I2S_NUM_0, i2sBuf, sizeof(i2sBuf), &leidos, portMAX_DELAY);

    int n = leidos / 4;
    for (int i = 0; i < n; i++) {
      int32_t val = (i2sBuf[i] >> 11) * cfg.gainFactor;
      if (val >  32767) val =  32767;
      if (val < -32768) val = -32768;

      ramBuf[bufIdx++] = (int16_t)val;
      sumaRMS += (int64_t)val * val;
      nMuestras++;

      if (bufIdx >= CHUNK_SAMPLES) {
        dataSize += f.write((const uint8_t*)ramBuf, CHUNK_SAMPLES * 2);
        bufIdx = 0;
      }
    }
  }
  if (bufIdx > 0) {
    dataSize += f.write((const uint8_t*)ramBuf, bufIdx * 2);
  }

  sellarWav(f, dataSize);
  f.close();

  float rms    = nMuestras > 0 ? sqrt((double)sumaRMS / nMuestras) : 0;
  float durSeg = (float)dataSize / (SAMPLE_RATE * 2);
  float tamKB  = (float)(dataSize + 44) / 1024.0f;

  Serial.printf("[T%d] OK — %.1fs, %.1fKB, RMS=%.0f\n",
                trackFisico, durSeg, tamKB, rms);

  cfg.totalRecordings++;
  registrarCSV(filePath, trackFisico, rms, durSeg);
}

// ============================================================================
// LOG CSV
// ============================================================================
void registrarCSV(const char* archivo, int trackNum, float rms, float durSeg) {
  bool nuevo = !SD.exists(CSV_PATH);
  File f = SD.open(CSV_PATH, FILE_APPEND);
  if (!f) return;

  if (nuevo) {
    f.println("fecha,hora,unidad,estacion,proyecto,investigador,"
              "utm_zona,utm_este,utm_norte,"
              "track,especie,archivo,duracion_seg,rms,fw_version");
  }

  DateTime now = rtc.now();
  char linea[320];
  snprintf(linea, sizeof(linea),
    "%04d-%02d-%02d,%02d:%02d:%02d,"
    "%s,%s,%s,%s,"
    "%s,%ld,%ld,"
    "%d,%s,%s,%.1f,%.0f,%s",
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second(),
    cfg.unitName, cfg.stationName, cfg.projectName, cfg.researcher,
    cfg.utmZone, cfg.utmEaste, cfg.utmNorte,
    trackNum, cfg.tracks[trackNum - 1].species,
    archivo, durSeg, rms, FW_VERSION);

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
// WAV: HEADER ESTÁNDAR 44 BYTES
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
  f.write((uint8_t*)&zero,       4);   // offset  4: riffSize (placeholder)
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
  f.write((uint8_t*)&zero,       4);   // offset 40: dataSize (placeholder)
}

// ============================================================================
// WAV: SELLAR CON TAMAÑOS REALES (offsets fijos)
// ============================================================================
void sellarWav(File& f, uint32_t dataSize) {
  uint32_t riffSize = dataSize + 36;
  f.seek(4);  f.write((uint8_t*)&riffSize, 4);
  f.seek(40); f.write((uint8_t*)&dataSize, 4);
}
