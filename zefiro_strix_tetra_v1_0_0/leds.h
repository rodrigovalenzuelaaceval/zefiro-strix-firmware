/*
  leds.h
  Modulo de control de LEDs WS2812B - Tetra Main Board
  Zefiro Strix / Tetrapoda SpA

  Hardware:
    - 8x WS2812B en cadena, linea de datos en GPIO10 (net "LEDC" del esquematico)
    - Requiere la libreria "Adafruit NeoPixel" (Arduino Library Manager)

  Reglas de diseno (no negociables):
    1. NUNCA se encienden durante un ciclo de grabacion/reproduccion.
       Se controla con setLedsRecording(true/false); mientras este en true,
       updateLeds() no hace nada y cualquier intento de encenderlos es ignorado.
    2. Todas las animaciones son NO bloqueantes (usan millis(), nunca delay()),
       para no interferir con timing critico de audio/BLE en el resto del loop().
    3. Brillo bajo por defecto fuera del encendido, para cuidar la bateria
       en despliegues largos en terreno.

  Uso basico:
    #include "leds.h"

    void setup() {
      initLeds();
      startLedsStartupAnimation();
      pinMode(BOTON_ESTADO_PIN, INPUT_PULLUP); // GPIO6, boton "stealth"
    }

    void loop() {
      updateLeds();          // llamar siempre, en cada vuelta de loop()
      // ... resto de la logica del firmware ...
    }

    // Al iniciar un ciclo de grabacion/reproduccion:
    setLedsRecording(true);
    // ... grabar / reproducir ...
    setLedsRecording(false);

    // Al detectar carga (por ejemplo via GPIO7 / BAT_STAT):
    startLedsCharging();
    // Cuando el modulo de lectura de bateria por ADC confirme voltaje pleno:
    setLedsChargeComplete();

    // Al presionar el boton de estado (GPIO6):
    showLedsStatus(COLOR_VERDE, 2);   // ej: 2 parpadeos verdes = todo OK
*/

#ifndef ZEFIRO_LEDS_H
#define ZEFIRO_LEDS_H

#include <Adafruit_NeoPixel.h>
#include <math.h>

// ---------------- Configuracion de hardware ----------------
#define LED_PIN               47   // net "LED1", pin fisico 24 del modulo. GPIO10 (net "LEDC") NO tiene relacion con los LEDs — nombre similar pero circuito RC distinto.
#define LED_COUNT             8

// Brillo (0-255). Deliberadamente bajo fuera del encendido para ahorrar bateria.
#define LED_BRIGHTNESS_STATUS   40   // ~15%, usado en el LED unico de estado (boton)
#define LED_BRIGHTNESS_STARTUP  70   // un poco mas visible, es breve (pocos segundos)
#define LED_BRIGHTNESS_CHARGE   55

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- Colores de marca Tetrapoda ----------------
// Tomados del manual de identidad v2 (valores exactos, no aproximados)
#define COLOR_NARANJO   strip.Color(232, 103, 10)   // #E8670A
#define COLOR_VERDE     strip.Color(90, 170, 60)    // #5AAA3C
#define COLOR_AZUL      strip.Color(42, 122, 181)   // #2A7AB5

// ---------------- Colores ad hoc (avisos, no son de marca) ----------------
#define COLOR_ROJO      strip.Color(255, 0, 0)
#define COLOR_AMARILLO  strip.Color(255, 170, 0)
#define COLOR_OFF       strip.Color(0, 0, 0)

// ---------------- Estados internos ----------------
enum LedMode {
  LED_MODE_OFF,
  LED_MODE_STARTUP,
  LED_MODE_CHARGING,
  LED_MODE_CHARGE_COMPLETE,
  LED_MODE_STATUS_PULSE
};

static LedMode ledMode = LED_MODE_OFF;
static bool ledsRecordingActive = false;

// --- Animacion de encendido: barrido tipo "ola" por los 8 LEDs ---
// 3 pasadas: naranjo, verde, azul (una detras de otra)
static const uint8_t STARTUP_TOTAL_PASSES = 3;
static uint32_t startupColors[STARTUP_TOTAL_PASSES]; // se llena en initLeds()
static uint8_t  startupPassIndex = 0;
static int8_t   startupPixelIndex = 0;
static unsigned long startupLastStep = 0;
static const unsigned long STARTUP_STEP_MS = 55; // velocidad del barrido

// --- Carga: respiracion suave mientras carga, color fijo al completar ---
static unsigned long chargeBreathStart = 0;
static const unsigned long CHARGE_BREATH_PERIOD_MS = 2200;

// --- Pulso de estado: solo el LED indice 0, activado por el boton GPIO6 ---
static uint32_t statusPulseColor = 0;
static uint8_t  statusPulseBlinksRemaining = 0;
static bool     statusPulseOnPhase = false;
static unsigned long statusPulseLastToggle = 0;
static const unsigned long STATUS_PULSE_HALF_MS = 220;

// ---------------- API publica ----------------

void initLeds() {
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS_STATUS);
  strip.clear();
  strip.show();

  startupColors[0] = COLOR_NARANJO;
  startupColors[1] = COLOR_VERDE;
  startupColors[2] = COLOR_AZUL;
}

// Llamar SIEMPRE al iniciar/terminar un ciclo de grabacion o reproduccion.
// Al activarse, corta cualquier animacion en curso y apaga todo de inmediato.
void setLedsRecording(bool recording) {
  ledsRecordingActive = recording;
  if (recording) {
    strip.clear();
    strip.show();
    ledMode = LED_MODE_OFF;
  }
}

void startLedsStartupAnimation() {
  if (ledsRecordingActive) return;
  ledMode = LED_MODE_STARTUP;
  startupPassIndex = 0;
  startupPixelIndex = 0;
  startupLastStep = millis();
  strip.setBrightness(LED_BRIGHTNESS_STARTUP);
}

void startLedsCharging() {
  if (ledsRecordingActive) return;
  ledMode = LED_MODE_CHARGING;
  chargeBreathStart = millis();
  strip.setBrightness(LED_BRIGHTNESS_CHARGE);
}

void setLedsChargeComplete() {
  if (ledsRecordingActive) return;
  ledMode = LED_MODE_CHARGE_COMPLETE;
  strip.setBrightness(LED_BRIGHTNESS_CHARGE);
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, COLOR_VERDE);
  }
  strip.show();
}

// Pulso de estado en el LED 0 unicamente (uso tipico: boton "stealth" GPIO6)
// color: COLOR_VERDE (OK), COLOR_AMARILLO (bateria baja), COLOR_ROJO (error), etc.
// times: cantidad de parpadeos
void showLedsStatus(uint32_t color, uint8_t times) {
  if (ledsRecordingActive) return;
  ledMode = LED_MODE_STATUS_PULSE;
  strip.setBrightness(LED_BRIGHTNESS_STATUS);
  statusPulseColor = color;
  statusPulseBlinksRemaining = times;
  statusPulseOnPhase = false;
  statusPulseLastToggle = millis();
}

void stopLeds() {
  ledMode = LED_MODE_OFF;
  strip.clear();
  strip.show();
}

// Debe llamarse en cada vuelta de loop(). No bloqueante (no usa delay()).
void updateLeds() {
  if (ledsRecordingActive) {
    return; // regla dura: nunca tocar los LEDs durante grabacion/reproduccion
  }

  unsigned long now = millis();

  switch (ledMode) {

    case LED_MODE_OFF:
      break;

    case LED_MODE_STARTUP: {
      if (now - startupLastStep >= STARTUP_STEP_MS) {
        startupLastStep = now;
        strip.clear();
        strip.setPixelColor(startupPixelIndex, startupColors[startupPassIndex]);
        strip.show();
        startupPixelIndex++;
        if (startupPixelIndex >= LED_COUNT) {
          startupPixelIndex = 0;
          startupPassIndex++;
          if (startupPassIndex >= STARTUP_TOTAL_PASSES) {
            stopLeds();
          }
        }
      }
      break;
    }

    case LED_MODE_CHARGING: {
      // Respiracion suave en naranjo (modulacion senoidal del brillo, 0..1)
      unsigned long elapsed = (now - chargeBreathStart) % CHARGE_BREATH_PERIOD_MS;
      float phase = (float)elapsed / (float)CHARGE_BREATH_PERIOD_MS;
      float wave = (sinf(phase * 2.0f * PI - (PI / 2.0f)) + 1.0f) / 2.0f; // 0..1
      uint8_t brightness = (uint8_t)(wave * LED_BRIGHTNESS_CHARGE);
      strip.setBrightness(brightness);
      for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, COLOR_NARANJO);
      }
      strip.show();
      break;
    }

    case LED_MODE_CHARGE_COMPLETE:
      // Color fijo ya aplicado en setLedsChargeComplete(); nada que animar.
      break;

    case LED_MODE_STATUS_PULSE: {
      if (statusPulseBlinksRemaining == 0) {
        stopLeds();
        break;
      }
      if (now - statusPulseLastToggle >= STATUS_PULSE_HALF_MS) {
        statusPulseLastToggle = now;
        statusPulseOnPhase = !statusPulseOnPhase;
        if (statusPulseOnPhase) {
          strip.setPixelColor(0, statusPulseColor);
        } else {
          strip.setPixelColor(0, COLOR_OFF);
          statusPulseBlinksRemaining--;
        }
        strip.show();
      }
      break;
    }
  }
}

#endif
