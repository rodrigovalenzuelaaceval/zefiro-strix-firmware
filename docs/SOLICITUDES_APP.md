# Solicitudes de la app hacia el firmware

Documento vivo de coordinación entre zefiro-strix-app y zefiro-strix-firmware.
Cada solicitud se agrega cuando la app necesita un dato o comportamiento nuevo del firmware, y se marca como resuelta cuando el firmware lo implementa y ambos lados quedan sincronizados.

## Pendientes

### 1. Duración de reproducción por pista
La app necesita saber cuánto dura la reproducción de cada pista (no la grabación, esa ya viaja en recTime) para calcular cuántas noches de monitoreo caben en el espacio libre de la SD. Se relaciona con la función ya planeada de auto-poblar especies leyendo los nombres de archivo de la SD: al escanear, leer también la duración del WAV desde su encabezado y exponerla como un campo nuevo (ej. durationSec) en la página de Tracks del protocolo BLE.

### 2. Separar nombre común y nombre científico en cada pista
Hoy el campo species es un solo string de texto libre donde el nombre científico se escribe entre paréntesis por convención, sin estructura real. Se solicita separar en dos campos: commonName y scientificName, para que la app pueda aplicar formato (cursiva) de forma confiable al nombre científico, sin depender de parsear texto libre.

### 3. Sample rate de grabación configurable
Hoy SAMPLE_RATE es un valor fijo (8000 Hz) en el firmware. Se solicita que sea un campo configurable en cfg (ej. sampleRate), con un conjunto razonable de opciones (8000, 16000, 22050, 44100 Hz), reinstalando el driver I2S cuando cambie. Bits por muestra y número de canales se mantienen fijos (16-bit, mono), no se solicita hacerlos configurables. Este valor también es necesario para que la app calcule correctamente la autonomía de grabación restante, ya que la tasa de bytes por segundo depende de él.

## Resueltos

### sdFreeMB en Status
Ya implementado y documentado en docs/zefiro_ble_protocol_v1.md. La app ya lo usa para calcular capacidad de grabación restante.
