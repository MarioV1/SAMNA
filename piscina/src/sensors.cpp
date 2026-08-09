/*
 * sensors.cpp — PISCINA
 * ------------------------------------------------------------------
 * Ver sensors.h para el contrato y para por que nada aqui bloquea.
 *
 * POR QUE NO SE USA DallasTemperature
 *
 * La sonda de este montaje trae un ROM de fabrica con CRC invalido. No es un
 * problema de cableado ni de temporizacion: el algoritmo de busqueda y el
 * comando Read ROM devuelven EXACTAMENTE los mismos ocho bytes, de forma
 * estable, y esos bytes no cuadran con su propio CRC. El chip responde, su
 * codigo de familia es 0x28, y funciona — pero su numero de serie esta mal
 * grabado. Pasa con DS18B20 falsificados, que abundan.
 *
 * DallasTemperature descarta cualquier dispositivo cuyo ROM no valide, asi
 * que con esa sonda getDeviceCount() devuelve 0 para siempre.
 *
 * La salida: con UN solo sensor en el bus no hace falta su direccion.
 * Skip ROM (0xCC) habla con "el que haya". Y lo que de verdad importa —la
 * integridad de la lectura— sigue protegido, porque el scratchpad trae su
 * propio CRC, que no tiene nada que ver con el del ROM.
 *
 * LIMITACION que esto impone: solo puede haber UN dispositivo en el bus
 * 1-Wire. Si algun dia se cuelga un segundo sensor de GPIO7, contestaran
 * todos a la vez y saldra basura. Hoy no hay ninguno mas previsto.
 * ------------------------------------------------------------------
 */

#include "sensors.h"

#include <Arduino.h>
#include <OneWire.h>
#include <math.h>

#include "pins.h"

/* ==================================================================
 *  Bus 1-Wire
 * ================================================================== */

static OneWire oneWire(PIN_ONEWIRE);

/* Comandos del DS18B20 */
#define CMD_SKIP_ROM        0xCC
#define CMD_CONVERT_T       0x44
#define CMD_READ_SCRATCH    0xBE
#define CMD_WRITE_SCRATCH   0x4E

/*
 * Resolucion 11 bits (byte de configuracion 0x5F): 0.125 grados por paso,
 * 375 ms de conversion.
 *
 * No se usan los 12 bits que admite el chip. Su exactitud es de +-0.5 grados,
 * asi que los 0.0625 grados de paso del modo de 12 bits son resolucion por
 * debajo del error: cifras que se mueven sin significar nada. Con 11 bits el
 * paso sigue siendo cuatro veces mas fino que la exactitud y la conversion
 * tarda la mitad.
 */
#define CONFIG_11BIT        0x5F
#define CONVERSION_MS       375

#define SAMPLE_PERIOD_MS    1500
#define RESCAN_PERIOD_MS    10000

/* ==================================================================
 *  Estado
 * ================================================================== */

typedef enum {
  T_IDLE = 0,
  T_CONVERTING,
  T_ABSENT,
} TempState;

static TempState state = T_ABSENT;

static float     lastTemp   = NAN;
static uint32_t  lastSample = 0;
static uint32_t  convStart  = 0;
static uint32_t  lastScan   = 0;

static TempStats stats;

/* ==================================================================
 *  Primitivas
 * ================================================================== */

static bool skipRomCmd(uint8_t cmd) {
  if (oneWire.reset() != 1) {
    return false;
  }
  oneWire.write(CMD_SKIP_ROM);
  oneWire.write(cmd);
  return true;
}

/*
 * Lee los 9 bytes del scratchpad y valida SU CRC.
 *
 * Este CRC es el que protege el dato de verdad. Es independiente del ROM, y
 * por eso una sonda con el numero de serie mal grabado puede seguir dando
 * temperaturas fiables.
 */
static bool readScratchpad(uint8_t *sp) {
  if (!skipRomCmd(CMD_READ_SCRATCH)) {
    return false;
  }
  for (uint8_t i = 0; i < 9; i++) {
    sp[i] = oneWire.read();
  }
  return OneWire::crc8(sp, 8) == sp[8];
}

/* Fija la resolucion. TH y TL son alarmas que no usamos; van a 0. */
static bool setResolution() {
  if (oneWire.reset() != 1) {
    return false;
  }
  oneWire.write(CMD_SKIP_ROM);
  oneWire.write(CMD_WRITE_SCRATCH);
  oneWire.write(0x00);          /* TH */
  oneWire.write(0x00);          /* TL */
  oneWire.write(CONFIG_11BIT);  /* configuracion */
  return true;
}

/*
 * Mira el bus y deja constancia de lo que hay.
 *
 * El ROM se sigue leyendo y publicando aunque no se use para nada: es la
 * unica pista de que la sonda tiene el numero de serie mal grabado, y sin
 * ella el diagnostico habria que rehacerlo entero cada vez.
 */
static bool findProbe() {
  lastScan = millis();

  pinMode(PIN_ONEWIRE, INPUT);
  delayMicroseconds(100);
  stats.lineHigh = (digitalRead(PIN_ONEWIRE) == HIGH);

  stats.presence = (oneWire.reset() == 1);
  if (!stats.presence) {
    stats.present  = false;
    stats.romCrcOk = false;
    stats.addr[0]  = '\0';
    return false;
  }

  uint8_t rom[8];
  oneWire.write(0x33);   /* Read ROM: valido con un solo dispositivo */
  for (uint8_t i = 0; i < 8; i++) {
    rom[i] = oneWire.read();
  }
  stats.romCrcOk = (OneWire::crc8(rom, 7) == rom[7]);
  snprintf(stats.addr, sizeof(stats.addr),
           "%02X%02X%02X%02X%02X%02X%02X%02X",
           rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7]);

  /*
   * La prueba que decide: si el scratchpad valida su CRC, el bus lee bien y
   * la sonda sirve, tenga el ROM que tenga.
   */
  uint8_t sp[9];
  if (!readScratchpad(sp)) {
    stats.present = false;
    return false;
  }

  setResolution();
  stats.present = true;
  return true;
}

/* ==================================================================
 *  API
 * ================================================================== */

void sensorsBegin() {
  memset(&stats, 0, sizeof(stats));
  lastTemp = NAN;
  state = findProbe() ? T_IDLE : T_ABSENT;
}

void sensorsPoll() {
  const uint32_t now = millis();

  switch (state) {
    case T_ABSENT:
      if ((now - lastScan) >= RESCAN_PERIOD_MS) {
        if (findProbe()) {
          state = T_IDLE;
        }
      }
      break;

    case T_IDLE:
      if ((now - lastSample) >= SAMPLE_PERIOD_MS) {
        lastSample = now;
        convStart  = now;
        if (skipRomCmd(CMD_CONVERT_T)) {
          state = T_CONVERTING;
        } else {
          /* Desaparecio del bus. */
          stats.disconnects++;
          lastTemp = NAN;
          state    = T_ABSENT;
          lastScan = now;
        }
      }
      break;

    case T_CONVERTING: {
      /*
       * Se espera por tiempo y no sondeando el bus. Sondear obligaria a
       * hablar con el sensor en cada vuelta del bucle, y cada transaccion
       * 1-Wire deshabilita interrupciones unos microsegundos — justo lo que
       * no conviene hacer sin necesidad mientras la radio esta escuchando.
       */
      if ((now - convStart) < CONVERSION_MS) {
        break;
      }

      stats.convMs = now - convStart;
      state = T_IDLE;

      uint8_t sp[9];
      if (!readScratchpad(sp)) {
        /* CRC del scratchpad malo: el dato llego corrupto. Se tira. */
        stats.faults++;
        stats.crcFails++;
        lastTemp = NAN;
        break;
      }

      const int16_t raw = (int16_t)((sp[1] << 8) | sp[0]);
      const float   t   = raw / 16.0f;

      /*
       * 85.00 exacto es el valor con el que arranca el registro antes de la
       * primera conversion. Es una trampa clasica: no es un error evidente
       * como el -127, es una temperatura que parece real y que en un dia de
       * calor casi cuela. Se cuenta aparte, porque si sube el problema es de
       * alimentacion, no del codigo.
       */
      if (t == 85.0f) {
        stats.resetValues++;
        stats.faults++;
        lastTemp = NAN;
        break;
      }

      /* Rango fisico del sensor. No se estrecha al esperable en una
       * camaronera: se descartan lecturas imposibles, no se decide por el
       * usuario que temperatura puede tener su agua. */
      if (t < -55.0f || t > 125.0f) {
        stats.faults++;
        lastTemp = NAN;
        break;
      }

      lastTemp = t;
      stats.reads++;
      break;
    }
  }
}

float sensorTemperature() {
  return lastTemp;
}

bool sensorTempOk() {
  return !isnan(lastTemp);
}

const TempStats *sensorTempStats() {
  return &stats;
}
