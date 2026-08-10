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
#include <Preferences.h>
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
 *  pH — modulo HW-828 sobre PIN_PH_ADC
 * ================================================================== */

/*
 * Pendiente teorica de Nernst a 25 grados: 59.16 mV por unidad de pH.
 *
 * Negativa porque estas placas entregan MAS tension cuanto MAS acido, que es
 * lo contrario de lo que hace la sonda desnuda: el modulo invierte.
 *
 * Es una SUPOSICION sobre la ganancia del HW-828, que asume unidad. Si al
 * comprobar con un segundo liquido el pH sale sistematicamente comprimido o
 * estirado, la ganancia no es 1 y hace falta calibracion de dos puntos con
 * patrones de verdad. Por eso se guarda siempre la tension cruda.
 */
#define PH_SLOPE_NOMINAL_MV   (-59.16f)

/* Cuantas muestras se promedian. El ADC del ESP32-S3 es ruidoso y una sonda
 * de pH es lenta: promediar sale gratis y quita varias decimas de ruido. */
#define PH_SAMPLES            64
#define PH_SAMPLE_PERIOD_MS   500

/* Peso de cada muestra nueva en el filtro. Ver la nota en phSample(). */
#define PH_FILTER_ALPHA       0.1f

/*
 * Divisor resistivo entre Po y el GPIO. OBLIGATORIO en este montaje.
 *
 * Medido en placa: el HW-828 entrega 4.0 V con la sonda en bicarbonato, y la
 * entrada del ESP32-S3 admite 3.3. Conectarlo directo degrada o destruye el
 * ADC, y sin aviso.
 *
 * Con dos resistencias IGUALES el GPIO ve la mitad, asi que aqui se
 * multiplica por 2 para volver a la tension real de Po. Si algun dia se
 * cambia la relacion del divisor, este es el unico numero que hay que tocar
 * — pero entonces hay que RECALIBRAR, porque el desplazamiento guardado en
 * NVS esta expresado en voltios de Po.
 */
#define PH_DIVIDER_GAIN       2.0f

static Preferences phPrefs;
static PhStats     ph;
static uint32_t    lastPhSample = 0;

static void phLoadCal() {
  phPrefs.begin("ph", false);
  ph.calibrated = phPrefs.getBool("cal", false);
  ph.twoPoint   = phPrefs.getBool("two", false);
  ph.slope      = phPrefs.getFloat("slope", PH_SLOPE_NOMINAL_MV);
  ph.offsetPh   = phPrefs.getFloat("offPh", 7.0f);
  ph.offsetV    = phPrefs.getFloat("offV", 2.5f);
}

static void phStoreCal() {
  phPrefs.putBool("cal", ph.calibrated);
  phPrefs.putBool("two", ph.twoPoint);
  phPrefs.putFloat("slope", ph.slope);
  phPrefs.putFloat("offPh", ph.offsetPh);
  phPrefs.putFloat("offV", ph.offsetV);
}

static void phSample() {
  uint32_t acc = 0;
  for (uint16_t i = 0; i < PH_SAMPLES; i++) {
    acc += analogReadMilliVolts(PIN_PH_ADC);
  }
  const float mv = (float)acc / PH_SAMPLES;
  float atPin = mv / 1000.0f;

  /*
   * Filtro exponencial sobre la tension.
   *
   * Promediar 64 muestras seguidas quita el ruido rapido, pero queda una
   * deriva de +-20 mV entre lecturas que a 59 mV por unidad de pH son +-0.3
   * de dispersion — medido en banco: 8.89, 8.27, 8.68, 8.41 en el mismo
   * vaso quieto.
   *
   * Con alfa 0.1 y una muestra cada 500 ms, la constante de tiempo queda en
   * unos 5 s. Eso no pierde NADA de informacion util: el pH de una piscina
   * de camarones cambia en horas. Lo unico que se sacrifica es velocidad de
   * respuesta que aqui no sirve para nada.
   */
  if (ph.reads == 0) {
    ph.filtV = atPin;   /* primera muestra: se siembra el filtro */
  } else {
    ph.filtV += PH_FILTER_ALPHA * (atPin - ph.filtV);
  }
  atPin = ph.filtV;

  /* Se deshace el divisor: lo que se publica es la tension REAL de Po, que
   * es la que se mide con el multimetro y la que tiene sentido comparar. */
  ph.volts    = atPin * PH_DIVIDER_GAIN;
  ph.pinVolts = atPin;
  ph.reads++;

  /*
   * Con 11 dB de atenuacion el ADC del ESP32-S3 mide hasta unos 3.1 V. La
   * saturacion se comprueba en el PIN, no en Po: es el pin el que recorta.
   * Con el divisor de 2 esto solo deberia saltar si Po pasa de ~6.2 V.
   */
  if (atPin >= 3.05f) {
    ph.saturated++;
  }
}

float sensorPhVolts() {
  return ph.volts;
}

float sensorPh() {
  if (!ph.calibrated) {
    return NAN;
  }
  if (ph.pinVolts >= 3.05f) {
    return NAN;   /* señal recortada en el pin: no se inventa un valor */
  }

  /*
   * Compensacion por temperatura.
   *
   * La pendiente de Nernst escala con la temperatura ABSOLUTA: a 30 grados
   * es un 1.7 % mayor que a 25. Casi ningun montaje casero lo corrige, pero
   * aqui el DS18B20 esta justo al lado y el dato sale gratis.
   *
   * Solo se aplica cuando la pendiente es la teorica. Si se midio con dos
   * patrones, ya lleva dentro la temperatura a la que se calibro y
   * corregirla otra vez seria contarla dos veces.
   */
  float slope = ph.slope;
  ph.tempComp = false;
  if (!ph.twoPoint && sensorTempOk()) {
    const float tC = sensorTemperature();
    ph.compTempC = tC;
    ph.tempComp  = true;
    slope = PH_SLOPE_NOMINAL_MV * ((tC + 273.15f) / 298.15f);
  }

  const float mvDiff = (ph.volts - ph.offsetV) * 1000.0f;
  const float value  = ph.offsetPh + (mvDiff / slope);

  /* Rango fisico. Fuera de 0-14 la lectura no es pH, es un fallo. */
  if (value < 0.0f || value > 14.0f) {
    return NAN;
  }
  return value;
}

bool sensorPhOk() {
  return !isnan(sensorPh());
}

bool sensorPhCalOnePoint(float knownPh) {
  if (knownPh < 0.0f || knownPh > 14.0f) {
    return false;
  }
  ph.offsetV    = ph.volts;
  ph.offsetPh   = knownPh;
  ph.slope      = PH_SLOPE_NOMINAL_MV;
  ph.twoPoint   = false;
  ph.calibrated = true;
  ph.pendingA   = false;
  phStoreCal();
  return true;
}

bool sensorPhCalPointA(float knownPh) {
  if (knownPh < 0.0f || knownPh > 14.0f) {
    return false;
  }
  ph.pendingAV  = ph.volts;
  ph.pendingAPh = knownPh;
  ph.pendingA   = true;
  return true;
}

bool sensorPhCalPointB(float knownPh) {
  if (!ph.pendingA || knownPh < 0.0f || knownPh > 14.0f) {
    return false;
  }
  const float dPh = knownPh - ph.pendingAPh;
  if (fabsf(dPh) < 0.5f) {
    /* Dos patrones casi iguales dan una pendiente sin sentido. Mejor
     * rechazarlo que guardar una calibracion que parece buena y no lo es. */
    return false;
  }
  ph.slope      = ((ph.volts - ph.pendingAV) * 1000.0f) / dPh;
  ph.offsetV    = ph.volts;
  ph.offsetPh   = knownPh;
  ph.twoPoint   = true;
  ph.calibrated = true;
  ph.pendingA   = false;
  phStoreCal();
  return true;
}

void sensorPhCalReset() {
  ph.calibrated = false;
  ph.twoPoint   = false;
  ph.slope      = PH_SLOPE_NOMINAL_MV;
  ph.offsetPh   = 7.0f;
  ph.offsetV    = 2.5f;
  ph.pendingA   = false;
  phStoreCal();
}

const PhStats *sensorPhStats() {
  return &ph;
}

/* ==================================================================
 *  API
 * ================================================================== */

void sensorsBegin() {
  memset(&stats, 0, sizeof(stats));
  lastTemp = NAN;
  state = findProbe() ? T_IDLE : T_ABSENT;

  memset(&ph, 0, sizeof(ph));
  /* 11 dB: fondo de escala ~3.1 V, que es lo mas que admite la entrada. */
  analogSetPinAttenuation(PIN_PH_ADC, ADC_11db);
  phLoadCal();
  phSample();
}

void sensorsPoll() {
  const uint32_t now = millis();

  if ((now - lastPhSample) >= PH_SAMPLE_PERIOD_MS) {
    lastPhSample = now;
    phSample();
  }

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
