/*
 * actuators.cpp — PISCINA
 * ------------------------------------------------------------------
 * Ver actuators.h para el contrato y la secuencia de un ciclo.
 * ------------------------------------------------------------------
 */

#include "actuators.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "pins.h"

/* ==================================================================
 *  Estado
 * ================================================================== */

static Preferences prefs;

static DoseState  state       = DOSE_IDLE;
static uint32_t   stateMs     = 0;    /* cuando se entro al estado actual */
static uint32_t   augerStart  = 0;    /* cuando arranco el sinfin         */
static uint32_t   onMs        = 0;    /* t_on calculado para este ciclo   */
static float      doseGrams   = 0.0f;

static float      rateGps     = 0.0f; /* m_punto, g/s. 0 = sin calibrar   */
static uint8_t    sprayLevel  = 0;
static uint8_t    sprayMin    = 0;

static ActStats   stats;

/* Barrido de duty del aspersor, para hallar su minimo de arranque. */
static bool       sweeping    = false;
static uint32_t   sweepMs     = 0;
static uint8_t    sweepDuty   = 0;
#define SWEEP_STEP_MS   400
#define SWEEP_STEP      5

/* Tirada de calibracion del sinfin: corre un tiempo fijo y para. */
static bool       calRun      = false;
static uint32_t   calRunMs    = 0;
static uint32_t   calRunStart = 0;

/*
 * Duty minimo de arranque por defecto, hasta que se mida.
 *
 * 80 sobre 255 es un 31 %. Es una SUPOSICION, no una medida: sirve para que
 * el aspersor haga algo antes de calibrarlo, pero el valor real hay que
 * hallarlo con el barrido. Ponerlo demasiado bajo deja el nivel 1 zumbando
 * sin girar; demasiado alto, el nivel 1 ya sale disparado y se pierde
 * recorrido util.
 */
#define SPRAY_MIN_DEFAULT   80

/*
 * Deficit de la rampa, expresado en tiempo.
 *
 * Durante los FEEDER_RAMP_MS de subida el duty va de 0 a 100 % de forma
 * lineal, asi que el sinfin entrega de media LA MITAD de lo que entregaria a
 * plena marcha. En material eso equivale a perder media rampa de tiempo.
 *
 * Sin compensarlo, cada dosis sale corta de forma sistematica, y el error
 * pesa mas cuanto mas breve sea el ciclo: con m_punto = 32.5 g/s una racion
 * de 60 g dura 1.85 s, y los 100 ms de deficit son un 5 % de menos SIEMPRE,
 * en la misma direccion. Un sesgo constante es peor que ruido: no se
 * promedia con las repeticiones.
 *
 * La suposicion es que el caudal es proporcional a la velocidad. En un
 * sinfin es razonable por encima del umbral de arranque; por debajo no se
 * mueve nada, asi que el deficit real es algo mayor que este. Si al pesar
 * una racion sale sistematicamente corta, subir este numero.
 */
#define RAMP_DEFICIT_MS     (FEEDER_RAMP_MS / 2)

/* ==================================================================
 *  Salidas
 * ================================================================== */

static void augerWrite(uint8_t duty) {
  ledcWrite(LEDC_CH_FEEDER, duty);
  stats.augerDuty = duty;
}

static void sprayerWrite(uint8_t duty) {
  ledcWrite(LEDC_CH_SPRAYER, duty);
  stats.sprayerDuty = duty;
}

uint8_t actuatorsDutyForLevel(uint8_t level) {
  if (level == 0) {
    return 0;
  }
  if (level > SPRAYER_LEVEL_MAX) {
    level = SPRAYER_LEVEL_MAX;
  }
  /*
   * El nivel 1 cae en el duty minimo de arranque y el 10 en el maximo. NO es
   * un nivel*25.5 desde cero: con esa formula el nivel 1 daria un duty por
   * debajo del que el motor necesita para moverse, y el usuario veria un
   * aspersor que zumba y no gira.
   */
  const uint16_t span = FEED_PWM_MAX - sprayMin;
  return (uint8_t)(sprayMin + (span * (level - 1)) / (SPRAYER_LEVEL_MAX - 1));
}

/*
 * Rampa de arranque del sinfin.
 *
 * Sube de 0 a 100 % en FEEDER_RAMP_MS. Evita meter la corriente de arranque
 * completa a la bateria y el golpe seco a la reductora.
 *
 * La rampa va DENTRO de t_on a proposito: los gramos que salen durante la
 * subida cuentan, y por eso hay que calibrar con este mismo perfil. Ver la
 * nota de actuatorsCalRunAuger().
 */
static void augerRamp(uint32_t sinceStart) {
  if (sinceStart >= FEEDER_RAMP_MS) {
    augerWrite(FEED_PWM_MAX);
    return;
  }
  augerWrite((uint8_t)((FEED_PWM_MAX * sinceStart) / FEEDER_RAMP_MS));
}

/* ==================================================================
 *  Arranque
 * ================================================================== */

void actuatorsBegin() {
  memset(&stats, 0, sizeof(stats));

  /*
   * Los pines a nivel bajo ANTES de configurar el LEDC.
   *
   * Entre el reset y esta linea el pin esta flotando, y un BTS7960 con los
   * dos EN en alto interpreta cualquier nivel alto en RPWM como "gira". Es el
   * motivo de la nota de pins.h sobre soldar la resistencia de pull-down en
   * el modulo y no en el lado del ESP32.
   */
  pinMode(PIN_FEEDER_RPWM, OUTPUT);
  digitalWrite(PIN_FEEDER_RPWM, LOW);
  pinMode(PIN_SPRAYER_RPWM, OUTPUT);
  digitalWrite(PIN_SPRAYER_RPWM, LOW);

  ledcSetup(LEDC_CH_FEEDER, FEED_PWM_FREQ_HZ, FEED_PWM_RES_BITS);
  ledcSetup(LEDC_CH_SPRAYER, FEED_PWM_FREQ_HZ, FEED_PWM_RES_BITS);
  ledcAttachPin(PIN_FEEDER_RPWM, LEDC_CH_FEEDER);
  ledcAttachPin(PIN_SPRAYER_RPWM, LEDC_CH_SPRAYER);

  augerWrite(0);
  sprayerWrite(0);

  prefs.begin("act", false);
  rateGps  = prefs.getFloat("rate", 0.0f);
  sprayMin = (uint8_t)prefs.getUChar("spmin", SPRAY_MIN_DEFAULT);

  state = DOSE_IDLE;
}

/* ==================================================================
 *  Ciclo de dosificacion
 * ================================================================== */

DoseResult actuatorsStartDose(uint8_t grams) {
  if (state != DOSE_IDLE || calRun || sweeping) {
    stats.rejected++;
    return DOSE_ERR_BUSY;
  }
  if (grams == 0) {
    stats.rejected++;
    return DOSE_ERR_ZERO;
  }
  if (rateGps <= 0.0f) {
    stats.rejected++;
    return DOSE_ERR_NOCAL;
  }

  /* t_on = gramos/m_punto + el deficit de la rampa. Ver RAMP_DEFICIT_MS. */
  const float seconds = (float)grams / rateGps;
  const uint32_t ms   = (uint32_t)(seconds * 1000.0f) + RAMP_DEFICIT_MS;

  /*
   * Se RECHAZA en vez de recortar. Recortar entregaria una racion menor de
   * la pedida informando de exito — el peor fallo posible aqui: silencioso,
   * y con el camaron comiendo de menos sin que nadie lo sepa.
   */
  if (ms > MAX_DOSE_MS) {
    stats.rejected++;
    return DOSE_ERR_TOOLONG;
  }

  onMs      = ms;
  doseGrams = grams;
  stateMs   = millis();
  state     = DOSE_PRESPIN;

  /* El aspersor arranca YA; el sinfin espera al pre-giro. */
  sprayerWrite(actuatorsDutyForLevel(sprayLevel));
  return DOSE_ACCEPTED;
}

bool actuatorsBusy() {
  return state != DOSE_IDLE || calRun || sweeping;
}

DoseState actuatorsDoseState() {
  return state;
}

uint32_t actuatorsPlannedMs() {
  return onMs;
}

void actuatorsSetSprayerLevel(uint8_t level) {
  sprayLevel = (level > SPRAYER_LEVEL_MAX) ? SPRAYER_LEVEL_MAX : level;
}

uint8_t actuatorsSprayerLevel() {
  return sprayLevel;
}

void actuatorsStopAll() {
  augerWrite(0);
  sprayerWrite(0);
  state    = DOSE_IDLE;
  calRun   = false;
  sweeping = false;
}

/* ==================================================================
 *  Bucle
 * ================================================================== */

void actuatorsPoll() {
  const uint32_t now = millis();

  /* --- barrido de duty del aspersor --- */
  if (sweeping) {
    if ((now - sweepMs) >= SWEEP_STEP_MS) {
      sweepMs = now;
      if (sweepDuty >= FEED_PWM_MAX) {
        sweeping = false;
        sprayerWrite(0);
      } else {
        sweepDuty = (uint8_t)min<uint16_t>(FEED_PWM_MAX, sweepDuty + SWEEP_STEP);
        sprayerWrite(sweepDuty);
      }
    }
    return;
  }

  /* --- tirada de calibracion del sinfin --- */
  if (calRun) {
    const uint32_t elapsed = now - calRunStart;
    if (elapsed >= calRunMs) {
      augerWrite(0);
      calRun = false;
      stats.lastOnMs = elapsed;
    } else {
      augerRamp(elapsed);
    }
    return;
  }

  /* --- ciclo normal --- */
  switch (state) {
    case DOSE_IDLE:
      break;

    case DOSE_PRESPIN:
      if ((now - stateMs) >= SPRAY_LEAD_MS) {
        augerStart = now;
        stateMs    = now;
        state      = DOSE_RUNNING;
        augerRamp(0);
      }
      break;

    case DOSE_RUNNING: {
      const uint32_t elapsed = now - augerStart;

      /* La guarda es un vigilante, no el limite normal: si salta, algo se
       * quedo colgado y hay que enterarse. */
      if (elapsed >= MAX_DOSE_MS) {
        stats.guardTrips++;
        augerWrite(0);
        stateMs = now;
        state   = DOSE_TAIL;
        break;
      }

      if (elapsed >= onMs) {
        augerWrite(0);
        stats.lastOnMs  = elapsed;
        stats.lastGrams = doseGrams;
        stateMs = now;
        state   = DOSE_TAIL;
        break;
      }

      augerRamp(elapsed);
      break;
    }

    case DOSE_TAIL:
      if ((now - stateMs) >= SPRAY_TAIL_MS) {
        sprayerWrite(0);
        state = DOSE_IDLE;
        stats.cycles++;
      }
      break;
  }
}

/* ==================================================================
 *  Calibracion
 * ================================================================== */

bool actuatorsCalibrated() {
  return rateGps > 0.0f;
}

float actuatorsRate() {
  return rateGps;
}

bool actuatorsCalRunAuger(uint32_t ms) {
  if (actuatorsBusy() || ms == 0 || ms > MAX_DOSE_MS) {
    return false;
  }
  calRunMs    = ms;
  calRunStart = millis();
  calRun      = true;
  augerRamp(0);
  return true;
}

bool actuatorsCalSetGrams(float grams) {
  if (grams <= 0.0f || stats.lastOnMs <= RAMP_DEFICIT_MS) {
    return false;
  }
  /*
   * Se descuenta el deficit de la rampa ANTES de dividir, para que m_punto
   * quede expresado como caudal a plena marcha — que es lo que espera
   * actuatorsStartDose() al volver a sumarlo. Sin este descuento, calibrar
   * con el firmware daria un caudal mas bajo que el real y la compensacion
   * de la rampa se aplicaria dos veces.
   */
  const float effectiveS = (stats.lastOnMs - RAMP_DEFICIT_MS) / 1000.0f;
  rateGps = grams / effectiveS;
  prefs.putFloat("rate", rateGps);
  return true;
}

bool actuatorsCalSetRate(float gramsPerSec) {
  if (gramsPerSec <= 0.0f || gramsPerSec > 1000.0f) {
    return false;
  }
  rateGps = gramsPerSec;
  prefs.putFloat("rate", rateGps);
  return true;
}

void actuatorsCalReset() {
  rateGps = 0.0f;
  prefs.putFloat("rate", 0.0f);
}

void actuatorsSetSprayerMinDuty(uint8_t duty) {
  sprayMin = duty;
  prefs.putUChar("spmin", duty);
}

uint8_t actuatorsSprayerMinDuty() {
  return sprayMin;
}

/*
 * Modelo de alcance, medido en banco el 2026-08-10.
 *   40 % de duty  ->  1 m
 *   +10 % de duty -> +2 m
 * Ver la advertencia sobre la linealidad en actuators.h.
 */
#define RADIUS_REF_PCT      40.0f
#define RADIUS_REF_M        1.0f
#define RADIUS_M_PER_PCT    0.2f

float actuatorsRadiusForDuty(uint8_t duty) {
  if (duty == 0) {
    return 0.0f;
  }
  const float pct = (duty * 100.0f) / FEED_PWM_MAX;
  const float r   = RADIUS_REF_M + (pct - RADIUS_REF_PCT) * RADIUS_M_PER_PCT;
  return (r < 0.0f) ? 0.0f : r;
}

bool actuatorsCalSweepSprayer() {
  if (actuatorsBusy()) {
    return false;
  }
  sweeping  = true;
  sweepDuty = 0;
  sweepMs   = millis();
  sprayerWrite(0);
  return true;
}

bool actuatorsSweeping() {
  return sweeping;
}

uint8_t actuatorsSweepDuty() {
  return sweepDuty;
}

bool actuatorsSetSprayerRaw(uint8_t duty) {
  if (state != DOSE_IDLE || calRun) {
    return false;
  }
  sweeping = false;

  /*
   * Se pasa por cero antes de aplicar el valor. Sin esto, subir de 90 a 95
   * con el motor YA girando mediria el duty de mantenimiento, que es menor
   * que el de arranque — y el nivel 1 quedaria por debajo de lo que hace
   * falta para partir de parado, que es el caso real.
   */
  sprayerWrite(0);
  delay(300);
  sprayerWrite(duty);
  return true;
}

const ActStats *actuatorsStats() {
  return &stats;
}
