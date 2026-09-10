/*
 * thrusters.cpp — PISCINA
 * ------------------------------------------------------------------
 * Ver thrusters.h para el contrato, el armado y la rampa.
 * ------------------------------------------------------------------
 */

#include "thrusters.h"

#include <Arduino.h>
#include <Preferences.h>

#include "pins.h"

static Preferences prefs;

static uint8_t  throttlePct = THRUST_DEFAULT_PCT;
static uint16_t rampTenths  = THRUST_RAMP_US_PER_MS * 10;

static uint16_t portUs = ESC_US_NEUTRAL;
static uint16_t stbdUs = ESC_US_NEUTRAL;
static uint16_t portTarget = ESC_US_NEUTRAL;
static uint16_t stbdTarget = ESC_US_NEUTRAL;

static uint32_t armStartMs = 0;
static bool     armed      = false;
static uint32_t lastRampMs = 0;

static NavCmd   curNav = NAV_STOP;
static ThrStats stats;

/* Modo de identificacion de pines: niveles continuos en vez de servo. */
static bool     pinTest = false;

/* ==================================================================
 *  Salida
 * ================================================================== */

static void writePort(uint16_t us) {
  portUs = us;
  ledcWrite(LEDC_CH_ESC_PORT, escMicrosToDuty(us));
}

static void writeStbd(uint16_t us) {
  stbdUs = us;
  ledcWrite(LEDC_CH_ESC_STBD, escMicrosToDuty(us));
}

/*
 * Pulso correspondiente a "todo adelante" o "todo atras" al empuje actual.
 * El neutro son 1500 y el recorrido util 500 us a cada lado.
 */
static uint16_t usForward() {
  return (uint16_t)(ESC_US_NEUTRAL + (500L * throttlePct) / 100);
}

static uint16_t usReverse() {
  return (uint16_t)(ESC_US_NEUTRAL - (500L * throttlePct) / 100);
}

/*
 * Mezcla diferencial. Es la tabla de mezcla diferencial del proyecto:
 *
 *   comando       babor      estribor
 *   NAV_FORWARD   adelante   adelante
 *   NAV_REVERSE   atras      atras
 *   NAV_CW        adelante   atras       (giro horario)
 *   NAV_CCW       atras      adelante
 */
static void mixTargets(NavCmd nav) {
  switch (nav) {
    case NAV_FORWARD: portTarget = usForward(); stbdTarget = usForward(); break;
    case NAV_REVERSE: portTarget = usReverse(); stbdTarget = usReverse(); break;
    case NAV_CW:      portTarget = usForward(); stbdTarget = usReverse(); break;
    case NAV_CCW:     portTarget = usReverse(); stbdTarget = usForward(); break;
    case NAV_STOP:
    default:          portTarget = ESC_US_NEUTRAL; stbdTarget = ESC_US_NEUTRAL; break;
  }
}

/* Acerca `cur` a `target` como mucho `step` microsegundos. */
static uint16_t approach(uint16_t cur, uint16_t target, uint16_t step) {
  if (cur < target) {
    return ((uint16_t)(target - cur) <= step) ? target : (uint16_t)(cur + step);
  }
  if (cur > target) {
    return ((uint16_t)(cur - target) <= step) ? target : (uint16_t)(cur - step);
  }
  return cur;
}

/* ==================================================================
 *  API
 * ================================================================== */

void thrustersBegin() {
  memset(&stats, 0, sizeof(stats));

  /*
   * Los pines en BAJO antes de configurar el LEDC. Sin pulso valido un ESC
   * no arma, que es exactamente el estado seguro mientras la placa arranca:
   * mas vale que no arme a que arme con una señal a medio configurar.
   */
  pinMode(PIN_ESC_PORT, OUTPUT);
  digitalWrite(PIN_ESC_PORT, LOW);
  pinMode(PIN_ESC_STBD, OUTPUT);
  digitalWrite(PIN_ESC_STBD, LOW);

  /*
   * ledcSetup devuelve la frecuencia REAL conseguida, o 0 si no pudo. No
   * comprobarlo dejaba pasar en silencio el caso en que el temporizador no
   * admite la combinacion de frecuencia y resolucion pedida — y el sintoma
   * seria justo un pin que no conduce.
   */
  stats.setupHzPort = ledcSetup(LEDC_CH_ESC_PORT, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  stats.setupHzStbd = ledcSetup(LEDC_CH_ESC_STBD, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  ledcAttachPin(PIN_ESC_PORT, LEDC_CH_ESC_PORT);
  ledcAttachPin(PIN_ESC_STBD, LEDC_CH_ESC_STBD);

  prefs.begin("thr", false);
  throttlePct = prefs.getUChar("pct", THRUST_DEFAULT_PCT);
  if (throttlePct > 100) {
    throttlePct = THRUST_DEFAULT_PCT;
  }
  rampTenths = prefs.getUShort("ramp", THRUST_RAMP_US_PER_MS * 10);
  if (rampTenths == 0) {
    rampTenths = THRUST_RAMP_US_PER_MS * 10;
  }

  /* Neutro sostenido: es lo que arma los ESC. */
  writePort(ESC_US_NEUTRAL);
  writeStbd(ESC_US_NEUTRAL);
  portTarget = ESC_US_NEUTRAL;
  stbdTarget = ESC_US_NEUTRAL;

  curNav     = NAV_STOP;
  armStartMs = millis();
  armed      = false;
  lastRampMs = millis();
}

void thrustersPoll() {
  const uint32_t now = millis();

  if (pinTest) {
    return;   /* niveles fijos: aqui no se toca nada */
  }

  if (!armed) {
    if ((now - armStartMs) >= ESC_ARM_MS) {
      armed = true;
    }
    /* Durante el armado no se toca nada: los ESC tienen que ver neutro
     * quieto, y cualquier movimiento aqui reiniciaria su cuenta. */
    lastRampMs = now;
    return;
  }

  /* Rampa. Se avanza por tiempo transcurrido, no por vuelta de bucle: asi
   * la velocidad de inversion no depende de lo ocupado que este el bucle. */
  const uint32_t dt = now - lastRampMs;
  if (dt == 0) {
    return;
  }
  lastRampMs = now;

  /*
   * Se acumulan decimas y solo se gasta la parte entera, para que una rampa
   * lenta no se pierda por redondeo: a 0.2 us/ms, cada vuelta de 5 ms daria
   * 1 decima, que truncada a us seria 0 y la señal no se moveria nunca.
   */
  static uint32_t tenthsAcc = 0;
  tenthsAcc += dt * rampTenths;
  uint32_t step = tenthsAcc / 10;
  tenthsAcc -= step * 10;

  if (step > 500) {
    step = 500;   /* un salto de bucle largo no se convierte en un salto de senal */
  }

  const uint16_t p = approach(portUs, portTarget, (uint16_t)step);
  const uint16_t s = approach(stbdUs, stbdTarget, (uint16_t)step);
  if (p != portUs) { writePort(p); }
  if (s != stbdUs) { writeStbd(s); }
}

bool thrustersArmed() {
  return armed;
}

void thrustersSetNav(NavCmd nav) {
  curNav = nav;
  if (!armed) {
    /* Sin armar no se mueve nada; los objetivos se quedan en neutro para
     * que el ESC siga viendo lo que necesita. */
    portTarget = ESC_US_NEUTRAL;
    stbdTarget = ESC_US_NEUTRAL;
    return;
  }
  mixTargets(nav);
}

void thrustersStopNow() {
  curNav     = NAV_STOP;
  portTarget = ESC_US_NEUTRAL;
  stbdTarget = ESC_US_NEUTRAL;
  writePort(ESC_US_NEUTRAL);
  writeStbd(ESC_US_NEUTRAL);
  stats.stops++;
}

void thrustersSetThrottlePct(uint8_t pct) {
  throttlePct = (pct > 100) ? 100 : pct;
  prefs.putUChar("pct", throttlePct);
  /* Se recalculan los objetivos para que el cambio se note ya, con rampa. */
  if (armed) {
    mixTargets(curNav);
  }
}

uint8_t thrustersThrottlePct() {
  return throttlePct;
}

void thrustersSetRampTenths(uint16_t tenths) {
  if (tenths == 0) {
    tenths = 1;   /* 0 dejaria la señal congelada para siempre */
  }
  rampTenths = tenths;
  prefs.putUShort("ramp", rampTenths);
}

uint16_t thrustersRampTenths() {
  return rampTenths;
}

/* ==================================================================
 *  Identificacion de pines
 * ================================================================== */

void thrustersPinTest(uint8_t which) {
  pinTest = true;

  /* Se sueltan los canales LEDC para poder gobernar los pines a mano. */
  ledcDetachPin(PIN_ESC_PORT);
  ledcDetachPin(PIN_ESC_STBD);
  pinMode(PIN_ESC_PORT, OUTPUT);
  pinMode(PIN_ESC_STBD, OUTPUT);

  digitalWrite(PIN_ESC_PORT, (which == 1) ? HIGH : LOW);
  digitalWrite(PIN_ESC_STBD, (which == 2) ? HIGH : LOW);
}

void thrustersEndPinTest() {
  if (!pinTest) {
    return;
  }
  pinTest = false;

  ledcAttachPin(PIN_ESC_PORT, LEDC_CH_ESC_PORT);
  ledcAttachPin(PIN_ESC_STBD, LEDC_CH_ESC_STBD);

  /*
   * Se vuelve a neutro y se REARMA. Durante la prueba de pines los ESC
   * estuvieron sin señal valida, asi que hay que darles otra vez sus
   * ESC_ARM_MS de neutro antes de aceptar navegacion.
   */
  writePort(ESC_US_NEUTRAL);
  writeStbd(ESC_US_NEUTRAL);
  portTarget = ESC_US_NEUTRAL;
  stbdTarget = ESC_US_NEUTRAL;
  curNav     = NAV_STOP;
  armStartMs = millis();
  armed      = false;
  lastRampMs = millis();
}

bool thrustersInPinTest() {
  return pinTest;
}

const ThrStats *thrustersStats() {
  stats.armed        = armed;
  stats.armLeftMs    = armed ? 0 : (ESC_ARM_MS - (millis() - armStartMs));
  stats.portUs       = portUs;
  stats.stbdUs       = stbdUs;
  stats.portTargetUs = portTarget;
  stats.stbdTargetUs = stbdTarget;
  stats.nav          = (uint8_t)curNav;
  return &stats;
}
