/*
 * main.cpp — PISCINA · banco de prueba del enlace LoRa (paso 3)
 * ------------------------------------------------------------------
 * ESTO NO ES EL FIRMWARE DEFINITIVO. Solo ejercita el enlace:
 *
 *   - recibe comandos y los imprime
 *   - simula un ciclo de alimentacion con un temporizador, para poder
 *     probar el camino del ACK y el de ACK_BUSY sin motor conectado
 *   - manda telemetria SINTETICA cada TLM_PERIOD_MS
 *   - vigila el deadman de navegacion e informa cuando salta
 *
 * No toca ningun GPIO de potencia. Los sensores reales son el paso 5, los
 * actuadores el 6 y los ESC el 7. Se puede flashear con la placa sola: nada
 * mas que la antena tiene que estar conectado.
 * ------------------------------------------------------------------
 */

#include <Arduino.h>

#include "actuators.h"
#include "lora_link.h"
#include "pins.h"
#include "sensors.h"
#include "thrusters.h"

/* ------------------------------------------------------------------
 *  Estado del banco
 * ------------------------------------------------------------------ */

static uint32_t lastTlmMs = 0;

static NavCmd   curNav    = NAV_STOP;
static uint16_t curGrams  = 0;   /* masa objetivo del proximo ciclo, en gramos */
static uint8_t  curSprayer = 0;  /* nivel del aspersor, 0-10 — se usa en el paso 6 */
static bool     navActive = false;

/* La dosificacion ya no se simula: la lleva actuators.{h,cpp} con motores
 * de verdad. Ver la secuencia del ciclo en actuators.h. */

/* Estado de la radio, para poder reintentar sin colgar la placa. */
static bool     radioReady     = false;
static uint32_t lastRadioTryMs = 0;
static uint32_t lastRadioMsgMs = 0;

static const char *navName(uint8_t nav) {
  switch (nav) {
    case NAV_STOP:    return "STOP";
    case NAV_FORWARD: return "ADELANTE";
    case NAV_REVERSE: return "ATRAS";
    case NAV_CW:      return "HORARIO";
    case NAV_CCW:     return "ANTIHORARIO";
    default:          return "?";
  }
}

/* ------------------------------------------------------------------
 *  Arranque
 * ------------------------------------------------------------------ */

void setup() {
  Serial.begin(115200);
  /* Con USB CDC el puerto tarda en enumerar. Se espera, pero con tope: si
   * la placa arranca sin PC enchufado no puede quedarse aqui colgada. */
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("=================================================="));
  /* Solo ASCII en lo que sale por serial: los guiones largos y el punto
   * medio salen como interrogaciones en varios monitores. */
  Serial.println(F(" PISCINA - banco de prueba del enlace LoRa"));
  Serial.println(F("=================================================="));
  Serial.printf("Radio: %.1f MHz  SF%d  BW %.0f kHz  CR 4/%d  %d dBm\n",
                LORA_FREQ_MHZ, LORA_SF, LORA_BW_KHZ, LORA_CR, LORA_TX_DBM);
  Serial.printf("Sync word: 0x%02X (privado)\n", LORA_SYNC_WORD);
  Serial.printf("Telemetria cada %d ms | deadman a %d ms\n",
                TLM_PERIOD_MS, NAV_DEADMAN_MS);

  radioReady = linkBegin();
  if (radioReady) {
    Serial.println(F("Radio lista. Escuchando comandos.\n"));
  }

  /*
   * Los actuadores ANTES que los sensores: lo primero que tiene que pasar
   * tras el reset es que los dos RPWM queden en nivel bajo. Cada
   * milisegundo que pasan flotando es un milisegundo en que un BTS7960 con
   * los EN en alto podria estar moviendo un motor.
   */
  actuatorsBegin();
  Serial.printf("Actuadores: sinfin GPIO %d, aspersor GPIO %d, ambos a 0.\n",
                PIN_FEEDER_RPWM, PIN_SPRAYER_RPWM);
  if (!actuatorsCalibrated()) {
    Serial.println(F("m_punto SIN CALIBRAR: la alimentacion se rechazara."));
  } else {
    Serial.printf("m_punto = %.3f g/s\n", actuatorsRate());
  }

  /*
   * Los ESC ANTES que los sensores: el armado son 2 segundos de neutro
   * sostenido, y cuanto antes empiece esa cuenta, antes esta el catamaran
   * listo para responder.
   */
  thrustersBegin();
  Serial.printf("ESC: babor GPIO %d, estribor GPIO %d. Armando %d ms a "
                "%d us de neutro...\n",
                PIN_ESC_PORT, PIN_ESC_STBD, ESC_ARM_MS, ESC_US_NEUTRAL);
  Serial.printf("Empuje de trabajo %u %%  ->  %d us adelante / %d us atras\n",
                thrustersThrottlePct(),
                ESC_US_NEUTRAL + (500 * thrustersThrottlePct()) / 100,
                ESC_US_NEUTRAL - (500 * thrustersThrottlePct()) / 100);

  sensorsBegin();
  {
    const TempStats *ts = sensorTempStats();
    if (ts->present) {
      Serial.printf("DS18B20 encontrado en GPIO %d, direccion %s, %d bits\n\n",
                    PIN_ONEWIRE, ts->addr, 11);
    } else {
      Serial.printf("DS18B20 NO encontrado en GPIO %d.\n"
                    "  Comprueba la pull-up de 4.7k entre datos y 3V3, y que\n"
                    "  VCC y GND no esten invertidos (la sonda se calienta).\n"
                    "  Se reintenta cada 10 s.\n\n", PIN_ONEWIRE);
    }
  }

  lastTlmMs = millis();
}

/*
 * Radio caida: informar y reintentar.
 *
 * La primera version imprimia el error una sola vez y se quedaba colgada en
 * un bucle vacio. Mala idea: si nadie tenia el monitor abierto en ese
 * instante — y con USB CDC el puerto tarda en enumerar, asi que es lo
 * normal — la placa se queda muda para siempre y no hay forma de saber que
 * le pasa sin reflashear. Ademas un fallo transitorio de SPI dejaba la
 * unidad flotante inservible hasta ir a pulsarle el reset.
 */
static bool radioRetry() {
  const uint32_t now = millis();

  if ((now - lastRadioMsgMs) >= 2000) {
    lastRadioMsgMs = now;
    Serial.print(F("ERROR: "));
    Serial.println(linkLastError());
    Serial.println(F("  Comprueba la antena. Si el fallo es de chip o SPI,"));
    Serial.println(F("  prueba LORA_TCXO_V a 1.8 en shared/protocol.h."));
  }

  if ((now - lastRadioTryMs) >= 5000) {
    lastRadioTryMs = now;
    radioReady = linkBegin();
    if (radioReady) {
      Serial.println(F("Radio lista tras reintento.\n"));
      lastTlmMs = millis();
    }
  }
  return radioReady;
}

/* ------------------------------------------------------------------
 *  Comandos entrantes
 * ------------------------------------------------------------------ */

static void handleCmd(const CmdPacket &cmd) {
  const LinkStats *s = linkStats();

  Serial.printf("[RX cmd  seq=%-5u] nav=%-11s racion=%3u g  asp=%2u/10  "
                "RSSI %d dBm  SNR %.1f dB\n",
                cmd.hdr.seq, navName(cmd.nav), cmd.grams, cmd.sprayer,
                s->rssi, s->snr);

  curNav     = (NavCmd)cmd.nav;
  curGrams   = cmd.grams;
  curSprayer = cmd.sprayer;
  actuatorsSetSprayerLevel(curSprayer);
  thrustersSetNav(curNav);
  navActive = (curNav != NAV_STOP);

  /*
   * PARO expreso. Va antes que cualquier otra cosa y no lleva ACK: es
   * idempotente y llega repetido, asi que la repeticion sustituye a la
   * confirmacion.
   */
  if (cmd.feed == FEED_ABORT) {
    const float got = actuatorsAbort();
    if (got >= 0.0f) {
      Serial.printf("[PARO] ciclo abortado. Salieron ~%.0f g de los %u pedidos.\n"
                    "       Es una ESTIMACION por tiempo, no una medida.\n",
                    got, curGrams);
    }
    return;
  }

  if (cmd.feed != FEED_START) {
    return;
  }

  /*
   * Alimentacion real. Si ya hay un ciclo en curso se rechaza: la masa se
   * controla con el tiempo, asi que solapar dos ciclos daria una racion
   * imposible de calcular.
   *
   * Los reintentos con seq repetido no llegan hasta aqui — el enlace los
   * filtra y responde por su cuenta.
   */
  const DoseResult r = actuatorsStartDose(curGrams);

  switch (r) {
    case DOSE_ACCEPTED:
      Serial.printf("[FEED seq=%-5u] aceptado: %u g -> %lu ms de sinfin, "
                    "aspersor %u/10 -> ACK_OK\n",
                    cmd.hdr.seq, curGrams,
                    (unsigned long)actuatorsPlannedMs(), curSprayer);
      linkAckFeed(cmd.hdr.seq, ACK_OK);
      break;

    case DOSE_ERR_BUSY:
      Serial.printf("[FEED seq=%-5u] rechazado: ciclo en curso -> ACK_BUSY\n",
                    cmd.hdr.seq);
      linkAckFeed(cmd.hdr.seq, ACK_BUSY);
      break;

    case DOSE_ERR_NOCAL:
      Serial.printf("[FEED seq=%-5u] RECHAZADO: m_punto sin calibrar.\n"
                    "                 Calibra con 'ar'/'ag' antes de dosificar.\n",
                    cmd.hdr.seq);
      linkAckFeed(cmd.hdr.seq, ACK_REJECTED);
      break;

    case DOSE_ERR_TOOLONG:
      Serial.printf("[FEED seq=%-5u] RECHAZADO: %u g exigirian mas de %d ms.\n"
                    "                 Se rechaza en vez de recortar: una racion\n"
                    "                 corta informando de exito seria peor.\n",
                    cmd.hdr.seq, curGrams, MAX_DOSE_MS);
      linkAckFeed(cmd.hdr.seq, ACK_REJECTED);
      break;

    case DOSE_ERR_ZERO:
      Serial.printf("[FEED seq=%-5u] RECHAZADO: 0 gramos.\n", cmd.hdr.seq);
      linkAckFeed(cmd.hdr.seq, ACK_REJECTED);
      break;
  }
}

/* ------------------------------------------------------------------
 *  Consola de calibracion
 *
 *  Piscina no tenia teclado hasta ahora. Hace falta para el pH: sus
 *  constantes viven en NVS y se fijan aqui, de modo que recalibrar sea un
 *  procedimiento de banco y no una edicion de codigo.
 * ------------------------------------------------------------------ */

static void printPhHelp() {
  const PhStats *p = sensorPhStats();
  Serial.println(F("\n--- calibracion de pH ----------------------------"));
  Serial.printf("  Po = %.3f V  (en el GPIO %.3f V tras el divisor)\n  pH = ",
                p->volts, p->pinVolts);
  if (sensorPhOk()) {
    Serial.printf("%.2f\n", sensorPh());
  } else {
    Serial.println(p->calibrated ? "fuera de rango" : "SIN CALIBRAR");
  }
  Serial.printf("  modo: %s\n",
                !p->calibrated ? "sin calibrar"
                               : (p->twoPoint ? "dos puntos (pendiente medida)"
                                              : "un punto (pendiente teorica)"));
  Serial.printf("  pendiente %.2f mV/pH   referencia %.2f pH a %.3f V\n",
                p->slope, p->offsetPh, p->offsetV);
  if (p->tempComp) {
    Serial.printf("  compensado a %.2f C con el DS18B20\n", p->compTempC);
  }
  if (p->saturated > 0) {
    Serial.printf("  AVISO: %lu lecturas pegadas al tope del ADC.\n"
                  "         Po supera los 3.1 V que admite la entrada:\n"
                  "         hace falta un divisor entre Po y GPIO %d.\n",
                  (unsigned long)p->saturated, PIN_PH_ADC);
  }
  Serial.println(F("\n  pH:"));
  Serial.println(F("    c1 <ph>   calibrar con UN punto usando el liquido actual"));
  Serial.println(F("    ca <ph>   primer punto de una calibracion de dos"));
  Serial.println(F("    cb <ph>   segundo punto; calcula pendiente y guarda"));
  Serial.println(F("    cr        borrar la calibracion de pH"));

  const ActStats *as = actuatorsStats();
  Serial.println(F("\n--- actuadores -----------------------------------"));
  if (actuatorsCalibrated()) {
    Serial.printf("  m_punto = %.3f g/s   (100 g -> %.1f s de sinfin)\n",
                  actuatorsRate(), 100.0f / actuatorsRate());
  } else {
    Serial.println(F("  m_punto SIN CALIBRAR. La alimentacion se rechaza"));
    Serial.println(F("  con ACK_REJECTED hasta que se mida."));
  }
  Serial.printf("  aspersor: nivel %u/10, duty minimo de arranque %u/255\n",
                actuatorsSprayerLevel(), actuatorsSprayerMinDuty());
  Serial.printf("  duty actual: sinfin %u  aspersor %u\n",
                as->augerDuty, as->sprayerDuty);
  Serial.printf("  ciclos %lu   rechazados %lu   guarda MAX_DOSE %lu\n",
                (unsigned long)as->cycles, (unsigned long)as->rejected,
                (unsigned long)as->guardTrips);

  Serial.println(F("\n  calibracion del sinfin (m_punto):"));
  Serial.println(F("    ar <ms>   correr el sinfin ese tiempo, con la rampa real"));
  Serial.println(F("    ag <g>    decirle cuanto peso lo que salio -> calcula m_punto"));
  Serial.println(F("    am <g/s>  fijar m_punto a mano (si se midio fuera)"));
  Serial.println(F("    arr       borrar m_punto"));
  Serial.println(F("\n  aspersor:"));
  Serial.println(F("    sd <duty> fijar un duty crudo y mantenerlo (0-255)"));
  Serial.println(F("    sw        barrido automatico de duty"));
  Serial.println(F("    sm <duty> guardar ese minimo (0-255)"));
  Serial.println(F("    sl <0-10> probar un nivel"));
  Serial.println(F("\n  propulsion:"));
  Serial.println(F("    ti        estado de los ESC"));
  Serial.println(F("    tg <0-2>  nivel CONTINUO para localizar el pin con"));
  Serial.println(F("              multimetro (1=babor 2=estribor). tg 9 sale"));
  Serial.println(F("    tp <pct>  empuje de trabajo, 0-100 %"));
  Serial.println(F("    tr <us/ms> velocidad de rampa (2.0 normal, 0.2 para"));
  Serial.println(F("              verla comoda en el osciloscopio)"));
  Serial.println(F("    tv <0-4>  probar una direccion (0=stop 1=adel 2=atras"));
  Serial.println(F("              3=horario 4=antihorario)"));
  Serial.println(F("\n    x         PARAR TODO"));
  Serial.println(F("    ?         volver a mostrar esto"));
  Serial.println(F("--------------------------------------------------\n"));
}

static void handleLine(char *line) {
  while (*line == ' ') { line++; }
  if (*line == '\0') {
    return;
  }

  if (line[0] == '?') {
    printPhHelp();
    return;
  }

  if (strncmp(line, "cr", 2) == 0) {
    sensorPhCalReset();
    Serial.println(F("[pH] calibracion borrada. sensorPh() devuelve NAN."));
    return;
  }

  float v = 0.0f;
  int   n = 0;
  if (strncmp(line, "c1", 2) == 0 && sscanf(line + 2, "%f", &v) == 1) {
    const float volts = sensorPhVolts();
    if (sensorPhCalOnePoint(v)) {
      Serial.printf("[pH] un punto: %.2f pH a %.3f V. Pendiente teorica.\n"
                    "     Comprueba ahora con OTRO liquido: si el pH que sale\n"
                    "     no cuadra con la tira, la ganancia del modulo no es 1\n"
                    "     y hara falta calibracion de dos puntos.\n", v, volts);
    } else {
      Serial.println(F("[pH] valor fuera de 0-14."));
    }
    return;
  }

  if (strncmp(line, "ca", 2) == 0 && sscanf(line + 2, "%f", &v) == 1) {
    if (sensorPhCalPointA(v)) {
      Serial.printf("[pH] primer punto guardado: %.2f pH a %.3f V.\n"
                    "     Enjuaga la sonda, metela en el segundo liquido,\n"
                    "     espera a que se estabilice y usa 'cb <ph>'.\n",
                    v, sensorPhVolts());
    } else {
      Serial.println(F("[pH] valor fuera de 0-14."));
    }
    return;
  }

  if (strncmp(line, "cb", 2) == 0 && sscanf(line + 2, "%f", &v) == 1) {
    if (sensorPhCalPointB(v)) {
      const PhStats *p = sensorPhStats();
      Serial.printf("[pH] dos puntos: pendiente %.2f mV/pH.\n", p->slope);
      /*
       * El aviso mira la pendiente contra los -59.16 mV/pH de Nernst. Un
       * electrodo de vidrio NO puede superarlos — es termodinamica — asi que
       * una pendiente mayor solo puede venir de la ganancia del modulo o de
       * unos patrones mal medidos. Con tiras reactivas las dos causas son
       * indistinguibles, y por eso el aviso informa en vez de rechazar.
       */
      const float gain = fabsf(p->slope) / 59.16f;
      if (gain < 0.8f || gain > 1.2f) {
        Serial.printf("     AVISO: implica una ganancia de modulo de x%.2f.\n"
                      "     Plausible en un HW-828, pero tambien lo seria un\n"
                      "     error de +-0.5 en las tiras. Con tiras no se puede\n"
                      "     distinguir: anota la incertidumbre en la memoria.\n", gain);
      }
    } else {
      Serial.println(F("[pH] falta el primer punto ('ca'), el valor esta fuera\n"
                       "     de 0-14, o los dos patrones estan a menos de 0.5 pH\n"
                       "     y la pendiente no saldria fiable."));
    }
    return;
  }

  /* --- actuadores --- */

  if (line[0] == 'x' || line[0] == 'X') {
    actuatorsStopAll();
    thrustersStopNow();
    Serial.println(F("[ACT] TODO PARADO: sinfin, aspersor y ESC a neutro."));
    return;
  }

  /* --- propulsion --- */

  if (strncmp(line, "ti", 2) == 0) {
    const ThrStats *t = thrustersStats();
    Serial.println(F("\n--- propulsion -----------------------------------"));
    if (t->armed) {
      Serial.println(F("  ESC ARMADOS"));
    } else {
      Serial.printf("  armando... faltan %lu ms\n", (unsigned long)t->armLeftMs);
    }
    Serial.printf("  LEDC configurado a %lu Hz (babor) y %lu Hz (estribor)\n",
                  (unsigned long)t->setupHzPort, (unsigned long)t->setupHzStbd);
    if (t->setupHzPort == 0 || t->setupHzStbd == 0) {
      Serial.println(F("  ERROR: un 0 aqui significa que el LEDC NO acepto la"));
      Serial.println(F("  combinacion de frecuencia y resolucion. Ese pin no"));
      Serial.println(F("  esta emitiendo nada."));
    }
    if (thrustersInPinTest()) {
      Serial.println(F("  *** MODO IDENTIFICACION DE PINES ACTIVO ***"));
      Serial.println(F("  Niveles continuos, SIN senal de servo. 'tg 9' para salir."));
    }
    Serial.printf("  empuje %u %%   rampa %.1f us/ms\n",
                  thrustersThrottlePct(), thrustersRampTenths() / 10.0f);
    Serial.printf("  esperado en el osciloscopio: 50 Hz, periodo 20.0 ms\n"
                  "    neutro %d us | adelante %d us | atras %d us\n",
                  ESC_US_NEUTRAL,
                  ESC_US_NEUTRAL + (500 * thrustersThrottlePct()) / 100,
                  ESC_US_NEUTRAL - (500 * thrustersThrottlePct()) / 100);
    Serial.printf("  nav vigente: %s\n", navName(t->nav));
    Serial.printf("  babor    %4u us  ->  %4u us\n", t->portUs, t->portTargetUs);
    Serial.printf("  estribor %4u us  ->  %4u us\n", t->stbdUs, t->stbdTargetUs);
    Serial.printf("  paradas forzadas: %lu\n", (unsigned long)t->stops);
    Serial.println(F("  OJO: un ESC no da realimentacion. Esto dice que la"));
    Serial.println(F("  senal es correcta, NO que el ESC haya armado."));
    Serial.println(F("--------------------------------------------------\n"));
    return;
  }

  if (strncmp(line, "tp", 2) == 0 && sscanf(line + 2, "%d", &n) == 1) {
    if (n < 0 || n > 100) {
      Serial.println(F("[THR] porcentaje fuera de 0-100."));
      return;
    }
    thrustersSetThrottlePct((uint8_t)n);
    Serial.printf("[THR] empuje %d %% -> %d us adelante / %d us atras\n",
                  n, ESC_US_NEUTRAL + (500 * n) / 100,
                  ESC_US_NEUTRAL - (500 * n) / 100);
    return;
  }

  if (strncmp(line, "tg", 2) == 0 && sscanf(line + 2, "%d", &n) == 1) {
    if (n == 9) {
      thrustersEndPinTest();
      Serial.printf("[THR] senal de servo restaurada. Rearmando %d ms.\n", ESC_ARM_MS);
      return;
    }
    if (n < 0 || n > 2) {
      Serial.println(F("[THR] tg 0=ambos a 0V  1=babor a 3.3V  2=estribor a 3.3V"));
      Serial.println(F("      tg 9 = volver a la senal de servo"));
      return;
    }
    thrustersPinTest((uint8_t)n);
    Serial.printf("[THR] NIVEL CONTINUO: babor GPIO %d = %s, estribor GPIO %d = %s\n"
                  "      Busca con el multimetro en continua, respecto a GND.\n"
                  "      NO hay senal de servo mientras esto este activo.\n"
                  "      'tg 9' para volver.\n",
                  PIN_ESC_PORT, (n == 1) ? "3.3 V" : "0 V",
                  PIN_ESC_STBD, (n == 2) ? "3.3 V" : "0 V");
    return;
  }

  if (strncmp(line, "tr", 2) == 0 && sscanf(line + 2, "%f", &v) == 1) {
    if (v <= 0.0f || v > 50.0f) {
      Serial.println(F("[THR] rampa fuera de rango (0.1 a 50 us/ms)."));
      return;
    }
    thrustersSetRampTenths((uint16_t)(v * 10.0f));
    const float fullUs = 2.0f * (500.0f * thrustersThrottlePct() / 100.0f);
    Serial.printf("[THR] rampa %.1f us/ms.\n"
                  "      Una inversion completa (%.0f us) durara %.0f ms,\n"
                  "      es decir unos %.0f pulsos a 50 Hz.\n",
                  v, fullUs, fullUs / v, (fullUs / v) / 20.0f);
    return;
  }

  if (strncmp(line, "tv", 2) == 0 && sscanf(line + 2, "%d", &n) == 1) {
    if (n < 0 || n > 4) {
      Serial.println(F("[THR] 0=stop 1=adelante 2=atras 3=horario 4=antihorario"));
      return;
    }
    if (thrustersInPinTest()) {
      Serial.println(F("[THR] estas en modo identificacion de pines. 'tg 9' primero."));
      return;
    }
    if (!thrustersArmed()) {
      Serial.println(F("[THR] los ESC aun no han armado. Espera."));
      return;
    }
    thrustersSetNav((NavCmd)n);
    Serial.printf("[THR] %s. La senal llega con rampa, no de golpe.\n", navName(n));
    if (n != 0) {
      /*
       * Aviso deliberadamente ruidoso. El deadman solo se arma con
       * navegacion recibida por LoRa — la condicion navActive — asi que un
       * comando de consola SE QUEDA puesto. Hace falta que sea asi para el
       * banco: con la rampa lenta, una inversion dura mas que el deadman y
       * no se podria observar. Pero un empuje que no caduca merece decirse.
       */
      Serial.println(F("      *** ESTO NO CADUCA ***"));
      Serial.println(F("      El deadman solo lo alimenta la navegacion por"));
      Serial.println(F("      LoRa. Un comando de consola se queda puesto"));
      Serial.println(F("      hasta que mandes 'tv 0' o 'x'."));
    }
    return;
  }

  if (strncmp(line, "arr", 3) == 0) {
    actuatorsCalReset();
    Serial.println(F("[ACT] m_punto borrado. La alimentacion se rechazara."));
    return;
  }

  unsigned long ms = 0;
  if (strncmp(line, "ar", 2) == 0 && sscanf(line + 2, "%lu", &ms) == 1) {
    if (actuatorsCalRunAuger(ms)) {
      Serial.printf("[ACT] sinfin %lu ms con la rampa de produccion.\n"
                    "      Pesa lo que salga y dime cuanto con 'ag <gramos>'.\n",
                    (unsigned long)ms);
    } else {
      Serial.println(F("[ACT] no se pudo: hay algo en marcha, o el tiempo\n"
                       "      es 0 o pasa de MAX_DOSE_MS."));
    }
    return;
  }

  if (strncmp(line, "ag", 2) == 0 && sscanf(line + 2, "%f", &v) == 1) {
    if (actuatorsCalSetGrams(v)) {
      Serial.printf("[ACT] m_punto = %.3f g/s.\n"
                    "      Un ciclo de 100 g durara %.1f s.\n"
                    "      Comprueba pidiendo una racion y volviendo a pesar.\n",
                    actuatorsRate(), 100.0f / actuatorsRate());
    } else {
      Serial.println(F("[ACT] no se pudo: peso invalido, o todavia no has\n"
                       "      corrido el sinfin con 'ar <ms>'."));
    }
    return;
  }

  if (strncmp(line, "am", 2) == 0 && sscanf(line + 2, "%f", &v) == 1) {
    if (actuatorsCalSetRate(v)) {
      Serial.printf("[ACT] m_punto = %.3f g/s (caudal a plena marcha).\n"
                    "      60 g -> %.2f s   100 g -> %.2f s\n"
                    "      (incluye +%d ms de compensacion de rampa)\n",
                    actuatorsRate(),
                    60.0f / v + (FEEDER_RAMP_MS / 2) / 1000.0f,
                    100.0f / v + (FEEDER_RAMP_MS / 2) / 1000.0f,
                    FEEDER_RAMP_MS / 2);
    } else {
      Serial.println(F("[ACT] valor fuera de rango."));
    }
    return;
  }

  if (strncmp(line, "sw", 2) == 0) {
    if (actuatorsCalSweepSprayer()) {
      Serial.println(F("[ACT] barrido del aspersor: el duty sube 5 cada 400 ms.\n"
                       "      MIRA el motor y anota el duty al que EMPIEZA a girar.\n"
                       "      Luego guardalo con 'sm <duty>'. 'x' para cortar."));
    } else {
      Serial.println(F("[ACT] no se pudo: hay algo en marcha."));
    }
    return;
  }

  if (strncmp(line, "sd", 2) == 0 && sscanf(line + 2, "%d", &n) == 1) {
    if (n < 0 || n > 255) {
      Serial.println(F("[ACT] duty fuera de 0-255."));
      return;
    }
    if (actuatorsSetSprayerRaw((uint8_t)n)) {
      Serial.printf("[ACT] aspersor a duty %d (%d %%), arrancando desde parado.\n"
                    "      Mira el EJE: gira o solo zumba?\n", n, n * 100 / 255);
    } else {
      Serial.println(F("[ACT] no se pudo: hay un ciclo en marcha."));
    }
    return;
  }

  if (strncmp(line, "sm", 2) == 0 && sscanf(line + 2, "%d", &n) == 1) {
    if (n < 0 || n > 255) {
      Serial.println(F("[ACT] duty fuera de 0-255."));
      return;
    }
    actuatorsSetSprayerMinDuty((uint8_t)n);
    Serial.printf("[ACT] duty minimo util = %d (%d %%).\n"
                  "      Mapa de niveles a radio estimado:\n", n, n * 100 / 255);
    for (uint8_t lv = 1; lv <= SPRAYER_LEVEL_MAX; lv++) {
      const uint8_t d = actuatorsDutyForLevel(lv);
      Serial.printf("        nivel %2u -> duty %3u (%3d %%) -> ~%.1f m\n",
                    lv, d, d * 100 / 255, actuatorsRadiusForDuty(d));
    }
    return;
  }

  if (strncmp(line, "sl", 2) == 0 && sscanf(line + 2, "%d", &n) == 1) {
    if (n < 0 || n > SPRAYER_LEVEL_MAX) {
      Serial.printf("[ACT] nivel fuera de 0-%d.\n", SPRAYER_LEVEL_MAX);
      return;
    }
    actuatorsSetSprayerLevel((uint8_t)n);
    Serial.printf("[ACT] nivel %d -> duty %u. Se aplica en el proximo ciclo.\n",
                  n, actuatorsDutyForLevel((uint8_t)n));
    return;
  }

  Serial.println(F("[?] no entiendo. Pulsa '?' para ver los comandos."));
}

static void pollConsole() {
  static char buf[32];
  static uint8_t n = 0;

  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      buf[n] = '\0';
      handleLine(buf);
      n = 0;
      continue;
    }
    if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

/* ------------------------------------------------------------------
 *  Telemetria
 * ------------------------------------------------------------------ */

static void sendTelemetry() {
  TlmPacket tlm;
  memset(&tlm, 0, sizeof(tlm));

  const float t = millis() / 1000.0f;

  /* Temperatura REAL — DS18B20, paso 5a. NAN si la sonda no responde, y
   * entonces Estacion omite la clave en Firebase en vez de escribir un cero
   * que la app confundiria con agua helada. */
  tlm.temperature = sensorTemperature();

  /* pH REAL — paso 5b. NAN mientras no este calibrado: un pH sin calibrar
   * es un voltaje con unidades inventadas, y publicarlo como medida seria
   * peor que no publicar nada. */
  tlm.ph = sensorPh();

  /* VALOR FALSO todavia. El MAX4466 es el paso 5c y aun no ha llegado. */
  tlm.soundLevel = (uint16_t)(400 + 200 * (0.5f + 0.5f * sinf(t / 7.0f)));

  tlm.status = 0;
  if (actuatorsBusy()) { tlm.status |= ST_DOSING; }
  if (navActive)       { tlm.status |= ST_NAV_ACTIVE; }
  if (!sensorTempOk()) { tlm.status |= ST_TEMP_FAULT; }
  if (!sensorPhOk())   { tlm.status |= ST_PH_FAULT; }
  if (thrustersArmed()) { tlm.status |= ST_ESC_ARMED; }

  const bool ok = linkSendTlm(&tlm);
  const LinkStats *s = linkStats();

  /* La temperatura se imprime aparte porque NAN no se ve bien con %.2f y
   * hay que poder distinguir "sin sonda" de un numero cualquiera. */
  char tempTxt[16];
  if (sensorTempOk()) {
    snprintf(tempTxt, sizeof(tempTxt), "%.2f C", tlm.temperature);
  } else {
    snprintf(tempTxt, sizeof(tempTxt), "SIN SONDA");
  }

  char phTxt[24];
  if (sensorPhOk()) {
    snprintf(phTxt, sizeof(phTxt), "pH %.2f", tlm.ph);
  } else {
    snprintf(phTxt, sizeof(phTxt), "pH s/cal %.2fV", sensorPhVolts());
  }

  Serial.printf("[TX tlm  seq=%-5u] %-9s  %-14s  ruido %u  estado 0x%02X  %s\n",
                tlm.hdr.seq, tempTxt, phTxt, tlm.soundLevel, tlm.status,
                ok ? "ok" : "FALLO");

  Serial.printf("          enlace: rx %lu  malos %lu  perdidos %lu  tx %lu/%lu\n",
                (unsigned long)s->rxOk, (unsigned long)s->rxBad,
                (unsigned long)s->lost,
                (unsigned long)s->txOk, (unsigned long)(s->txOk + s->txFail));

  const TempStats *ts = sensorTempStats();
  if (ts->present) {
    Serial.printf("          DS18B20: presente  lecturas %lu  fallos %lu"
                  " (CRC malo %lu, valor 85.00 %lu)  conversion %lu ms\n",
                  (unsigned long)ts->reads, (unsigned long)ts->faults,
                  (unsigned long)ts->crcFails, (unsigned long)ts->resetValues,
                  (unsigned long)ts->convMs);
  } else {
    /* Sin sonda, lo util no es el contador de lecturas sino lo que se ve en
     * el bus: cada combinacion apunta a una causa distinta. */
    const char *hint;
    if (!ts->lineHigh) {
      hint = "falta la pull-up de 4k7 a 3V3, o el amarillo no esta en GPIO7";
    } else if (!ts->presence) {
      hint = "pull-up OK pero nadie contesta: revisa rojo/negro del sensor";
    } else {
      hint = "contesta pero el scratchpad no valida: ruido en la linea";
    }
    Serial.printf("          DS18B20 AUSENTE  linea=%s  presencia=%s  ROM %s (CRC %s)\n"
                  "          -> %s\n",
                  ts->lineHigh ? "ALTA" : "BAJA",
                  ts->presence ? "SI" : "NO",
                  ts->addr, ts->romCrcOk ? "ok" : "malo",
                  hint);
  }
}

/* ------------------------------------------------------------------
 *  Bucle
 * ------------------------------------------------------------------ */

void loop() {
  if (!radioReady) {
    radioRetry();
    return;
  }

  linkPoll();
  sensorsPoll();
  actuatorsPoll();
  thrustersPoll();
  pollConsole();

  CmdPacket cmd;
  if (linkTakeCmd(&cmd)) {
    handleCmd(cmd);
  }

  /*
   * Barrido del aspersor: se imprime cada escalon.
   *
   * Sin esto el barrido sube el duty en silencio y no habria forma de saber
   * en que valor estaba el motor cuando arranco — que es exactamente el dato
   * que el barrido existe para obtener.
   */
  {
    static uint8_t lastSweep = 0;
    static bool    wasSweeping = false;
    const bool sweeping = actuatorsSweeping();

    if (sweeping) {
      const uint8_t d = actuatorsSweepDuty();
      if (d != lastSweep) {
        lastSweep = d;
        Serial.printf("[SW] duty %3u / 255   (%2u %%)\n", d, (unsigned)(d * 100 / 255));
      }
    } else if (wasSweeping) {
      Serial.println(F("[SW] barrido terminado. Guarda el minimo con 'sm <duty>'."));
      lastSweep = 0;
    }
    wasSweeping = sweeping;
  }

  /* Avisa una sola vez cuando el ciclo real termina. */
  {
    static DoseState prev = DOSE_IDLE;
    const DoseState now = actuatorsDoseState();
    if (prev != DOSE_IDLE && now == DOSE_IDLE) {
      const ActStats *as = actuatorsStats();
      Serial.printf("[FEED] ciclo terminado: %.0f g en %lu ms de sinfin\n",
                    as->lastGrams, (unsigned long)as->lastOnMs);
    }
    prev = now;
  }

  /*
   * Deadman de navegacion. Aqui solo se informa; en el paso 7 esta misma
   * condicion es la que devuelve los dos ESC a 1500 us.
   *
   * linkCmdAgeMs() devuelve UINT32_MAX mientras no haya llegado ningun
   * comando, asi que al arrancar el enlace ya cuenta como caducado y la
   * navegacion no puede activarse sola.
   */
  if (navActive && linkCmdAgeMs() > NAV_DEADMAN_MS) {
    navActive = false;
    curNav    = NAV_STOP;
    /* Ya no es un mensaje: esto mueve hardware. Sin rampa a proposito — un
     * deadman que se toma 300 ms en llegar a neutro no es un deadman. */
    thrustersStopNow();
    Serial.printf("[DEADMAN] sin comandos en %d ms -> ESC a neutro\n", NAV_DEADMAN_MS);
  }

  if ((millis() - lastTlmMs) >= TLM_PERIOD_MS) {
    lastTlmMs = millis();
    sendTelemetry();
  }
}
