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

#include "lora_link.h"
#include "pins.h"
#include "sensors.h"

/* ------------------------------------------------------------------
 *  Estado del banco
 * ------------------------------------------------------------------ */

static uint32_t lastTlmMs = 0;

static NavCmd   curNav    = NAV_STOP;
static uint8_t  curGrams  = 0;   /* masa objetivo del proximo ciclo */
static uint8_t  curSprayer = 0;  /* nivel del aspersor, 0-10 — se usa en el paso 6 */
static bool     navActive = false;

/*
 * Dosificacion simulada. En el firmware real este tiempo saldra de
 * t_on = M_objetivo / m_punto (paso 6); aqui es un valor fijo y generoso
 * para que de tiempo a pulsar 'f' dos veces seguidas en el banco de Estacion
 * y ver la respuesta ACK_BUSY.
 */
#define SIM_DOSE_MS   5000

static bool     dosing      = false;
static uint32_t doseStartMs = 0;

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
  /* El nivel se guarda pero todavia no mueve nada: mapearlo a duty y
   * arrancar el aspersor es del paso 6. */
  curSprayer = cmd.sprayer;
  navActive = (curNav != NAV_STOP);

  if (!cmd.feed) {
    return;
  }

  /*
   * Alimentacion. Si ya hay un ciclo en curso se rechaza: la masa se
   * controla con el tiempo, asi que solapar dos ciclos daria una racion
   * imposible de calcular.
   *
   * Los reintentos con seq repetido no llegan hasta aqui — el enlace los
   * filtra y responde por su cuenta.
   */
  if (dosing) {
    Serial.printf("[FEED seq=%-5u] rechazado: ciclo en curso -> ACK_BUSY\n", cmd.hdr.seq);
    linkAckFeed(cmd.hdr.seq, ACK_BUSY);
    return;
  }

  /*
   * Aqui es donde en el paso 6 ira t_on = curGrams / m_punto. De momento la
   * duracion es fija: m_punto todavia no esta calibrado, y inventarme un
   * valor haria que el banco pareciera dosificar bien cuando no mide nada.
   * La racion recibida se imprime para comprobar que llega intacta.
   */
  dosing      = true;
  doseStartMs = millis();
  Serial.printf("[FEED seq=%-5u] aceptado: %u g, ciclo simulado de %d ms -> ACK_OK\n",
                cmd.hdr.seq, curGrams, SIM_DOSE_MS);
  linkAckFeed(cmd.hdr.seq, ACK_OK);
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
  Serial.printf("  Po = %.3f V   pH = ", p->volts);
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
  Serial.println(F("\n  comandos (escribe y pulsa enter):"));
  Serial.println(F("    c1 <ph>   calibrar con UN punto usando el liquido actual"));
  Serial.println(F("    ca <ph>   primer punto de una calibracion de dos"));
  Serial.println(F("    cb <ph>   segundo punto; calcula pendiente y guarda"));
  Serial.println(F("    cr        borrar la calibracion"));
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
      if (fabsf(p->slope) < 40.0f || fabsf(p->slope) > 80.0f) {
        Serial.println(F("     AVISO: la pendiente se aleja mucho de los -59 mV/pH\n"
                         "     teoricos. Revisa los patrones o la sonda."));
      }
    } else {
      Serial.println(F("[pH] falta el primer punto ('ca'), el valor esta fuera\n"
                       "     de 0-14, o los dos patrones estan a menos de 0.5 pH\n"
                       "     y la pendiente no saldria fiable."));
    }
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
  if (dosing)          { tlm.status |= ST_DOSING; }
  if (navActive)       { tlm.status |= ST_NAV_ACTIVE; }
  if (!sensorTempOk()) { tlm.status |= ST_TEMP_FAULT; }
  if (!sensorPhOk())   { tlm.status |= ST_PH_FAULT; }
  /* ST_ESC_ARMED se queda en 0: no hay ESC hasta el paso 7. */

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
  pollConsole();

  CmdPacket cmd;
  if (linkTakeCmd(&cmd)) {
    handleCmd(cmd);
  }

  /* Fin del ciclo de alimentacion simulado. */
  if (dosing && (millis() - doseStartMs) >= SIM_DOSE_MS) {
    dosing = false;
    Serial.println(F("[FEED] ciclo simulado terminado"));
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
    Serial.printf("[DEADMAN] sin comandos en %d ms -> ESC a neutro\n", NAV_DEADMAN_MS);
  }

  if ((millis() - lastTlmMs) >= TLM_PERIOD_MS) {
    lastTlmMs = millis();
    sendTelemetry();
  }
}
