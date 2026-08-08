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

/* ------------------------------------------------------------------
 *  Estado del banco
 * ------------------------------------------------------------------ */

static uint32_t lastTlmMs = 0;

static NavCmd   curNav    = NAV_STOP;
static uint8_t  curPwm    = 0;
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

  Serial.printf("[RX cmd  seq=%-5u] nav=%-11s pwm=%3u%%  RSSI %d dBm  SNR %.1f dB\n",
                cmd.hdr.seq, navName(cmd.nav), cmd.pwm, s->rssi, s->snr);

  if (cmd.nav != (uint8_t)curNav || cmd.pwm != curPwm) {
    curNav = (NavCmd)cmd.nav;
    curPwm = cmd.pwm;
  }
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

  dosing      = true;
  doseStartMs = millis();
  Serial.printf("[FEED seq=%-5u] aceptado: ciclo simulado de %d ms -> ACK_OK\n",
                cmd.hdr.seq, SIM_DOSE_MS);
  linkAckFeed(cmd.hdr.seq, ACK_OK);
}

/* ------------------------------------------------------------------
 *  Telemetria sintetica
 * ------------------------------------------------------------------ */

static void sendTelemetry() {
  TlmPacket tlm;
  memset(&tlm, 0, sizeof(tlm));

  /*
   * VALORES FALSOS. Los sensores reales entran en el paso 5. Se generan con
   * ondas lentas y de periodo distinto para que en el otro extremo se vea
   * que cambian de forma continua: si llegaran escalonados o congelados,
   * el problema estaria en el enlace y no en el sensor.
   */
  const float t = millis() / 1000.0f;
  tlm.temperature = 26.0f + 2.0f * sinf(t / 20.0f);
  tlm.ph          = 7.60f + 0.30f * cosf(t / 31.0f);
  tlm.soundLevel  = (uint16_t)(400 + 200 * (0.5f + 0.5f * sinf(t / 7.0f)));

  tlm.status = 0;
  if (dosing)    { tlm.status |= ST_DOSING; }
  if (navActive) { tlm.status |= ST_NAV_ACTIVE; }
  /* ST_ESC_ARMED se queda en 0: no hay ESC hasta el paso 7.
   * ST_TEMP_FAULT y ST_PH_FAULT tampoco, no hay sensores que fallen. */

  const bool ok = linkSendTlm(&tlm);
  const LinkStats *s = linkStats();

  Serial.printf("[TX tlm  seq=%-5u] %.2f C  pH %.2f  ruido %u  estado 0x%02X  %s\n",
                tlm.hdr.seq, tlm.temperature, tlm.ph, tlm.soundLevel, tlm.status,
                ok ? "ok" : "FALLO");

  Serial.printf("          enlace: rx %lu  malos %lu  perdidos %lu  tx %lu/%lu\n",
                (unsigned long)s->rxOk, (unsigned long)s->rxBad,
                (unsigned long)s->lost,
                (unsigned long)s->txOk, (unsigned long)(s->txOk + s->txFail));
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
