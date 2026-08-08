/*
 * lora_link.cpp — ESTACION
 * ------------------------------------------------------------------
 * Ver lora_link.h para el contrato. Aqui solo esta el como.
 *
 * El arranque de la radio es identico al de piscina/src/lora_link.cpp a
 * proposito: los dos leen los mismos parametros de protocol.h. Si algun dia
 * hay que tocar la secuencia de begin(), hay que tocarla en los dos sitios.
 * ------------------------------------------------------------------
 */

#include "lora_link.h"

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

/* ==================================================================
 *  Radio
 * ================================================================== */

static_assert(SS == LORA_PIN_CS, "El CS del variante no coincide con LORA_PIN_CS");

static SX1262 radio = new Module(LORA_PIN_CS, LORA_PIN_DIO1, LORA_PIN_RST, LORA_PIN_BUSY);

static volatile bool dio1Flag = false;

static void IRAM_ATTR onDio1() {
  dio1Flag = true;
}

/* ==================================================================
 *  Estado
 * ================================================================== */

#define LINK_RX_BUF   32

static char       lastError[64] = "";
static LinkStats  stats;

static uint16_t   txSeq = 0;
static uint32_t   lastTxMs = 0;   /* fin de la ultima transmision, de cualquier tipo */

/* Comando vigente: lo que se manda en cada envio, con o sin feed. */
static NavCmd     curNav     = NAV_STOP;
static uint8_t    curGrams   = 0;
static uint8_t    curSprayer = 0;

/* Telemetria: buzon de un solo hueco. */
static TlmPacket  tlmBox;
static bool       tlmPending = false;

/* Secuencia de Piscina, para contar perdidas. Cuenta telemetria y ACK
 * juntos porque Piscina numera ambos con el mismo contador. */
static uint16_t   lastRxSeq  = 0;
static bool       rxSeqValid = false;

/* Maquina de estados de la alimentacion. */
static FeedState  feedState   = FEED_IDLE;
static AckResult  feedLastAck = ACK_OK;
static uint16_t   feedSeq     = 0;   /* se conserva entre reintentos */
static uint32_t   feedSentMs  = 0;
static uint8_t    feedTries   = 0;

static void setError(const char *what, int16_t code) {
  snprintf(lastError, sizeof(lastError), "%s (RadioLib %d)", what, (int)code);
}

/* ==================================================================
 *  Arranque
 * ================================================================== */

bool linkBegin() {
  memset(&stats, 0, sizeof(stats));
  lastError[0] = '\0';

  SPI.begin(SCK, MISO, MOSI, LORA_PIN_CS);

  int16_t st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                           LORA_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBLE,
                           LORA_TCXO_V, false);
  if (st != RADIOLIB_ERR_NONE) {
    setError("la radio no arranca", st);
    return false;
  }

  /* CRC de 2 bytes explicito: toda la integridad del enlace descansa aqui,
   * y protocol.h no lleva CRC de aplicacion porque este esta puesto. */
  st = radio.setCRC(2);
  if (st != RADIOLIB_ERR_NONE) {
    setError("no se pudo activar el CRC", st);
    return false;
  }

  radio.setDio1Action(onDio1);

  st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) {
    setError("no se pudo poner en escucha", st);
    return false;
  }

  dio1Flag = false;
  return true;
}

const char *linkLastError() {
  return lastError;
}

/* ==================================================================
 *  Transmision
 * ================================================================== */

static bool txPacket(const void *buf, size_t len) {
  int16_t st = radio.transmit((uint8_t *)buf, len);

  /*
   * Se apunta la hora aunque el envio falle: el aire estuvo ocupado igual, y
   * lo que interesa es cuando quedo libre para que conteste el otro extremo.
   */
  lastTxMs = millis();

  /* TxDone tambien levanta DIO1. Se limpia con la radio aun en standby,
   * antes de volver a escuchar, para no borrar el aviso de un paquete
   * entrante. */
  dio1Flag = false;

  int16_t rx = radio.startReceive();
  if (rx != RADIOLIB_ERR_NONE) {
    setError("no se pudo volver a escuchar tras transmitir", rx);
  }

  if (st != RADIOLIB_ERR_NONE) {
    setError("fallo al transmitir", st);
    stats.txFail++;
    return false;
  }

  stats.txOk++;
  return true;
}

/*
 * Construye el paquete desde el comando vigente. Si `seq` es negativo se
 * toma uno nuevo del contador; si no, se reusa el que se pasa — que es como
 * los reintentos de alimentacion conservan su numero de secuencia mientras
 * refrescan la navegacion.
 */
static bool sendCmd(bool feed, int32_t seq) {
  CmdPacket cmd;
  memset(&cmd, 0, sizeof(cmd));
  protoFillHeader(&cmd.hdr, MSG_CMD, (seq < 0) ? txSeq++ : (uint16_t)seq);
  cmd.nav     = (uint8_t)curNav;
  cmd.grams   = curGrams;
  cmd.feed    = feed ? 1 : 0;
  cmd.sprayer = curSprayer;
  return txPacket(&cmd, sizeof(cmd));
}

void linkSetNav(NavCmd nav, uint8_t grams, uint8_t sprayer) {
  curNav     = nav;
  curGrams   = (grams > 100) ? 100 : grams;
  curSprayer = (sprayer > SPRAYER_LEVEL_MAX) ? SPRAYER_LEVEL_MAX : sprayer;
}

bool linkSendCmd() {
  return sendCmd(false, -1);
}

/* ==================================================================
 *  Alimentacion
 * ================================================================== */

bool linkStartFeed() {
  if (feedState == FEED_PENDING) {
    return false;
  }

  feedSeq     = txSeq++;
  feedTries   = 1;
  feedSentMs  = millis();
  feedState   = FEED_PENDING;

  if (!sendCmd(true, (int32_t)feedSeq)) {
    /* No se pudo ni transmitir. Se deja PENDING igualmente: linkPoll()
     * reintentara al vencer el plazo, que es mejor que rendirse al primer
     * fallo de radio. Se devuelve false para que quien llama lo sepa. */
    return false;
  }
  return true;
}

FeedState linkFeedState() {
  return feedState;
}

void linkFeedClear() {
  if (feedState != FEED_PENDING) {
    feedState = FEED_IDLE;
  }
}

AckResult linkFeedLastAck() {
  return feedLastAck;
}

/* Reintentos: los mueve linkPoll(), no un bucle bloqueante. */
static void feedTick() {
  if (feedState != FEED_PENDING) {
    return;
  }
  if (millis() - feedSentMs < FEED_ACK_TIMEOUT_MS) {
    return;
  }

  if (feedTries > FEED_ACK_RETRIES) {
    /*
     * Se agotaron los intentos. Ojo con lo que significa esto: NO significa
     * que la racion no saliera. Puede haber salido y haberse perdido el ACK
     * en el camino de vuelta. Lo honesto es "no se sabe", y por eso el
     * estado se llama FAILED y no algo como NOT_FED.
     */
    feedState = FEED_FAILED;
    return;
  }

  feedTries++;
  feedSentMs = millis();
  stats.feedRetries++;
  sendCmd(true, (int32_t)feedSeq);   /* mismo seq, navegacion fresca */
}

/* ==================================================================
 *  Recepcion
 * ================================================================== */

static void handleAck(const uint8_t *buf) {
  AckPacket ack;
  memcpy(&ack, buf, sizeof(ack));
  stats.rxAck++;

  /* Un ACK de una alimentacion que ya no esperamos es un rezagado: llego
   * despues de darla por fallida. Se ignora en vez de resucitar el estado. */
  if (feedState != FEED_PENDING || ack.ackSeq != feedSeq) {
    return;
  }

  feedLastAck = (AckResult)ack.result;

  switch (feedLastAck) {
    case ACK_OK:
    case ACK_DUPLICATE:
      /* Los dos quieren decir lo mismo para quien llama: la racion salio.
       * DUPLICATE solo aclara que fue un reintento el que llego. */
      feedState = FEED_DONE;
      break;
    case ACK_BUSY:
      feedState = FEED_BUSY;
      break;
    default:
      /* Resultado desconocido: firmware desparejado. Se trata como fallo. */
      feedState = FEED_FAILED;
      break;
  }
}

void linkPoll() {
  feedTick();

  if (!dio1Flag) {
    return;
  }
  dio1Flag = false;

  uint8_t buf[LINK_RX_BUF];
  size_t  len = radio.getPacketLength();

  if (len == 0 || len > sizeof(buf)) {
    stats.rxBad++;
    radio.startReceive();
    return;
  }

  int16_t st = radio.readData(buf, len);

  int16_t rssi = (int16_t)radio.getRSSI();
  float   snr  = radio.getSNR();

  radio.startReceive();

  if (st != RADIOLIB_ERR_NONE) {
    stats.rxBad++;
    return;
  }

  /* Esta placa recibe dos tipos distintos, asi que primero hay que mirar
   * cual es y despues validarlo contra su tamano. */
  MsgType type;
  if (!protoPeekType(buf, len, &type) || !protoValidate(buf, len, type)) {
    stats.rxBad++;
    return;
  }
  if (type == MSG_CMD) {
    /* Los comandos los mandamos nosotros. Si vuelve uno, es eco o hay otra
     * Estacion en el aire. */
    stats.rxBad++;
    return;
  }

  /* Piscina numera telemetria y ACK con el mismo contador, asi que el hueco
   * se mide sobre el flujo combinado y tambien delata los ACK perdidos. */
  const uint16_t seq = protoSeq(buf);
  if (rxSeqValid) {
    stats.lost += protoSeqGap(lastRxSeq, seq);
  }
  lastRxSeq  = seq;
  rxSeqValid = true;

  stats.rssi     = rssi;
  stats.snr      = snr;
  stats.lastRxMs = millis();

  if (type == MSG_ACK) {
    handleAck(buf);
    return;
  }

  memcpy(&tlmBox, buf, sizeof(tlmBox));
  tlmPending = true;
  stats.rxTlm++;
}

bool linkTakeTlm(TlmPacket *out) {
  if (!tlmPending || out == NULL) {
    return false;
  }
  *out = tlmBox;
  tlmPending = false;
  return true;
}

/* ==================================================================
 *  Diagnostico
 * ================================================================== */

const LinkStats *linkStats() {
  return &stats;
}

uint32_t linkLastTxMs() {
  return lastTxMs;
}

uint32_t linkRxAgeMs() {
  if (!rxSeqValid) {
    return UINT32_MAX;
  }
  return millis() - stats.lastRxMs;
}
