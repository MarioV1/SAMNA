/*
 * lora_link.cpp — PISCINA
 * ------------------------------------------------------------------
 * Ver lora_link.h para el contrato. Aqui solo esta el como.
 * ------------------------------------------------------------------
 */

#include "lora_link.h"

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

/* ==================================================================
 *  Radio
 * ================================================================== */

/*
 * El SPI del SX1262 es el de la placa. Los nombres SS/SCK/MOSI/MISO salen
 * del variante del framework; comprobamos que el CS del variante y el de
 * protocol.h son el mismo pin, porque si algun dia dejan de serlo el
 * sintoma seria "la radio no responde" y no "los pines no cuadran".
 */
static_assert(SS == LORA_PIN_CS, "El CS del variante no coincide con LORA_PIN_CS");

static SX1262 radio = new Module(LORA_PIN_CS, LORA_PIN_DIO1, LORA_PIN_RST, LORA_PIN_BUSY);

/*
 * DIO1 avisa de RxDone y TxDone. La ISR no hace nada mas que levantar la
 * bandera: no se puede llamar a RadioLib desde una interrupcion, porque
 * habla por SPI y eso bloquea.
 */
static volatile bool dio1Flag = false;

static void IRAM_ATTR onDio1() {
  dio1Flag = true;
}

/* ==================================================================
 *  Estado
 * ================================================================== */

/* El paquete mas grande del protocolo es TlmPacket (18 B). 32 da margen
 * para descartar con cabeza cualquier basura mas larga sin desbordar. */
#define LINK_RX_BUF   32

static char       lastError[64] = "";
static LinkStats  stats;

static uint16_t   txSeq       = 0;      /* secuencia propia, para tlm y ack */

static CmdPacket  cmdBox;               /* buzon de un solo hueco */
static bool       cmdPending  = false;

static uint16_t   lastRxSeq   = 0;      /* para contar huecos */
static bool       rxSeqValid  = false;

static uint16_t   lastFeedSeq = 0;      /* dedupe de alimentacion */
static AckResult  lastFeedResult = ACK_OK;
static bool       feedSeqValid = false;
static uint32_t   lastFeedMs   = 0;     /* cuando se atendio; acota el dedupe */

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

  /*
   * CRC de 2 bytes en hardware, explicito.
   *
   * RadioLib ya lo trae activado por defecto, pero esto no puede quedar
   * dependiendo de un valor por defecto: TODA la integridad del enlace
   * descansa aqui. protocol.h no lleva CRC de aplicacion justamente porque
   * este esta puesto. Si alguien lo apaga, los paquetes corruptos empiezan
   * a llegar como validos.
   */
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

/*
 * Transmite y vuelve a dejar la radio escuchando pase lo que pase. Si al
 * fallar un envio nos quedaramos fuera de recepcion, la unidad flotante se
 * volveria sorda y el deadman pararia los propulsores: el modo de fallo
 * seria seguro, pero irrecuperable sin reset.
 */
static bool txPacket(const void *buf, size_t len) {
  int16_t st = radio.transmit((uint8_t *)buf, len);

  /*
   * TxDone tambien dispara DIO1, asi que la bandera queda levantada por un
   * evento que no es un paquete entrante. Se limpia AQUI, con la radio aun
   * en standby: si se limpiara despues de startReceive() podriamos borrar
   * el aviso de un paquete recien llegado y perderlo hasta el siguiente.
   */
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

bool linkSendTlm(TlmPacket *tlm) {
  if (tlm == NULL) {
    return false;
  }
  protoFillHeader(&tlm->hdr, MSG_TLM, txSeq++);
  tlm->_pad = 0;
  return txPacket(tlm, sizeof(*tlm));
}

static bool sendAck(uint16_t ackSeq, AckResult result) {
  AckPacket ack;
  memset(&ack, 0, sizeof(ack));
  protoFillHeader(&ack.hdr, MSG_ACK, txSeq++);
  ack.ackSeq = ackSeq;
  ack.result = (uint8_t)result;
  return txPacket(&ack, sizeof(ack));
}

void linkAckFeed(uint16_t seq, AckResult result) {
  lastFeedSeq    = seq;
  lastFeedResult = result;
  feedSeqValid   = true;
  lastFeedMs     = millis();
  sendAck(seq, result);
}

/* ==================================================================
 *  Recepcion
 * ================================================================== */

void linkPoll() {
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

  /* RSSI y SNR se leen antes de re-armar la escucha: startReceive() los
   * invalida. Se guardan solo si el paquete resulta ser nuestro. */
  int16_t rssi = (int16_t)radio.getRSSI();
  float   snr  = radio.getSNR();

  radio.startReceive();

  if (st != RADIOLIB_ERR_NONE) {
    stats.rxBad++;
    return;
  }

  /* Esta placa solo espera comandos. Telemetria o ACK por aqui significan
   * que estamos oyendo nuestro propio eco o a otra Piscina: se descartan. */
  if (!protoValidate(buf, len, MSG_CMD)) {
    stats.rxBad++;
    return;
  }

  CmdPacket cmd;
  memcpy(&cmd, buf, sizeof(cmd));

  /* Huecos en la secuencia = paquetes que se perdieron por el camino.
   * El primero tras el arranque solo siembra el contador. */
  if (rxSeqValid) {
    stats.lost += protoSeqGap(lastRxSeq, cmd.hdr.seq);
  }
  lastRxSeq  = cmd.hdr.seq;
  rxSeqValid = true;

  stats.rxOk++;
  stats.rssi = rssi;
  stats.snr  = snr;
  stats.lastCmdMs = millis();

  /*
   * Reintento de alimentacion: mismo seq que uno ya atendido.
   *
   * Se responde aqui mismo y se borra el bit feed, de modo que el resto del
   * firmware nunca llega a ver el reintento. Se contesta con el resultado
   * guardado, no con uno nuevo, porque el ciclo real ya se decidio antes:
   *
   *   el original fue ACK_OK   -> ACK_DUPLICATE, "ya salio, no lo repito"
   *   el original fue ACK_BUSY -> ACK_BUSY,      "sigue sin salir"
   *
   * Estacion trata OK y DUPLICATE igual (la racion salio, deja de
   * reintentar) y BUSY como que no salio. La distincion solo sirve para el
   * log de la tesis.
   *
   * El resto del comando (nav, grams) si se entrega: un reintento tambien es
   * un refresco de navegacion perfectamente valido.
   *
   * La comprobacion de tiempo NO es un adorno: sin ella, un reinicio de
   * Estacion devuelve su contador a 0 y el siguiente comando de alimentacion
   * se confunde con uno ya atendido. Piscina contestaria "duplicado" sin
   * dosificar y Estacion lo daria por bueno. Ver FEED_DEDUP_WINDOW_MS.
   */
  if (cmd.feed && feedSeqValid && cmd.hdr.seq == lastFeedSeq) {
    if ((millis() - lastFeedMs) < FEED_DEDUP_WINDOW_MS) {
      sendAck(cmd.hdr.seq, lastFeedResult == ACK_OK ? ACK_DUPLICATE : lastFeedResult);
      cmd.feed = 0;
    } else {
      /* Fuera de plazo: no puede ser un reintento. Es un comando nuevo que
       * casualmente reusa el numero, asi que se olvida lo anterior y se
       * deja pasar. */
      feedSeqValid = false;
    }
  }

  cmdBox     = cmd;
  cmdPending = true;
}

bool linkTakeCmd(CmdPacket *out) {
  if (!cmdPending || out == NULL) {
    return false;
  }
  *out = cmdBox;
  cmdPending = false;
  return true;
}

/* ==================================================================
 *  Diagnostico
 * ================================================================== */

const LinkStats *linkStats() {
  return &stats;
}

uint32_t linkCmdAgeMs() {
  if (!rxSeqValid) {
    return UINT32_MAX;
  }
  return millis() - stats.lastCmdMs;
}
