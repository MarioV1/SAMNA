/*
 * lora_link.h — PISCINA (unidad flotante)
 * ------------------------------------------------------------------
 * Capa de enlace LoRa. Encapsula RadioLib y el contrato de protocol.h.
 *
 * Direccion del trafico en esta placa:
 *   recibe  MSG_CMD  desde Estacion
 *   envia   MSG_TLM  hacia Estacion
 *   envia   MSG_ACK  hacia Estacion (solo alimentacion)
 *
 * No sabe nada de motores, sensores ni Firebase. Solo mueve paquetes.
 *
 * Uso:
 *   linkBegin() una vez en setup().
 *   linkPoll()  a menudo en loop(): no bloquea y no hace nada si no hay
 *               paquete. Todo el trabajo de radio ocurre aqui, la ISR de
 *               DIO1 solo levanta una bandera.
 * ------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>

#include "protocol.h"

/* ------------------------------------------------------------------
 *  Arranque
 * ------------------------------------------------------------------ */

/*
 * Inicializa SPI y el SX1262 con los parametros de protocol.h y deja la
 * radio escuchando. Devuelve false si la radio no responde; en ese caso
 * linkLastError() explica por que.
 */
bool linkBegin();

/* Texto del ultimo fallo de radio, o cadena vacia si no hubo. */
const char *linkLastError();

/* ------------------------------------------------------------------
 *  Bucle
 * ------------------------------------------------------------------ */

/* Atiende la radio. Llamar seguido; no bloquea. */
void linkPoll();

/*
 * Entrega el ultimo comando recibido, si hay uno sin leer, y deja el buzon
 * vacio. Devuelve false si no ha llegado nada nuevo.
 *
 * Solo se guarda el comando MAS RECIENTE: si llegan dos entre dos llamadas,
 * el viejo se pierde. Es lo correcto para navegacion, donde el ultimo manda.
 * Un `feed` perdido asi no es grave: al no confirmarse, Estacion lo reintenta.
 *
 * Los reintentos de alimentacion se filtran aqui dentro. Si el comando trae
 * feed=1 con un seq ya atendido, el enlace responde por su cuenta y entrega
 * el comando con feed en 0: para el resto del firmware ese reintento no
 * existe y el camaron no come dos veces.
 */
bool linkTakeCmd(CmdPacket *out);

/* ------------------------------------------------------------------
 *  Envio
 * ------------------------------------------------------------------ */

/*
 * Transmite telemetria. Rellena la cabecera del paquete (por eso no es
 * const): el numero de secuencia lo lleva el enlace, no quien llama.
 *
 * Bloquea el tiempo de aire del paquete, ~185 ms a SF9. No es evitable ni
 * perjudicial: la radio es half-duplex y tampoco podria recibir mientras
 * transmite.
 */
bool linkSendTlm(TlmPacket *tlm);

/*
 * Confirma un ciclo de alimentacion y registra el seq como atendido, que es
 * lo que permite reconocer los reintentos despues.
 *
 * Llamar SIEMPRE tras un comando con feed=1, tanto si el ciclo arranco
 * (ACK_OK) como si se rechazo por haber otro en curso (ACK_BUSY). Si no se
 * llama, Estacion reintenta hasta agotar FEED_ACK_RETRIES y da la
 * alimentacion por fallida.
 */
void linkAckFeed(uint16_t seq, AckResult result);

/* ------------------------------------------------------------------
 *  Diagnostico — para las pruebas de alcance de la tesis
 * ------------------------------------------------------------------ */

typedef struct {
  uint32_t rxOk;       /* comandos validos recibidos                      */
  uint32_t rxBad;      /* paquetes descartados al validar                 */
  uint32_t lost;       /* acumulado de huecos en el numero de secuencia   */
  uint32_t txOk;       /* transmisiones completadas                       */
  uint32_t txFail;     /* transmisiones fallidas                          */
  uint32_t lastCmdMs;  /* millis() del ultimo comando valido — deadman    */
  int16_t  rssi;       /* dBm del ultimo comando valido                   */
  float    snr;        /* dB  del ultimo comando valido                   */
} LinkStats;

const LinkStats *linkStats();

/*
 * Milisegundos desde el ultimo comando valido. Devuelve UINT32_MAX si no ha
 * llegado ninguno desde el arranque, para que la comparacion contra
 * NAV_DEADMAN_MS de por caducado el enlace de entrada y los ESC se queden en
 * neutro, en vez de parecer recien refrescado.
 */
uint32_t linkCmdAgeMs();
