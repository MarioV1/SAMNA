/*
 * lora_link.h — ESTACION (estacion base)
 * ------------------------------------------------------------------
 * Capa de enlace LoRa. Encapsula RadioLib y el contrato de protocol.h.
 *
 * Direccion del trafico en esta placa:
 *   envia   MSG_CMD  hacia Piscina
 *   recibe  MSG_TLM  desde Piscina
 *   recibe  MSG_ACK  desde Piscina (solo alimentacion)
 *
 * No sabe nada de WiFi ni de Firebase. Solo mueve paquetes.
 *
 * Uso:
 *   linkBegin() una vez en setup().
 *   linkPoll()  a menudo en loop(): no bloquea. Ademas de atender la radio,
 *               es quien hace avanzar los reintentos de alimentacion, asi
 *               que tiene que llamarse aunque no se espere recibir nada.
 * ------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>

#include "protocol.h"

/* ------------------------------------------------------------------
 *  Arranque
 * ------------------------------------------------------------------ */

bool linkBegin();
const char *linkLastError();

/* Atiende la radio y hace avanzar la maquina de estados del ACK. */
void linkPoll();

/* ------------------------------------------------------------------
 *  Comando vigente
 *
 *  El enlace guarda un unico comando "vigente" (navegacion + velocidad del
 *  aspersor) que se actualiza con linkSetNav() y se manda con linkSendCmd().
 *
 *  Se hace asi por los reintentos de alimentacion. Si el paquete de
 *  alimentacion se guardara tal cual para reenviarlo, cada reintento
 *  repetiria la navegacion CONGELADA de hace hasta 800 ms, y el catamaran
 *  daria un tiron en una direccion que el operador ya solto. Reconstruyendo
 *  el paquete desde el comando vigente, el reintento lleva la navegacion
 *  actual y solo el numero de secuencia se conserva.
 * ------------------------------------------------------------------ */

/*
 * Fija la intencion de navegacion y la masa objetivo del proximo ciclo.
 *
 * `grams` son gramos de comida, 0-100. Sale de la clave /pwm de Firebase,
 * que pese al nombre no lleva un porcentaje ni gobierna el aspersor.
 * Los valores por encima de 100 se recortan, que es tambien el tope que
 * valida la app.
 */
void linkSetNav(NavCmd nav, uint8_t grams);

/*
 * Manda el comando vigente con feed=0. Dispara y olvida: la navegacion no
 * lleva ACK porque se refresca cuatro veces por ventana de deadman y una
 * perdida se corrige sola en el siguiente envio.
 *
 * Bloquea el tiempo de aire, ~145 ms a SF9.
 */
bool linkSendCmd();

/* ------------------------------------------------------------------
 *  Alimentacion, con confirmacion
 * ------------------------------------------------------------------ */

typedef enum {
  FEED_IDLE = 0,  /* nada en curso                                        */
  FEED_PENDING,   /* enviado, esperando ACK o reintentando                */
  FEED_DONE,      /* confirmado: la racion salio                          */
  FEED_BUSY,      /* Piscina tenia otro ciclo en curso: NO salio          */
  FEED_FAILED,    /* sin respuesta tras los reintentos: se desconoce      */
} FeedState;

/*
 * Arranca un ciclo de alimentacion: manda el comando vigente con feed=1 y
 * queda pendiente del ACK, reintentando hasta FEED_ACK_RETRIES veces con el
 * MISMO numero de secuencia. Peor caso 4 x FEED_ACK_TIMEOUT_MS = 3.2 s.
 *
 * NO bloquea esos 3.2 s: vuelve enseguida y el trabajo lo va haciendo
 * linkPoll(). Se hizo asi a proposito, porque en el paso 4 este bucle
 * tambien atiende Firebase y no puede quedarse congelado.
 *
 * Devuelve false si ya habia una alimentacion pendiente o si el primer
 * envio fallo.
 */
bool linkStartFeed();

/* Estado de la alimentacion. Los estados terminales se quedan fijos hasta
 * que se llame a linkFeedClear() o se arranque otra alimentacion. */
FeedState linkFeedState();

/* Vuelve a FEED_IDLE tras haber leido un estado terminal. */
void linkFeedClear();

/*
 * Resultado crudo del ultimo ACK recibido. Solo tiene sentido si el estado
 * es FEED_DONE o FEED_BUSY: distingue ACK_OK de ACK_DUPLICATE, que llevan
 * los dos a FEED_DONE. Sirve para el log de la tesis, no para decidir nada.
 */
AckResult linkFeedLastAck();

/* ------------------------------------------------------------------
 *  Telemetria recibida
 * ------------------------------------------------------------------ */

/*
 * Entrega la ultima telemetria sin leer y vacia el buzon. Solo se guarda la
 * mas reciente: si llegan dos entre dos llamadas, la vieja se descarta, que
 * es lo correcto cuando lo que interesa es el valor actual del sensor.
 */
bool linkTakeTlm(TlmPacket *out);

/* ------------------------------------------------------------------
 *  Diagnostico — para las pruebas de alcance de la tesis
 * ------------------------------------------------------------------ */

typedef struct {
  uint32_t rxTlm;      /* telemetrias validas recibidas                   */
  uint32_t rxAck;      /* ACK validos recibidos                           */
  uint32_t rxBad;      /* paquetes descartados al validar                 */
  uint32_t lost;       /* huecos en la secuencia de Piscina               */
  uint32_t txOk;
  uint32_t txFail;
  uint32_t feedRetries;  /* reintentos de alimentacion acumulados         */
  uint32_t lastRxMs;     /* millis() del ultimo paquete valido            */
  int16_t  rssi;         /* dBm del ultimo paquete valido                 */
  float    snr;          /* dB  del ultimo paquete valido                 */
} LinkStats;

const LinkStats *linkStats();

/*
 * millis() de la ultima transmision, sea comando, alimentacion o reintento.
 *
 * Quien programe el refresco de navegacion DEBE contar desde aqui y no desde
 * su propio reloj. Si cuenta aparte, acaba transmitiendo encima de la
 * respuesta que esta esperando: la radio es half-duplex y mientras transmite
 * esta sorda. En el banco eso hizo que se perdiera el ACK de la primera
 * alimentacion y que hiciera falta un reintento para confirmar algo que ya
 * habia salido bien al primer disparo.
 */
uint32_t linkLastTxMs();

/*
 * Milisegundos desde el ultimo paquete valido de Piscina, o UINT32_MAX si
 * no ha llegado ninguno desde el arranque. Sirve para saber si el enlace
 * esta vivo antes de escribir telemetria vieja en Firebase.
 */
uint32_t linkRxAgeMs();
