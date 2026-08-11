/*
 * firebase_link.h — ESTACION
 * ------------------------------------------------------------------
 * Lectura de comandos desde Realtime Database. Paso 4c.
 *
 * El esquema es PLANO a proposito: la app Android lee estas claves de raiz
 * directamente. No crear nodos anidados ni añadir claves sin acordarlo.
 *
 *   /motores    bool   dispara un ciclo de alimentacion
 *   /pwm        int    gramos de comida, 0-100  (nombre historico)
 *   /aspersor   int    nivel de velocidad del aspersor, 0-10
 *   /nav/adelante /nav/atras /nav/ho /nav/aho   bool
 *
 * QUE SE BORRA Y QUE NO — comprobado leyendo la app:
 *
 *   /motores se pone en false en cuanto se lee, porque es un flanco y si no
 *   se re-dispararia en cada vuelta.
 *
 *   Las banderas de navegacion NO se tocan. La app las mantiene en true
 *   mientras el dedo esta sobre el mando y escribe las cuatro en false al
 *   soltar. Borrarlas desde aqui convertiria la navegacion sostenida en
 *   tironcitos de medio segundo.
 *
 * Igual que el resto del paso 4, nada de esto bloquea el bucle mas de lo
 * imprescindible: se usa un stream, no un sondeo, y se atiende con
 * readStream() desde firebasePoll().
 * ------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>

#include "protocol.h"

/* Estado de los comandos, tal y como estan en la base ahora mismo. */
typedef struct {
  NavCmd  nav;       /* resuelto desde las cuatro banderas    */
  uint16_t grams;    /* /pwm, en gramos, hasta GRAMS_MAX        */
  uint8_t sprayer;   /* /aspersor, 0-10                       */
  bool    feed;      /* /motores, ya consumido si estaba true */
} FbCommands;

/*
 * Arranca el cliente y abre el stream. No espera a que conecte: si el WiFi
 * todavia no esta listo, firebasePoll() lo reintenta.
 */
bool firebaseBegin();

/* Atiende el stream y los reintentos. Llamar a menudo. */
void firebasePoll();

/* true si el stream esta vivo y llegando datos. */
bool firebaseReady();

/* Estado actual de los comandos. Siempre valido; arranca todo a cero. */
const FbCommands *firebaseCommands();

/*
 * true una sola vez por cada flanco de /motores.
 *
 * Consumirlo tambien lanza el borrado de la clave en la base. Se devuelve
 * como flanco y no como estado para que quien llama no pueda disparar dos
 * ciclos con la misma orden.
 */
bool firebaseTakeFeed();

/*
 * true una sola vez por cada flanco de /paro. Consumirlo tambien lanza el
 * borrado de la clave, igual que /motores.
 *
 * Existe como clave propia y no se deduce de /motores puesto a false porque
 * Estacion pone /motores a false ella misma en cuanto lo lee: ese false es
 * indistinguible del que escribiria la app al pulsar PARO. Sin una clave
 * aparte, el paro es invisible para Piscina.
 */
bool firebaseTakeStop();

/* ------------------------------------------------------------------
 *  Telemetria hacia la base
 * ------------------------------------------------------------------ */

/*
 * Escribe la telemetria en UN SOLO multi-path update sobre la raiz.
 *
 *   /temperatura   float
 *   /ph            float
 *   /nivelSonido   int
 *
 * Una sola escritura y no tres: tres peticiones son tres viajes de red, y
 * ademas la app podria leer un estado a medias, con la temperatura nueva y
 * el pH viejo. Con updateNode sobre la raiz solo se tocan estas claves; los
 * comandos (/motores, /pwm, /aspersor, /nav) no se rozan.
 *
 * Las claves cuyo valor venga en NAN se OMITEN. Piscina manda NAN cuando un
 * sensor no responde, y escribir un cero seria peor que no escribir nada:
 * la app no distinguiria "el agua esta a 0 grados" de "la sonda esta rota",
 * y el ultimo valor bueno que quedo en la base es mas util que un cero
 * inventado.
 *
 * Devuelve false si no habia nada que escribir, si no toca todavia por
 * ritmo, o si la escritura fallo.
 */
bool firebaseWriteTlm(const TlmPacket *tlm);

/*
 * Enciende o apaga la escritura de telemetria.
 *
 * Existe para poder aislar la escritura del stream durante las pruebas: si
 * el stream deja de caerse con esto apagado, es que las dos conexiones se
 * estorban. Sin un interruptor habria que reflashear para comprobarlo.
 */
void firebaseSetTlmEnabled(bool on);
bool firebaseTlmEnabled();

/* ------------------------------------------------------------------
 *  Diagnostico
 * ------------------------------------------------------------------ */

/* A partir de aqui una llamada se considera lenta y se cuenta aparte. */
#define FB_SLOW_CALL_MS   200

typedef struct {
  uint32_t events;      /* eventos de stream recibidos            */
  uint32_t errors;      /* fallos de stream o de escritura        */
  uint32_t reconnects;  /* veces que hubo que reabrir el stream   */
  uint32_t lastEventMs; /* millis() del ultimo evento util        */
  uint32_t calls;       /* llamadas a la libreria                 */
  uint32_t slowCalls;   /* de esas, cuantas pasaron de FB_SLOW_CALL_MS */
  uint32_t maxCallMs;   /* lo mas que ha tardado una llamada suya */
  uint32_t totalCallMs; /* suma, para sacar la media              */
  uint32_t writes;      /* telemetrias escritas con exito         */
  uint32_t writeFails;  /* escrituras que fallaron                */
  uint32_t skippedNan;  /* claves omitidas por venir en NAN       */
} FbStats;

const FbStats *firebaseStats();

/* Ultimo error en texto, o cadena vacia. */
const char *firebaseLastError();

/*
 * JSON crudo de la primera lectura del arbol, recortado.
 *
 * Para diagnostico: enseña lo que hay DE VERDAD en la base, en vez de
 * obligar a deducirlo a partir de que los comandos valgan cero.
 */
const char *firebaseInitialDump();

/* Estado crudo de la conexion del stream, para diagnostico. */
void firebaseStreamDebug(char *out, size_t n);
