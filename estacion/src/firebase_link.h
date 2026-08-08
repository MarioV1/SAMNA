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
  uint8_t grams;     /* /pwm, 0-100                           */
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
} FbStats;

const FbStats *firebaseStats();

/* Ultimo error en texto, o cadena vacia. */
const char *firebaseLastError();
