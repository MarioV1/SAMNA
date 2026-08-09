/*
 * sensors.h — PISCINA
 * ------------------------------------------------------------------
 * Sensores de la unidad flotante. Paso 5.
 *
 * Estado: DS18B20 implementado (5a). El pH es 5b y el MAX4466 5c, que se
 * escribira pero no se podra probar hasta que llegue el microfono.
 *
 * REGLA: nada aqui bloquea. Una conversion del DS18B20 tarda cientos de
 * milisegundos y los comandos de navegacion llegan cada NAV_REFRESH_MS
 * (500 ms). Pedirla en modo bloqueante dejaria a Piscina sorda justo el
 * tiempo suficiente para perder comandos, asi que la conversion se lanza y
 * se recoge despues, en vueltas distintas del bucle.
 *
 * Convenio de fallo: una lectura invalida se devuelve como NAN, nunca como
 * cero ni como el ultimo valor bueno. Estacion omite la clave de Firebase
 * cuando ve NAN, asi que la app distingue "el sensor no responde" de "el
 * agua esta a cero grados". Inventar un numero plausible seria peor que no
 * dar ninguno.
 * ------------------------------------------------------------------
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Arranca el bus 1-Wire y busca la sonda. No bloquea. */
void sensorsBegin();

/* Hace avanzar las conversiones. Llamar a menudo; no bloquea. */
void sensorsPoll();

/* ------------------------------------------------------------------
 *  Temperatura — DS18B20
 * ------------------------------------------------------------------ */

/*
 * Ultima temperatura valida en grados C, o NAN si la sonda no responde o
 * todavia no ha dado una lectura buena.
 */
float sensorTemperature();

/* true si la ultima lectura fue valida. */
bool sensorTempOk();

/* ------------------------------------------------------------------
 *  Diagnostico — para el banco y para la memoria de la tesis
 * ------------------------------------------------------------------ */

typedef struct {
  bool     present;      /* la sonda contesta y su scratchpad valida  */

  /*
   * Estado crudo del bus, para distinguir fallos que desde fuera se parecen
   * pero no tienen nada que ver:
   *
   *   lineHigh=false                 -> no hay pull-up, o datos a masa, o el
   *                                     cable esta en otro pin
   *   lineHigh=true, presence=false  -> pull-up bien, pero nada contesta:
   *                                     sensor muerto o sin alimentar
   *   presence=true, present=false   -> contesta pero el scratchpad no
   *                                     valida: ahi si hay ruido de verdad
   */
  bool     lineHigh;     /* la linea en reposo esta en alto           */
  bool     presence;     /* alguien respondio al pulso de reset       */

  /*
   * CRC del numero de serie de fabrica. Se publica pero NO se usa para
   * decidir nada: la sonda de este montaje lo trae mal grabado y funciona
   * igual. Sirve para no tener que rehacer el diagnostico cada vez que
   * alguien vea el ROM y le extrañe.
   */
  bool     romCrcOk;
  char     addr[24];     /* los 8 bytes del ROM, en hexadecimal       */

  uint32_t reads;        /* lecturas validas                          */
  uint32_t faults;       /* lecturas descartadas                      */
  uint32_t crcFails;     /* de esas, por CRC del scratchpad malo      */
  uint32_t disconnects;  /* veces que la sonda no contesto            */
  uint32_t resetValues;  /* veces que devolvio 85.00 exacto           */
  uint32_t convMs;       /* lo que tardo la ultima conversion         */
} TempStats;

const TempStats *sensorTempStats();
