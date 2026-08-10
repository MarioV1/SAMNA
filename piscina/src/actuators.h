/*
 * actuators.h — PISCINA
 * ------------------------------------------------------------------
 * Dosificador y aspersor, ambos por BTS7960. Paso 6.
 *
 * REGLA: nada aqui bloquea. Un ciclo de dosificacion dura segundos y no
 * puede secuestrar el bucle — la radio tiene que seguir atendiendo comandos
 * mientras el sinfin gira, y la telemetria seguir saliendo.
 *
 * SECUENCIA DE UN CICLO
 *
 *   t=0        aspersor ON al nivel pedido
 *   t=+500ms   sinfin ON, con rampa de FEEDER_RAMP_MS
 *   ...        girando t_on = gramos / m_punto
 *   t=+t_on    sinfin OFF
 *   t=+2s      aspersor OFF
 *
 * El pre-giro no es un capricho: el sinfin deja caer la comida SOBRE el
 * aspersor, y si el disco todavia no ha cogido velocidad los primeros gramos
 * se amontonan bajo el catamaran en vez de repartirse. La cola de 2 s lanza
 * lo que quede encima cuando el sinfin ya paro.
 *
 * UN CICLO EMPEZADO SE TERMINA, aunque se caiga el enlace LoRa. La dosis ya
 * esta acotada por t_on y por MAX_DOSE_MS, asi que no puede desbocarse, y
 * abortarla dejaria una racion parcial de masa desconocida — peor que una
 * completa, porque se pierde la trazabilidad de cuanto comio el camaron. Un
 * sinfin girando no es peligroso como un propulsor; el deadman de 2 s
 * gobierna la navegacion, no esto.
 * ------------------------------------------------------------------
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* SPRAYER_LEVEL_MAX vive en el contrato compartido: el nivel 0-10 llega en
 * el CmdPacket y el mapeo a duty tiene que respetar el mismo tope. */
#include "protocol.h"

/*
 * Guarda de seguridad: ningun ciclo puede pasar de aqui, pase lo que pase.
 *
 * Es un vigilante, no un limite de dosificacion. A un caudal razonable los
 * 100 g que admite el paquete salen en mucho menos, asi que esto solo salta
 * si el sinfin se atasca o si la maquina de estados se queda colgada.
 */
#define MAX_DOSE_MS       60000

/* Pre-giro del aspersor antes de soltar comida, y cola despues de parar. */
#define SPRAY_LEAD_MS     500
#define SPRAY_TAIL_MS     2000

typedef enum {
  DOSE_IDLE = 0,
  DOSE_PRESPIN,    /* aspersor cogiendo velocidad, sinfin parado */
  DOSE_RUNNING,    /* sinfin girando */
  DOSE_TAIL,       /* sinfin parado, aspersor terminando */
} DoseState;

/* Por que se rechazo un ciclo. */
typedef enum {
  DOSE_ACCEPTED = 0,
  DOSE_ERR_BUSY,        /* ya hay uno en curso            */
  DOSE_ERR_NOCAL,       /* m_punto sin calibrar           */
  DOSE_ERR_TOOLONG,     /* t_on pasaria de MAX_DOSE_MS    */
  DOSE_ERR_ZERO,        /* gramos = 0                     */
} DoseResult;

void actuatorsBegin();
void actuatorsPoll();

/*
 * Arranca un ciclo de `grams` gramos.
 *
 * Si el tiempo calculado excediera MAX_DOSE_MS se RECHAZA en vez de
 * recortarlo. Recortar entregaria una racion menor de la pedida informando
 * de exito, que es el peor fallo posible en este sistema: silencioso y con
 * el camaron comiendo de menos.
 */
DoseResult actuatorsStartDose(uint8_t grams);

bool      actuatorsBusy();
DoseState actuatorsDoseState();

/*
 * t_on planificado para el ciclo en curso, compensacion de rampa incluida.
 *
 * Existe para que los mensajes no mientan: recalcular gramos/m_punto por
 * fuera daba un numero distinto del que la maquina iba a usar, y el log
 * decia 1846 ms mientras el sinfin giraba 1946.
 */
uint32_t actuatorsPlannedMs();

/* Nivel del aspersor, 0-10. Se aplica en el proximo ciclo; no arranca el
 * motor por si solo. */
void    actuatorsSetSprayerLevel(uint8_t level);
uint8_t actuatorsSprayerLevel();

/* ------------------------------------------------------------------
 *  Calibracion — en NVS, como la del pH
 * ------------------------------------------------------------------ */

bool  actuatorsCalibrated();
float actuatorsRate();          /* m_punto en g/s, o 0 si sin calibrar */

/*
 * Corre el sinfin `ms` milisegundos con EL MISMO perfil de arranque que usa
 * la produccion, rampa incluida.
 *
 * Que sea el mismo perfil es la parte que importa: los gramos que salen
 * durante la rampa entran en el promedio, asi que calibrar con un arranque
 * distinto del real falsea m_punto desde el primer dia.
 */
bool actuatorsCalRunAuger(uint32_t ms);

/* Tras pesar lo dosificado en la ultima tirada, se le dice cuanto salio y
 * calcula m_punto. */
bool actuatorsCalSetGrams(float grams);

/*
 * Fija m_punto directamente, sin pasar por una tirada.
 *
 * Para cuando el caudal se midio fuera del firmware. Ojo: si esa medida se
 * hizo sin la rampa de arranque, el valor sale optimista y las dosis reales
 * quedaran cortas — mas cuanto mas breve sea el ciclo.
 */
bool actuatorsCalSetRate(float gramsPerSec);

void actuatorsCalReset();

/* ------------------------------------------------------------------
 *  Aspersor: duty minimo UTIL
 *
 *  Hay dos umbrales distintos y el que importa NO es el evidente:
 *
 *    minimo mecanico  por debajo el eje no se mueve. Medido en banco: por
 *                     debajo de 40/255. Es el que se busca por instinto y
 *                     es el equivocado.
 *
 *    minimo UTIL      por debajo el disco gira pero no lanza la comida lo
 *                     bastante lejos para servir de nada. Medido con el
 *                     disco montado: 102/255, el 40 % de duty, que lanza a
 *                     ~1 m.
 *
 *  El nivel 1 se ancla en el UTIL. Anclarlo en el mecanico desperdiciaria
 *  los tres primeros niveles en velocidades que giran sin esparcir.
 * ------------------------------------------------------------------ */

void    actuatorsSetSprayerMinDuty(uint8_t duty);
uint8_t actuatorsSprayerMinDuty();
uint8_t actuatorsDutyForLevel(uint8_t level);

/*
 * Radio de aspersion estimado, en metros, para un duty dado.
 *
 * Medido en banco por Mario el 2026-08-10: al 40 % de duty el disco lanza a
 * ~1 m, y cada 10 % adicional suma ~2 m. Es lo que convierte los niveles de
 * numeros arbitrarios en una magnitud fisica.
 *
 * OJO con el modelo: es LINEAL porque asi se observo, pero el alcance de un
 * proyectil crece con el CUADRADO de la velocidad de salida. Si la
 * observacion se hizo con dos o tres puntos, lo mas probable es que la
 * relacion real sea algo superlineal y que los radios de los niveles altos
 * esten subestimados. Conviene comprobar un par de niveles en campo con
 * comida de verdad antes de dar los metros por buenos en la memoria.
 */
float actuatorsRadiusForDuty(uint8_t duty);

/* Barrido de duty para encontrar el minimo a ojo: sube despacio e informa.
 * Se corta solo, y cualquier otra cosa lo cancela. */
bool actuatorsCalSweepSprayer();
bool actuatorsSweeping();

/*
 * Duty actual del barrido. Hay que publicarlo: sin verlo por serial no hay
 * forma de saber en que valor estaba el motor cuando empezo a girar, que es
 * justo el dato que el barrido existe para obtener.
 */
uint8_t actuatorsSweepDuty();

/*
 * Fija un duty crudo en el aspersor y lo mantiene ahi.
 *
 * Para hallar el minimo de arranque a base de preguntas: se pone un valor,
 * se mira si el eje gira, y se converge. El barrido automatico no sirve
 * cuando quien mira el motor y quien lee el serial no son la misma persona
 * — un puerto solo lo puede abrir un proceso a la vez.
 *
 * ARRANCA DESDE PARADO cada vez: pone 0 un instante antes. Es lo que hace
 * que la medida sea la buena, porque el duty que hace falta para arrancar es
 * mayor que el que basta para seguir girando, y el caso real es siempre
 * partir de parado.
 */
bool actuatorsSetSprayerRaw(uint8_t duty);

/* Para todo AHORA. Para el banco y para emergencias. */
void actuatorsStopAll();

/* ------------------------------------------------------------------
 *  Diagnostico
 * ------------------------------------------------------------------ */

typedef struct {
  uint32_t cycles;        /* ciclos completados            */
  uint32_t rejected;      /* ciclos rechazados             */
  uint32_t guardTrips;    /* veces que salto MAX_DOSE_MS   */
  uint32_t lastOnMs;      /* duracion del ultimo t_on      */
  float    lastGrams;     /* masa del ultimo ciclo         */
  uint8_t  augerDuty;     /* duty actual del sinfin        */
  uint8_t  sprayerDuty;   /* duty actual del aspersor      */
} ActStats;

const ActStats *actuatorsStats();
