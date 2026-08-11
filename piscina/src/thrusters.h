/*
 * thrusters.h — PISCINA
 * ------------------------------------------------------------------
 * Propulsion: dos APISQUEEN U2 MINI brushless, un ESC bidireccional cada
 * uno. Paso 7.
 *
 * SEÑAL: servo de 50 Hz, pulso de 1000 a 2000 us, 1500 = neutro. Va por los
 * canales LEDC 2 y 3 sobre el TIMER 1, reservados desde el paso 2 para no
 * pisar el timer 0 de los actuadores de alimentacion — dos regimenes PWM
 * incompatibles conviviendo en la misma placa.
 *
 * ARMADO. Los ESC exigen ver neutro un rato antes de aceptar nada; aqui son
 * ESC_ARM_MS. Hasta que pasen, cualquier comando de navegacion se ignora.
 *
 * Y una limitacion que conviene tener presente: un ESC no da realimentacion.
 * Podemos garantizar que la señal es correcta, pero NO que el ESC armo. Si
 * uno no arma, el sintoma es que ese motor no responde mientras el otro si.
 *
 * DEADMAN. Quien llama debe pasar NAV_STOP si deja de llegar navegacion. Es
 * requisito de seguridad, no una opcion. Este modulo no mira el reloj del
 * enlace: solo obedece.
 * ------------------------------------------------------------------
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "protocol.h"

/*
 * Empuje de trabajo, en porcentaje sobre el neutro.
 *
 * 60 % significa 1800 us adelante y 1200 atras. Es empuje realista desde la
 * primera prueba: lo que se vea en el banco se parece a lo que hara en el
 * agua. Ajustable en marcha y guardado en NVS.
 */
#define THRUST_DEFAULT_PCT   60

/*
 * Velocidad de la rampa, en microsegundos de pulso por milisegundo.
 *
 * 2 us/ms recorre los 600 us que separan 1800 de 1200 en 300 ms — el tiempo
 * de una inversion completa a 60 %.
 *
 * La rampa NO es comodidad. Un ESC bidireccional que recibe una inversion
 * instantanea tiene que frenar el motor y relanzarlo al reves: es un pico de
 * corriente doble y un esfuerzo mecanico que con dos motores de 8 A puede
 * hundir la bateria, y algunos ESC entran en proteccion y se desarman. Al
 * mando, 300 ms no se notan.
 */
#define THRUST_RAMP_US_PER_MS   2

void thrustersBegin();
void thrustersPoll();

/* true cuando el armado termino y se aceptan comandos. */
bool thrustersArmed();

/*
 * Fija la intencion de navegacion. La señal se mueve hacia ella con rampa;
 * no salta.
 *
 * Antes de que termine el armado esto no hace nada: los ESC siguen viendo
 * neutro, que es justo lo que necesitan para armar.
 */
void thrustersSetNav(NavCmd nav);

/* Ambos a neutro AHORA, sin rampa. Para el deadman y para emergencias. */
void thrustersStopNow();

/* Empuje de trabajo, 0-100 %. Se guarda en NVS. */
void    thrustersSetThrottlePct(uint8_t pct);
uint8_t thrustersThrottlePct();

/*
 * Velocidad de la rampa, en us de pulso por ms. Se guarda en NVS.
 *
 * Existe sobre todo para el osciloscopio: a 2 us/ms una inversion completa
 * son 300 ms, que a 50 Hz son solo 15 pulsos y cuesta capturarlos. Bajandolo
 * a 0.2 la misma inversion dura 3 s y la rampa se ve como una envolvente
 * comoda de medir.
 *
 * Se expresa en decimas de us por ms para no meter flotantes en NVS: 20 son
 * 2.0 us/ms.
 */
void     thrustersSetRampTenths(uint16_t tenthsUsPerMs);
uint16_t thrustersRampTenths();

typedef struct {
  bool     armed;
  uint32_t armLeftMs;    /* lo que falta de armado          */
  uint16_t portUs;       /* pulso actual de babor           */
  uint16_t stbdUs;       /* pulso actual de estribor        */
  uint16_t portTargetUs;
  uint16_t stbdTargetUs;
  uint8_t  nav;          /* NavCmd vigente                  */
  uint32_t stops;        /* veces que se forzo neutro       */
} ThrStats;

const ThrStats *thrustersStats();
