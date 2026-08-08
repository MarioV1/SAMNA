/*
 * pins.h — PISCINA (unidad flotante)
 * ------------------------------------------------------------------
 * Asignacion de GPIO del Heltec WiFi LoRa 32 V3 (ESP32-S3FN8).
 *
 * Verificado contra el pinout oficial de Heltec y contra el variante
 * del framework (framework-arduinoespressif32/variants/
 * heltec_wifi_lora_32_V3/pins_arduino.h).
 *
 * Los pines de la radio SX1262 NO estan aqui: viven en shared/protocol.h,
 * para que ambas placas los tomen de la misma fuente.
 * ------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>

/* ==================================================================
 *  Pines ocupados por la placa — NO reasignar
 *
 *   8, 9, 10, 11   SPI del SX1262 (NSS, SCK, MOSI, MISO)
 *   12, 13, 14     LoRa RST / BUSY / DIO1
 *   17, 18, 21     OLED SSD1306 (SDA, SCL, RST)
 *   19, 20         USB nativo D- / D+  <-- ver nota abajo
 *   26 - 32        Flash SPI del ESP32-S3FN8
 *   35             LED blanco de placa
 *   36             Control de Vext
 *   37             ADC_Ctrl (habilita el divisor de bateria)
 *   39 - 42        JTAG
 *   43, 44         UART0
 *
 * NOTA sobre 19/20: el wiki de Heltec los lista como libres, pero esta
 * placa enumera como USB nativo (VID/PID 0x303A:0x1001) y ambos .ini
 * compilan con ARDUINO_USB_CDC_ON_BOOT=1. Por ahi sale Serial y por ahi
 * se flashea: usarlos como GPIO deja la placa sin monitor y sin carga.
 *
 * Los nombres Vext, LED, RST_OLED, SDA_OLED, SCL_OLED, SS, MOSI, MISO y
 * SCK ya los declara pins_arduino.h. No se redefinen aqui.
 *
 * Pines de proposito general realmente libres en esta placa:
 *   2, 4, 5, 6, 7, 47, 48   (siete)
 * Este proyecto los usa TODOS. No queda ninguno de reserva.
 * ================================================================== */

/* ------------------------------------------------------------------
 *  Sensores
 * ------------------------------------------------------------------ */

/* Sonda de pH B09H1MJS4S — salida analogica 0-14 pH.
 * ADC1_CH4. ADC1 a proposito: ADC2 queda inutilizable si algun dia se
 * enciende el WiFi en esta placa. */
#define PIN_PH_ADC        5

/* Microfono MAX4466 — actividad de alimentacion.
 * ADC1_CH5. Reposo en torno a VCC/2, la senal es alterna alrededor de ahi. */
#define PIN_SOUND_ADC     6

/* DS18B20 — bus 1-Wire.
 * Necesita pull-up de 4.7 kOhm a 3V3. El ESP32-S3 no la trae; va externa. */
#define PIN_ONEWIRE       7

/* ------------------------------------------------------------------
 *  Bateria — divisor ya soldado en la placa
 * ------------------------------------------------------------------ */

/* ADC1_CH0, divisor 390k/100k de fabrica: VBAT = VADC * (100+390)/100.
 * No cuesta un pin del pool libre porque ya viene cableado.
 * Requiere poner PIN_ADC_CTRL en alto antes de leer, y bajarlo despues
 * para no dejar el divisor drenando la bateria. */
#define PIN_VBAT_ADC      1
#define PIN_ADC_CTRL      37

/* ------------------------------------------------------------------
 *  Actuadores de alimentacion — BTS7960, solo RPWM
 *
 *  En ambos modulos: LPWM a GND, R_EN y L_EN a 3V3 fijos.
 *  Los dos EN en alto son obligatorios: con L_EN en bajo el medio puente
 *  izquierdo queda en alta impedancia, el motor se queda sin retorno y
 *  no gira.
 *
 *  Al reset estos pines quedan flotando hasta que el firmware los
 *  configura. Comprobar con ohmetro si el modulo trae pull-down de
 *  fabrica entre RPWM y GND; si no, soldar 4.7 kOhm EN EL MODULO
 *  (no en el lado del ESP32: si el cable se suelta, la resistencia se
 *  va con el y deja la entrada flotando con el motor alimentado).
 * ------------------------------------------------------------------ */

#define PIN_FEEDER_RPWM   47  /* dosificador — sinfin, JGB37-520 60 rpm  */
#define PIN_SPRAYER_RPWM  48  /* aspersor    — JGB37-520 600 rpm         */

/* ------------------------------------------------------------------
 *  Propulsion — senal de servo a los ESC
 *  APISQUEEN U2 MINI, brushless, un ESC bidireccional por propulsor.
 * ------------------------------------------------------------------ */

#define PIN_ESC_PORT      2   /* babor     */
#define PIN_ESC_STBD      4   /* estribor  */

/* ==================================================================
 *  Regimenes PWM — dos distintos, no confundirlos
 *
 *  En el ESP32-S3 los canales LEDC comparten timer de dos en dos:
 *  canales 0-1 usan el timer 0, canales 2-3 el timer 1, y asi. Los
 *  canales que comparten timer DEBEN compartir frecuencia y resolucion.
 *
 *  De ahi este reparto: los dos motores de alimentacion van juntos en el
 *  timer 0 y los dos ESC juntos en el timer 1. Cuatro canales de los ocho
 *  disponibles, dos timers de los cuatro. Asignacion explicita a proposito:
 *  dejar que una libreria reclame timers por su cuenta es lo que hace que
 *  los ESC reprogramen el timer de los motores de alimentacion.
 * ================================================================== */

/* --- Motores de alimentacion: LEDC ~1 kHz, 8 bits (timer 0) --- */
#define FEED_PWM_FREQ_HZ    1000
#define FEED_PWM_RES_BITS   8
#define FEED_PWM_MAX        255

#define LEDC_CH_FEEDER      0
#define LEDC_CH_SPRAYER     1

/* El dosificador solo usa 0 % o 100 %: la masa se controla con el tiempo,
 * nunca con la velocidad. Se queda en LEDC en vez de digitalWrite para
 * conservar la rampa de arranque — al 100 % de duty el pin queda en alto
 * constante, electricamente identico.
 *
 * La rampa evita la corriente de arranque completa y el golpe seco a la
 * reductora. Calibrar m_punto con esta misma rampa: los gramos que salen
 * durante la subida entran en el promedio. */
#define FEEDER_RAMP_MS      200

/* --- ESC de propulsion: 50 Hz, 16 bits (timer 1) --- */
#define ESC_PWM_FREQ_HZ     50
#define ESC_PWM_RES_BITS    16

#define LEDC_CH_ESC_PORT    2
#define LEDC_CH_ESC_STBD    3

#define ESC_US_MIN          1000
#define ESC_US_NEUTRAL      1500
#define ESC_US_MAX          2000

/* Armado al arranque: mantener neutro este tiempo antes de aceptar
 * cualquier comando de navegacion. Requisito de los ESC. */
#define ESC_ARM_MS          2000

/*
 * Conversion de microsegundos a cuentas de duty.
 * A 50 Hz el periodo es 20000 us y la escala completa de 16 bits son
 * 65536 cuentas, asi que el paso es 0.305 us — mejor resolucion que la
 * que da ESP32Servo.
 *
 * constexpr: los valores fijos (neutro, minimo, maximo) quedan plegados
 * en tiempo de compilacion, sin division en ejecucion.
 */
static constexpr uint32_t escMicrosToDuty(uint32_t micros) {
  return (micros * (1UL << ESC_PWM_RES_BITS)) / (1000000UL / ESC_PWM_FREQ_HZ);
}
