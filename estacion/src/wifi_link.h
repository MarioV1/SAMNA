/*
 * wifi_link.h — ESTACION
 * ------------------------------------------------------------------
 * Conexion WiFi con memoria de redes y portal de aprovisionamiento.
 * Pasos 4a y 4b.
 *
 * REGLA QUE MANDA SOBRE TODO LO DEMAS AQUI: nada de esto puede bloquear.
 *
 * Estacion refresca la navegacion por LoRa cada NAV_REFRESH_MS (500 ms), y
 * si se pierden cuatro refrescos seguidos el deadman de Piscina devuelve los
 * ESC a neutro y el catamaran se para solo en mitad de una maniobra. Una
 * espera de conexion "solo un par de segundos" dentro del bucle es
 * exactamente eso: ocho refrescos perdidos.
 *
 * Por eso todo es maquina de estados: wifiBegin() lanza el intento y vuelve,
 * y wifiPoll() lo hace avanzar sin esperar a nadie.
 *
 * Las credenciales NO se compilan. Se aprovisionan desde el movil por el
 * portal y viven en NVS. secrets.h solo aporta el nombre y la clave del
 * punto de acceso de configuracion.
 * ------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>

/* Longitudes maximas segun el estandar: SSID 32, clave WPA2 63. +1 por el
 * terminador. */
#define WIFI_SSID_MAX   33
#define WIFI_PASS_MAX   64

/* Cuantas redes se recuerdan. */
#define WIFI_SLOTS      2

typedef enum {
  WIFI_ST_IDLE = 0,   /* sin arrancar                                     */
  WIFI_ST_TRYING,     /* intentando una red concreta                      */
  WIFI_ST_ONLINE,     /* asociado y con IP                                */
  WIFI_ST_BACKOFF,    /* se acabaron los candidatos; esperando otra ronda */
} WifiState;

bool wifiBegin();
void wifiPoll();

WifiState   wifiState();
const char *wifiStateName();
bool        wifiConnected();

/* Red asociada ahora mismo, o cadena vacia. Nunca devuelve la contrasena. */
const char *wifiSsid();
const char *wifiIp();
int8_t      wifiRssi();

/*
 * Prueba unas credenciales recibidas del portal.
 *
 * Se intentan SIN guardarlas. Solo si la asociacion prospera pasan a NVS.
 * Guardar antes de comprobar llenaria la ranura 0 de contrasenas mal
 * tecleadas, y como esa ranura se prueba primero, cada arranque perderia
 * ocho segundos con una red que nunca va a entrar.
 */
bool wifiTryCredentials(const char *ssid, const char *pass);

/*
 * Por que fallo el ultimo intento, en castellano y listo para enseñar en el
 * portal. Cadena vacia si no ha fallado nada desde el arranque.
 */
const char *wifiLastAttemptError();

/* ------------------------------------------------------------------
 *  Redes recordadas
 * ------------------------------------------------------------------ */

bool        wifiSaveNetwork(const char *ssid, const char *pass);
void        wifiForgetAll();
const char *wifiStoredSsid(uint8_t slot);

/* ------------------------------------------------------------------
 *  Portal
 * ------------------------------------------------------------------ */

/* true si el punto de acceso de configuracion esta levantado. */
bool wifiPortalUp();

/*
 * Levanta el portal a mano, sin esperar a que falle una ronda.
 * Para el banco, y para reconfigurar sin tener que apagar el router.
 *
 * Se cierra solo a los 5 minutos SI hay red. Sin red el plazo no corre,
 * porque entonces el portal es la unica via de entrada y cerrarlo dejaria
 * la placa incomunicada.
 */
void wifiForcePortal();

/* Cierra el portal ahora mismo. */
void wifiClosePortal();

const char *wifiApIp();

/*
 * Cuantas veces se agotaron todos los candidatos. Si esto crece y crece,
 * o la contrasena esta mal o la red no esta al alcance: son dos problemas
 * distintos que desde aqui no se distinguen, pero el numero avisa.
 */
uint32_t wifiFailedRounds();
