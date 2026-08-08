/*
 * wifi_portal.h — ESTACION
 * ------------------------------------------------------------------
 * Portal cautivo de aprovisionamiento. Paso 4b.
 *
 * Sirve la pagina donde se teclean SSID y contrasena desde el movil, mas el
 * DNS que hace que el telefono muestre solo el aviso de "iniciar sesion en
 * la red" al conectarse al AP.
 *
 * NO gestiona el AP ni los modos del WiFi: de eso manda wifi_link, que es
 * quien sabe cuando hay que levantarlo y cuando bajarlo. Aqui solo viven el
 * servidor HTTP, el DNS y el HTML.
 *
 * Igual que el resto del paso 4, nada de esto bloquea: portalPoll() atiende
 * lo que haya pendiente y vuelve.
 * ------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>

/* Arranca DNS y servidor HTTP. El AP ya tiene que estar levantado. */
bool portalStart();

/* Para ambos servidores y libera el escaneo pendiente si lo hubiera. */
void portalStop();

/* Atiende peticiones HTTP y DNS. Llamar a menudo mientras este activo. */
void portalPoll();

bool portalActive();

/*
 * Lanza un escaneo de redes ASINCRONO.
 *
 * El escaneo sincrono tarda entre uno y dos segundos con la radio ocupada, y
 * eso son de dos a cuatro refrescos de navegacion perdidos. Asincrono, el
 * bucle sigue girando y los resultados se recogen cuando esten.
 */
void portalStartScan();
