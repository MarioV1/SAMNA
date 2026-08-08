/*
 * pins.h — ESTACION (estacion base)
 * ------------------------------------------------------------------
 * Asignacion de GPIO del Heltec WiFi LoRa 32 V3 (ESP32-S3FN8).
 *
 * Estacion no lleva NINGUN periferico externo: solo radio, WiFi y OLED,
 * y los tres ya estan cableados en la placa. Este archivo existe para
 * dejar constancia de que el pool libre esta intacto y de cuales son los
 * pines intocables, para que nadie tome uno reservado mas adelante.
 *
 * Los pines de la radio SX1262 viven en shared/protocol.h, no aqui.
 * ------------------------------------------------------------------
 */

#pragma once

/* ==================================================================
 *  Pines ocupados por la placa — NO reasignar
 *
 *   8, 9, 10, 11   SPI del SX1262 (NSS, SCK, MOSI, MISO)
 *   12, 13, 14     LoRa RST / BUSY / DIO1
 *   17, 18, 21     OLED SSD1306 (SDA, SCL, RST)
 *   26 - 32        Flash SPI del ESP32-S3FN8
 *   35             LED blanco de placa
 *   36             Control de Vext
 *   37             ADC_Ctrl
 *   39 - 42        JTAG
 *   43, 44         UART0 -> puente CP2102 -> conector USB
 *
 * CORRECCION sobre 19/20 (comprobado en placa el 2026-08-08): aqui ponia
 * que llevaban el USB nativo D-/D+. Es falso. La placa usa un puente
 * CP2102 (VID 0x10C4 / PID 0xEA60) sobre UART0, el USB nativo del ESP32-S3
 * no esta cableado, y 19/20 quedan libres. Ver la nota larga en
 * piscina/include/pins.h.
 *
 * Los nombres Vext, LED, RST_OLED, SDA_OLED, SCL_OLED, SS, MOSI, MISO y
 * SCK ya los declara pins_arduino.h del variante. No se redefinen aqui.
 * ================================================================== */

/* ==================================================================
 *  Pool libre — sin usar
 *
 *   2, 4, 5, 6, 7, 47, 48, 19, 20   (nueve, todos disponibles)
 *
 * Candidatos naturales si hicieran falta:
 *   - pulsador de reaprovisionamiento (forzar el portal SoftAP)
 *   - LED externo de estado del enlace LoRa
 *
 * Al asignar alguno, respetar los criterios de la unidad flotante:
 * evitar los strapping del ESP32-S3 (0, 3, 45, 46) y reservar ADC1
 * (GPIO 1-10) para entradas analogicas, ya que ADC2 es inutilizable
 * mientras el WiFi este activo — y en esta placa lo esta siempre.
 * ================================================================== */

/* Bateria: no aplica. Estacion va alimentada de red. */
