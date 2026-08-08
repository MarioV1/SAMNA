/*
 * protocol.h
 * ------------------------------------------------------------------
 * Contrato de mensajes del enlace LoRa 915 MHz entre Piscina y Estacion.
 *
 * Incluido por AMBOS proyectos. Cualquier cambio aqui obliga a recompilar
 * y reflashear las dos placas: no hay negociacion de version en caliente,
 * solo rechazo del paquete.
 *
 * Codificacion: structs binarios de tamano fijo, empaquetados, little-endian.
 * Ambos extremos son ESP32-S3 con el mismo compilador, asi que el layout
 * es identico. No usar este header en un tercer equipo sin revisarlo.
 *
 * Integridad: se confia en el CRC de hardware del SX1262, que descarta los
 * paquetes corruptos antes de entregarlos. La validacion de aplicacion
 * (magic + version + tipo + longitud) solo filtra trafico ajeno o firmware
 * desparejado.
 * ------------------------------------------------------------------
 */

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* ==================================================================
 *  Parametros de radio — IDENTICOS en ambos lados
 *  Viven aqui a proposito: que un solo archivo los defina hace
 *  imposible que las placas queden desparejadas.
 * ================================================================== */

#define LORA_FREQ_MHZ    915.0f
#define LORA_BW_KHZ      125.0f
#define LORA_SF          9
#define LORA_CR          5      /* 4/5 */
#define LORA_TX_DBM      22
#define LORA_PREAMBLE    8
#define LORA_SYNC_WORD   0x34   /* privado; 0x34 es el valor por defecto */

/* Pines SX1262 del Heltec WiFi LoRa 32 V3 — restriccion fija de hardware */
#define LORA_PIN_CS      8
#define LORA_PIN_DIO1    14
#define LORA_PIN_RST     12
#define LORA_PIN_BUSY    13

/* ==================================================================
 *  Cabecera comun
 * ================================================================== */

#define PROTO_MAGIC      0xA5C3
#define PROTO_VERSION    1

/* Tipos de mensaje. El sentido es fijo: cada tipo viaja en una direccion. */
enum MsgType : uint8_t {
  MSG_CMD = 1,  /* Estacion  -> Piscina */
  MSG_TLM = 2,  /* Piscina   -> Estacion */
};

typedef struct __attribute__((packed)) {
  uint16_t magic;    /* PROTO_MAGIC */
  uint8_t  version;  /* PROTO_VERSION */
  uint8_t  type;     /* MsgType */
  uint16_t seq;      /* incremental por emisor; da la vuelta en 65535 */
} MsgHeader;

/* ==================================================================
 *  Comando — Estacion -> Piscina
 * ================================================================== */

/*
 * Navegacion mutuamente excluyente. Estacion lee las cuatro banderas de
 * navegacion de Firebase, resuelve cualquier combinacion contradictoria y
 * manda UNA sola intencion. Piscina no puede recibir "adelante y atras".
 *
 * Correspondencia con la mezcla de propulsores:
 *
 *   comando       babor      estribor
 *   NAV_FORWARD   adelante   adelante
 *   NAV_REVERSE   atras      atras
 *   NAV_CW        adelante   atras
 *   NAV_CCW       atras      adelante
 */
enum NavCmd : uint8_t {
  NAV_STOP    = 0,  /* ambos ESC a neutro */
  NAV_FORWARD = 1,  /* Firebase: /nav/adelante */
  NAV_REVERSE = 2,  /* Firebase: /nav/atras    */
  NAV_CW      = 3,  /* Firebase: /nav/ho       — giro horario     */
  NAV_CCW     = 4,  /* Firebase: /nav/aho      — giro antihorario */
};

typedef struct __attribute__((packed)) {
  MsgHeader hdr;
  uint8_t   nav;   /* NavCmd */
  uint8_t   pwm;   /* 0-100 % — velocidad del ASPERSOR unicamente.
                    * El dosificador no usa PWM: corre siempre al 100 %. */
  uint8_t   feed;  /* 1 = disparar un ciclo de alimentacion (flanco) */
  uint8_t   _pad;  /* alineacion; debe ir en 0 */
} CmdPacket;

/*
 * La masa objetivo de dosificacion NO viaja en el paquete: el esquema
 * Firebase solo expone /motores y /pwm, asi que M_objetivo es constante
 * de firmware en Piscina y t_on = M_objetivo / m_punto se calcula alli.
 *
 * `feed` es un disparo, no un estado: Piscina ejecuta un ciclo por cada
 * transicion a 1 y lo ignora mientras ya haya un ciclo en curso.
 */

/* ==================================================================
 *  Telemetria — Piscina -> Estacion
 * ================================================================== */

/* Bits de `status`. */
#define ST_DOSING      (1u << 0)  /* ciclo de alimentacion en curso */
#define ST_ESC_ARMED   (1u << 1)  /* ESC pasaron el armado de arranque */
#define ST_NAV_ACTIVE  (1u << 2)  /* propulsores fuera de neutro */
#define ST_TEMP_FAULT  (1u << 3)  /* DS18B20 no responde */
#define ST_PH_FAULT    (1u << 4)  /* lectura de pH fuera de rango fisico */

typedef struct __attribute__((packed)) {
  MsgHeader hdr;
  float     temperature;  /* DS18B20, grados C. NAN si el sensor fallo */
  float     ph;           /* sonda de pH, 0-14.  NAN si el sensor fallo */
  uint16_t  soundLevel;   /* MAX4466, cuentas del ADC */
  uint8_t   status;       /* bitfield ST_* */
  uint8_t   _pad;         /* alineacion; debe ir en 0 */
} TlmPacket;

/*
 * NAN como centinela de sensor caido: no inventa un valor plausible y
 * Estacion puede omitir esa clave del multi-path update en vez de
 * escribir basura en Firebase.
 */

/* ==================================================================
 *  Tamanos — congelados a proposito
 *  Sin CRC de aplicacion, un desajuste de layout entre las dos placas
 *  no se detecta en vuelo. Estas comprobaciones lo detectan al compilar.
 * ================================================================== */

static_assert(sizeof(MsgHeader) == 6,  "MsgHeader debe medir 6 bytes");
static_assert(sizeof(CmdPacket) == 10, "CmdPacket debe medir 10 bytes");
static_assert(sizeof(TlmPacket) == 18, "TlmPacket debe medir 18 bytes");

/* ==================================================================
 *  Temporizacion del enlace
 * ================================================================== */

/*
 * Deadman de navegacion: si Piscina no recibe un CmdPacket valido en esta
 * ventana, devuelve ambos ESC a 1500 us neutro. Requisito de seguridad.
 */
#define NAV_DEADMAN_MS   2000

/*
 * Cadencia de refresco del comando desde Estacion mientras haya navegacion
 * activa. Cuatro refrescos por ventana de deadman: toleramos tres perdidas
 * seguidas antes de que los propulsores se detengan solos.
 *
 * A SF9/BW125/CR4-5 un CmdPacket de 10 B ocupa el aire ~145 ms, asi que
 * este periodo deja el enlace ocupado cerca del 30 % durante la navegacion.
 * No bajarlo sin recalcular: el enlace es half-duplex y Piscina no puede
 * recibir mientras transmite telemetria.
 */
#define NAV_REFRESH_MS   500

/* ==================================================================
 *  Ayudas de serializacion
 * ================================================================== */

/* Rellena la cabecera de un paquete saliente. */
static inline void protoFillHeader(MsgHeader *hdr, MsgType type, uint16_t seq) {
  hdr->magic   = PROTO_MAGIC;
  hdr->version = PROTO_VERSION;
  hdr->type    = (uint8_t)type;
  hdr->seq     = seq;
}

/*
 * Valida un buffer recibido: comprueba longitud, magic, version y tipo.
 * Devuelve true si el buffer se puede castear al struct correspondiente.
 */
static inline bool protoValidate(const uint8_t *buf, size_t len, MsgType expected) {
  const size_t want = (expected == MSG_CMD) ? sizeof(CmdPacket) : sizeof(TlmPacket);
  if (buf == NULL || len != want) {
    return false;
  }
  MsgHeader hdr;
  memcpy(&hdr, buf, sizeof(hdr));
  return hdr.magic == PROTO_MAGIC
      && hdr.version == PROTO_VERSION
      && hdr.type == (uint8_t)expected;
}

/*
 * Paquetes perdidos entre dos numeros de secuencia consecutivos, con la
 * vuelta de 16 bits ya contemplada. Para el ensayo de alcance de la tesis:
 * el receptor acumula esto y lo saca por serial junto a RSSI y SNR, sin
 * tocar el esquema de Firebase.
 */
static inline uint16_t protoSeqGap(uint16_t last, uint16_t current) {
  return (uint16_t)(current - last - 1);
}
