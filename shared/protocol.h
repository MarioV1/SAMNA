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
/*
 * Sync word PRIVADO. Ojo con este valor:
 *   0x12 = privado          (RADIOLIB_SX126X_SYNC_WORD_PRIVATE, por defecto)
 *   0x34 = publico, LoRaWAN (RADIOLIB_SX126X_SYNC_WORD_PUBLIC)
 *
 * Queremos el privado. Con 0x34 la radio se despertaria con cada trama
 * LoRaWAN de la zona: el filtro de magic las tiraria igual, pero son
 * colisiones y consumo regalados en una unidad que va a bateria.
 */
#define LORA_SYNC_WORD   0x12

/* Pines SX1262 del Heltec WiFi LoRa 32 V3 — restriccion fija de hardware */
#define LORA_PIN_CS      8
#define LORA_PIN_DIO1    14
#define LORA_PIN_RST     12
#define LORA_PIN_BUSY    13

/*
 * Voltaje del TCXO que el SX1262 alimenta por su DIO3.
 * 1.6 V es el valor por defecto de RadioLib y el que usa la libreria
 * comunitaria de referencia del Heltec V3 (ropg/heltec_esp32_lora_v3), que
 * ni siquiera lo pasa. Aqui va explicito para que se vea de donde sale.
 *
 * Si begin() devuelve error de SPI o de chip en placa, esto es lo primero
 * que hay que probar a 1.8: algunas revisiones montan otro TCXO.
 */
#define LORA_TCXO_V      1.6f

/* ==================================================================
 *  Cabecera comun
 * ================================================================== */

#define PROTO_MAGIC      0xA5C3
#define PROTO_VERSION    1

/* Tipos de mensaje. El sentido es fijo: cada tipo viaja en una direccion. */
enum MsgType : uint8_t {
  MSG_CMD = 1,  /* Estacion  -> Piscina */
  MSG_TLM = 2,  /* Piscina   -> Estacion */
  MSG_ACK = 3,  /* Piscina   -> Estacion — confirma un CmdPacket con feed=1 */
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
  uint8_t   nav;    /* NavCmd */
  uint8_t   grams;  /* masa objetivo del ciclo, 0-100 g */
  uint8_t   feed;   /* 1 = disparar un ciclo de alimentacion (flanco) */
  uint8_t   _pad;   /* alineacion; debe ir en 0 */
} CmdPacket;

/*
 * Sobre `grams` — ojo con la clave de Firebase de la que sale.
 *
 * Llega en `/pwm`, que es un nombre historico: la clave NO transporta un
 * porcentaje ni gobierna el aspersor. La app Android escribe ahi los gramos
 * de comida del ciclo (niveles de 60, 70, 80 y 100 g, mas un campo libre con
 * tope de 100). Comprobado leyendo controlActivity.java el 2026-08-08.
 *
 * Asi que la masa objetivo SI viaja en el paquete y Piscina calcula
 * t_on = grams / m_punto con el valor recibido, en lugar de con una
 * constante compilada. El tope de 100 g de la app es justo lo que cabe en
 * este uint8_t.
 *
 * El aspersor corre a velocidad fija de firmware: no tiene campo aqui.
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
 *  Confirmacion — Piscina -> Estacion
 *
 *  SOLO se confirma la alimentacion. Navegacion y telemetria van sin ACK:
 *  la navegacion se refresca cuatro veces por ventana de deadman, asi que
 *  una perdida se corrige sola 500 ms despues, y una muestra de telemetria
 *  perdida la reemplaza la siguiente.
 *
 *  `feed` es la excepcion porque es un flanco, no un estado: si ese paquete
 *  se pierde, la racion no sale y nadie se entera.
 * ================================================================== */

enum AckResult : uint8_t {
  ACK_OK        = 0,  /* ciclo aceptado y arrancado */
  ACK_BUSY      = 1,  /* ya habia un ciclo en curso; el comando se ignoro */
  ACK_DUPLICATE = 2,  /* seq ya atendido: es un reintento, NO se re-alimenta */
};

typedef struct __attribute__((packed)) {
  MsgHeader hdr;
  uint16_t  ackSeq;  /* seq del CmdPacket que se confirma */
  uint8_t   result;  /* AckResult */
  uint8_t   _pad;    /* alineacion; debe ir en 0 */
} AckPacket;

/*
 * REGLA QUE HACE QUE ESTO FUNCIONE — no romperla al implementar:
 *
 *   Los reintentos de un comando de alimentacion reusan EL MISMO `seq`.
 *
 * Piscina guarda el ultimo seq de alimentacion que atendio. Si le vuelve a
 * llegar ese seq, NO alimenta otra vez, pero SI reenvia el ACK. Asi el caso
 * incomodo — la racion ya salio pero el ACK se perdio — acaba en un
 * ACK_DUPLICATE y no en una doble dosificacion.
 *
 * Si los reintentos llevaran seq nuevo, cada uno seria un comando distinto
 * y el camaron comeria tres veces.
 */

/* ==================================================================
 *  Tamanos — congelados a proposito
 *  Sin CRC de aplicacion, un desajuste de layout entre las dos placas
 *  no se detecta en vuelo. Estas comprobaciones lo detectan al compilar.
 * ================================================================== */

static_assert(sizeof(MsgHeader) == 6,  "MsgHeader debe medir 6 bytes");
static_assert(sizeof(CmdPacket) == 10, "CmdPacket debe medir 10 bytes");
static_assert(sizeof(TlmPacket) == 18, "TlmPacket debe medir 18 bytes");
static_assert(sizeof(AckPacket) == 10, "AckPacket debe medir 10 bytes");

/*
 * AckPacket y CmdPacket miden lo mismo, pero no se confunden: viajan en
 * sentidos opuestos y el campo `type` de la cabecera los separa. Aun asi
 * protoValidate() exige tipo Y longitud, nunca solo la longitud.
 */

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

/*
 * Cadencia de telemetria de Piscina.
 *
 * Un TlmPacket de 18 B ocupa el aire ~185 ms, asi que a 2 s la telemetria
 * usa el ~9 % del canal; con la navegacion activa el total ronda el 38 %.
 *
 * Nadie coordina las dos radios, de modo que a veces Piscina transmite justo
 * cuando Estacion manda un comando y ese comando se pierde (~16 % de ellos
 * durante la navegacion). No es un problema de seguridad: la rafaga dura
 * 185 ms y los comandos van cada 500 ms, asi que una colision puede tumbar
 * como mucho UN comando, y el deadman necesita cuatro perdidas seguidas.
 */
#define TLM_PERIOD_MS    2000

/*
 * Ventana de espera del ACK de alimentacion.
 * Ida 145 ms + vuelta 145 ms + turnaround de la radio y margen.
 */
#define FEED_ACK_TIMEOUT_MS   800

/*
 * Reintentos antes de dar la alimentacion por fallida. Peor caso 4 x 800 ms
 * = 3.2 s desde el disparo hasta el veredicto.
 * Recordatorio: los reintentos van con el MISMO seq. Ver la nota de AckPacket.
 */
#define FEED_ACK_RETRIES      3

/*
 * Cuanto tiempo un seq repetido sigue contando como reintento.
 *
 * NO sobra, y esto costo una prueba de banco entenderlo. El filtro de
 * duplicados se apoya en el seq, pero el seq de Estacion vuelve a 0 cuando
 * Estacion se reinicia. Sin ventana temporal pasa esto:
 *
 *   1. Se alimenta con seq=N. Piscina guarda N como atendido.
 *   2. Estacion se reinicia (microcorte, reset, lo que sea) y su contador
 *      vuelve a empezar.
 *   3. El siguiente comando de alimentacion vuelve a llevar seq=N.
 *   4. Piscina lo toma por un reintento, responde ACK_DUPLICATE y NO dosifica.
 *   5. Estacion lo da por bueno. El camaron no comio y nadie se entero.
 *
 * Reproducido en banco el 2026-08-08.
 *
 * Un reintento real nunca puede tardar mas que el ultimo plazo de espera,
 * asi que se deriva de los propios parametros de reintento en vez de fijar
 * un numero suelto: si manana cambian, esto se ajusta solo.
 */
#define FEED_DEDUP_WINDOW_MS  (FEED_ACK_TIMEOUT_MS * (FEED_ACK_RETRIES + 2))

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
static inline size_t protoSizeOf(MsgType type) {
  switch (type) {
    case MSG_CMD: return sizeof(CmdPacket);
    case MSG_TLM: return sizeof(TlmPacket);
    case MSG_ACK: return sizeof(AckPacket);
  }
  return 0;
}

static inline bool protoValidate(const uint8_t *buf, size_t len, MsgType expected) {
  const size_t want = protoSizeOf(expected);
  if (buf == NULL || want == 0 || len != want) {
    return false;
  }
  MsgHeader hdr;
  memcpy(&hdr, buf, sizeof(hdr));
  return hdr.magic == PROTO_MAGIC
      && hdr.version == PROTO_VERSION
      && hdr.type == (uint8_t)expected;
}

/*
 * Lee el tipo de un buffer recibido sin saber de antemano cual esperabamos.
 * Hace falta en Estacion, que recibe telemetria y ACK por el mismo camino:
 * primero mira el tipo con esto, luego valida con protoValidate().
 *
 * Devuelve false si el buffer ni siquiera tiene cabecera o si el magic o la
 * version no cuadran — trafico ajeno o firmware desparejado.
 */
static inline bool protoPeekType(const uint8_t *buf, size_t len, MsgType *out) {
  if (buf == NULL || out == NULL || len < sizeof(MsgHeader)) {
    return false;
  }
  MsgHeader hdr;
  memcpy(&hdr, buf, sizeof(hdr));
  if (hdr.magic != PROTO_MAGIC || hdr.version != PROTO_VERSION) {
    return false;
  }
  *out = (MsgType)hdr.type;
  return protoSizeOf(*out) != 0;
}

/*
 * Numero de secuencia de un buffer ya validado. Atajo para no castear el
 * struct completo cuando solo interesa el seq (contador de perdidas, ACK).
 */
static inline uint16_t protoSeq(const uint8_t *buf) {
  MsgHeader hdr;
  memcpy(&hdr, buf, sizeof(hdr));
  return hdr.seq;
}

/*
 * Salto maximo que se acepta como perdida real.
 *
 * A la cadencia mas rapida del sistema, 1000 paquetes son mas de ocho
 * minutos de enlace. Un hueco asi no es una rafaga de perdidas: es que el
 * enlace estuvo caido, y contarlo como paquetes perdidos no mide nada.
 */
#define PROTO_SEQ_GAP_MAX   1000

/*
 * Paquetes perdidos entre dos numeros de secuencia consecutivos.
 *
 * La resta se hace en 16 bits, asi que la vuelta del contador sale bien
 * sola. Lo que NO sale solo es la secuencia yendo hacia atras, y hay dos
 * formas normales de que eso pase:
 *
 *   1. El emisor se reinicia y su contador vuelve a 0.
 *   2. Un reintento de alimentacion reusa un seq viejo — que es justo lo
 *      que hacemos a proposito para que Piscina reconozca el duplicado.
 *
 * En los dos casos la resta da un numero enorme (65533, 65109...) que
 * envenena el acumulado. Y este acumulado es un entregable de la tesis: es
 * el que sostiene las pruebas de alcance. Asi que los saltos imposibles se
 * descartan devolviendo 0, y el contador sigue siendo utilizable despues de
 * un reinicio o de un reintento.
 */
static inline uint16_t protoSeqGap(uint16_t last, uint16_t current) {
  const uint16_t gap = (uint16_t)(current - last - 1);
  return (gap > PROTO_SEQ_GAP_MAX) ? 0 : gap;
}
