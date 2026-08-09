/*
 * wifi_link.cpp — ESTACION
 * ------------------------------------------------------------------
 * Ver wifi_link.h para el contrato y para por que nada aqui bloquea.
 * ------------------------------------------------------------------
 */

#include "wifi_link.h"

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>

#include "secrets.h"
#include "wifi_portal.h"

/*
 * WPA2 exige 8 caracteres como minimo. Con menos, softAP() no falla de forma
 * evidente: levanta el AP ABIERTO. El sintoma seria una red de configuracion
 * sin contrasena y nadie se enteraria hasta que alguien la usara.
 *
 * sizeof de un literal cuenta el terminador, asi que 9 significa 8 caracteres.
 */
static_assert(sizeof(AP_PASSWORD) >= 9,
              "AP_PASSWORD necesita 8 caracteres o mas, o el AP queda abierto");

/* ==================================================================
 *  Tiempos
 * ================================================================== */

/*
 * Cuanto se le da a una red antes de pasar a la siguiente.
 *
 * 8 s es holgado para una asociacion WPA2 con DHCP. Mas corto descarta redes
 * buenas que solo iban lentas; mas largo alarga la ronda completa sin ganar
 * nada, porque una red que no asocia en 8 s casi nunca asocia en 20.
 */
#define WIFI_ATTEMPT_MS   8000

/*
 * Espera tras agotar los candidatos, antes de volver a empezar.
 *
 * Sin esta pausa, Estacion se pasaria la vida reasociando contra un router
 * apagado, y cada WiFi.begin() mueve la radio y come CPU justo cuando el
 * enlace LoRa la necesita.
 */
#define WIFI_BACKOFF_MS   15000

/* Indice reservado para las credenciales que llegan del portal. */
#define CAND_PENDING      0xFF

/* ==================================================================
 *  Estado
 * ================================================================== */

static Preferences prefs;

static WifiState state = WIFI_ST_IDLE;

static uint8_t   candidate    = 0;
static uint32_t  attemptMs    = 0;
static uint32_t  backoffMs    = 0;
static uint32_t  failedRounds = 0;

static char slotSsid[WIFI_SLOTS][WIFI_SSID_MAX];
static char slotPass[WIFI_SLOTS][WIFI_PASS_MAX];

static char pendSsid[WIFI_SSID_MAX] = "";
static char pendPass[WIFI_PASS_MAX] = "";

static char curSsid[WIFI_SSID_MAX] = "";
static char curIp[16]   = "";
static char apIp[16]     = "";
/* 160 y no 96: el mensaje mas largo ronda los 66 caracteres y un SSID puede
 * traer 32 mas. Con 96 el aviso se cortaba justo por donde explica el fallo. */
static char lastErr[160] = "";

static bool apUp = false;

/* ==================================================================
 *  NVS
 * ================================================================== */

static void slotKey(char *out, size_t n, const char *base, uint8_t slot) {
  snprintf(out, n, "%s%u", base, (unsigned)slot);
}

/*
 * Se comprueba isKey() antes de leer. getString() sobre una clave que no
 * existe funciona igual, pero el nucleo escupe un [E] ...NOT_FOUND por
 * serial, y en el primer arranque salen cuatro seguidos que parecen un fallo
 * grave cuando en realidad es NVS vacio, que es lo normal de fabrica.
 */
static void loadSlots() {
  char key[12];
  for (uint8_t i = 0; i < WIFI_SLOTS; i++) {
    slotSsid[i][0] = '\0';
    slotPass[i][0] = '\0';

    slotKey(key, sizeof(key), "ssid", i);
    if (!prefs.isKey(key)) {
      continue;
    }
    prefs.getString(key, slotSsid[i], sizeof(slotSsid[i]));

    slotKey(key, sizeof(key), "pass", i);
    if (prefs.isKey(key)) {
      prefs.getString(key, slotPass[i], sizeof(slotPass[i]));
    }
  }
}

static void storeSlots() {
  char key[12];
  for (uint8_t i = 0; i < WIFI_SLOTS; i++) {
    slotKey(key, sizeof(key), "ssid", i);
    prefs.putString(key, slotSsid[i]);
    slotKey(key, sizeof(key), "pass", i);
    prefs.putString(key, slotPass[i]);
  }
}

bool wifiSaveNetwork(const char *ssid, const char *pass) {
  if (ssid == NULL || ssid[0] == '\0') {
    return false;
  }
  /* Ya es la mas reciente: no se reescribe NVS por gusto, que tiene ciclos
   * de escritura finitos y esto se llama en cada reconexion. */
  if (strncmp(slotSsid[0], ssid, WIFI_SSID_MAX) == 0) {
    return true;
  }

  for (uint8_t i = WIFI_SLOTS - 1; i > 0; i--) {
    strncpy(slotSsid[i], slotSsid[i - 1], WIFI_SSID_MAX);
    strncpy(slotPass[i], slotPass[i - 1], WIFI_PASS_MAX);
  }
  strncpy(slotSsid[0], ssid, WIFI_SSID_MAX - 1);
  slotSsid[0][WIFI_SSID_MAX - 1] = '\0';
  strncpy(slotPass[0], pass ? pass : "", WIFI_PASS_MAX - 1);
  slotPass[0][WIFI_PASS_MAX - 1] = '\0';

  storeSlots();
  return true;
}

void wifiForgetAll() {
  for (uint8_t i = 0; i < WIFI_SLOTS; i++) {
    slotSsid[i][0] = '\0';
    slotPass[i][0] = '\0';
  }
  storeSlots();
}

const char *wifiStoredSsid(uint8_t slot) {
  return (slot < WIFI_SLOTS) ? slotSsid[slot] : "";
}

/* ==================================================================
 *  Punto de acceso de configuracion
 * ================================================================== */

static void apStart() {
  if (apUp) {
    return;
  }
  /* AP+STA a proposito: el portal queda disponible SIN dejar de reintentar
   * las redes guardadas. Si el router vuelve solo tras un corte de luz,
   * Estacion se reengancha sin que nadie saque el movil. */
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  strncpy(apIp, WiFi.softAPIP().toString().c_str(), sizeof(apIp) - 1);
  apIp[sizeof(apIp) - 1] = '\0';
  portalStart();
  apUp = true;
}

static void apStop() {
  if (!apUp) {
    return;
  }
  portalStop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apIp[0] = '\0';
  apUp = false;
}

bool wifiPortalUp() {
  return apUp;
}

/*
 * Portal abierto a mano.
 *
 * Se distingue del que se abre solo por falta de red, porque ese otro tiene
 * un final natural: se cierra en cuanto la STA conecta. El forzado no, y si
 * la STA YA estaba conectada cuando se pulso la tecla, se quedaria emitiendo
 * para siempre. En una unidad de campo eso es un punto de acceso abierto en
 * medio de la camaronera que nadie recuerda haber dejado ahi.
 */
#define PORTAL_FORCED_MS   (5 * 60 * 1000)

static uint32_t portalForcedMs = 0;
static bool     portalForced   = false;

void wifiForcePortal() {
  portalForced   = true;
  portalForcedMs = millis();
  apStart();
}

void wifiClosePortal() {
  portalForced = false;
  apStop();
}

/* Cierra el portal forzado cuando vence su plazo. Solo si hay red: sin ella
 * el portal es la unica via de entrada y cerrarlo dejaria la placa
 * incomunicada. */
static void portalTick() {
  if (!apUp || !portalForced) {
    return;
  }
  if (!wifiConnected()) {
    portalForcedMs = millis();   /* sin red, el plazo no corre */
    return;
  }
  if ((millis() - portalForcedMs) >= PORTAL_FORCED_MS) {
    portalForced = false;
    apStop();
  }
}

const char *wifiApIp() {
  return apIp;
}

/* ==================================================================
 *  Candidatos
 * ================================================================== */

static bool candidateAt(uint8_t idx, const char **ssid, const char **pass) {
  if (idx >= WIFI_SLOTS || slotSsid[idx][0] == '\0') {
    return false;
  }
  *ssid = slotSsid[idx];
  *pass = slotPass[idx];
  return true;
}

static void beginAttempt(uint8_t idx, const char *ssid, const char *pass) {
  candidate = idx;
  attemptMs = millis();
  state     = WIFI_ST_TRYING;

  strncpy(curSsid, ssid, sizeof(curSsid) - 1);
  curSsid[sizeof(curSsid) - 1] = '\0';
  curIp[0] = '\0';

  WiFi.disconnect(false, false);
  WiFi.begin(ssid, pass);   /* asincrono: vuelve enseguida */
}

/* Lanza el intento contra el primer candidato utilizable desde `from`.
 * Si no queda ninguno, pasa a backoff. */
static void tryFrom(uint8_t from) {
  for (uint8_t i = from; i < WIFI_SLOTS; i++) {
    const char *ssid;
    const char *pass;
    if (candidateAt(i, &ssid, &pass)) {
      beginAttempt(i, ssid, pass);
      return;
    }
  }

  /* Solo cuenta como ronda fallida si habia algo que probar. Una placa
   * virgen no "falla": es que todavia no la han configurado. */
  bool hadAny = false;
  for (uint8_t i = 0; i < WIFI_SLOTS; i++) {
    if (slotSsid[i][0] != '\0') {
      hadAny = true;
      break;
    }
  }
  if (hadAny) {
    failedRounds++;
  }

  backoffMs  = millis();
  state      = WIFI_ST_BACKOFF;
  curSsid[0] = '\0';
  curIp[0]   = '\0';

  /* Sin red utilizable, el portal es la unica salida. */
  apStart();
}

bool wifiTryCredentials(const char *ssid, const char *pass) {
  if (ssid == NULL || ssid[0] == '\0') {
    return false;
  }
  strncpy(pendSsid, ssid, sizeof(pendSsid) - 1);
  pendSsid[sizeof(pendSsid) - 1] = '\0';
  strncpy(pendPass, pass ? pass : "", sizeof(pendPass) - 1);
  pendPass[sizeof(pendPass) - 1] = '\0';

  lastErr[0] = '\0';
  beginAttempt(CAND_PENDING, pendSsid, pendPass);
  return true;
}

const char *wifiLastAttemptError() {
  return lastErr;
}

/* Traduce el motivo del fallo a algo que se pueda leer en el portal. */
static void recordFailure(const char *ssid) {
  switch (WiFi.status()) {
    case WL_NO_SSID_AVAIL:
      snprintf(lastErr, sizeof(lastErr),
               "No se encontro la red \"%s\". Recuerda que solo funciona 2,4 GHz.", ssid);
      break;
    case WL_CONNECT_FAILED:
      snprintf(lastErr, sizeof(lastErr),
               "\"%s\" rechazo la conexion. Revisa la contrasena.", ssid);
      break;
    default:
      snprintf(lastErr, sizeof(lastErr),
               "\"%s\" no respondio a tiempo. Puede ser la contrasena o la cobertura.", ssid);
      break;
  }
}

/* ==================================================================
 *  API
 * ================================================================== */

bool wifiBegin() {
  prefs.begin("wifi", false);
  loadSlots();

  WiFi.persistent(false);   /* la memoria de redes la llevamos nosotros */
  WiFi.mode(WIFI_STA);

  /*
   * Sin ahorro de energia. Estacion va conectada a la red electrica, y el
   * modem sleep introduce latencias de cientos de ms en la respuesta de
   * Firebase — justo lo que no podemos permitirnos con el deadman de por
   * medio.
   */
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);   /* la reconexion la gobierna esta maquina */

  tryFrom(0);
  return true;
}

void wifiPoll() {
  portalPoll();
  portalTick();

  switch (state) {
    case WIFI_ST_IDLE:
      break;

    case WIFI_ST_TRYING:
      if (WiFi.status() == WL_CONNECTED) {
        state = WIFI_ST_ONLINE;
        lastErr[0] = '\0';
        strncpy(curIp, WiFi.localIP().toString().c_str(), sizeof(curIp) - 1);
        curIp[sizeof(curIp) - 1] = '\0';

        if (candidate == CAND_PENDING) {
          /* Credenciales del portal que han demostrado funcionar: ahora si
           * se guardan. */
          wifiSaveNetwork(pendSsid, pendPass);
          pendSsid[0] = '\0';
          pendPass[0] = '\0';
        } else {
          wifiSaveNetwork(curSsid, slotPass[candidate]);
        }
        /* Con red, el portal sobra. */
        apStop();

      } else if ((millis() - attemptMs) >= WIFI_ATTEMPT_MS) {
        recordFailure(curSsid);
        if (candidate == CAND_PENDING) {
          /* Lo que vino del portal no sirve. No se guarda, y se vuelve a la
           * ronda normal dejando el portal arriba para reintentarlo. */
          pendSsid[0] = '\0';
          pendPass[0] = '\0';
          apStart();
          tryFrom(0);
        } else {
          tryFrom(candidate + 1);
        }
      }
      break;

    case WIFI_ST_ONLINE:
      if (WiFi.status() != WL_CONNECTED) {
        /* Se cayo. Se reempieza por la ranura 0, que es la que acaba de
         * funcionar: lo normal es que el router solo se haya reiniciado. */
        curIp[0] = '\0';
        tryFrom(0);
      }
      break;

    case WIFI_ST_BACKOFF:
      if ((millis() - backoffMs) >= WIFI_BACKOFF_MS) {
        tryFrom(0);
      }
      break;
  }
}

WifiState wifiState() {
  return state;
}

const char *wifiStateName() {
  switch (state) {
    case WIFI_ST_IDLE:    return "parado";
    case WIFI_ST_TRYING:  return "conectando";
    case WIFI_ST_ONLINE:  return "conectado";
    case WIFI_ST_BACKOFF: return apUp ? "sin red - portal abierto"
                                      : "sin red, esperando";
  }
  return "?";
}

bool wifiConnected() {
  return state == WIFI_ST_ONLINE;
}

const char *wifiSsid() {
  return curSsid;
}

const char *wifiIp() {
  return curIp;
}

int8_t wifiRssi() {
  return wifiConnected() ? (int8_t)WiFi.RSSI() : 0;
}

uint32_t wifiFailedRounds() {
  return failedRounds;
}
