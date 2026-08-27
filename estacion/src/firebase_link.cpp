/*
 * firebase_link.cpp — ESTACION
 * ------------------------------------------------------------------
 * Ver firebase_link.h para el contrato y para que se borra y que no.
 *
 * POR QUE SONDEO Y NO STREAM
 *
 * El diseño original escuchaba un stream de Realtime Database. No funciona
 * con esta version de la libreria en esta placa: el stream se abre, dice
 * httpConnected=SI, no da un solo error... y no entrega NI UN evento. Se
 * comprobo escribiendo 151 veces en la misma ruta que estaba escuchando.
 *
 * Descartado por experimento, no por sospecha:
 *   memoria            234 kB libres, minimo historico 228 kB
 *   timeouts           probados el de fabrica y recortados
 *   escrituras         con la telemetria apagada seguia sin entregar nada
 *   ritmo de sondeo    de 10 Hz a llamada continua, 2184 llamadas, 0 eventos
 *   API de callback    setStreamCallback tampoco dispara
 *   ruta               ni en la raiz ni en una clave suelta
 *
 * Lo que SI funciona sin un solo fallo es getJSON, updateNode y setBool. Asi
 * que se lee el arbol entero cada FB_POLL_MS y se aplica. Menos elegante que
 * un stream, pero probado en placa, que vale mas.
 * ------------------------------------------------------------------
 */

#include "firebase_link.h"

#include <Arduino.h>
#include <FirebaseESP32.h>

#include "secrets.h"
#include "wifi_link.h"

/* ==================================================================
 *  Cliente
 * ================================================================== */

/*
 * Dos objetos FirebaseData: uno para leer y otro para escribir.
 *
 * Podrian ser uno solo, pero alternar GET y PATCH sobre el mismo objeto le
 * hace renegociar la conexion a cada cambio de verbo. Con dos, cada uno
 * mantiene la suya. RAM sobra: quedan mas de 230 kB libres.
 */
static FirebaseData   fbRead;
static FirebaseData   fbWrite;
static FirebaseAuth   auth;
static FirebaseConfig config;

/*
 * Cada cuanto se relee el arbol de comandos.
 *
 * 500 ms es el equilibrio entre que la navegacion responda y no regalar
 * cuota. Cada lectura son unos 800 B con cabeceras y TLS, asi que a este
 * ritmo salen ~4 GB al mes si la estacion corriera sin parar los 30 dias;
 * el plan gratuito da 10 GB. Para una unidad que se enciende para ensayos
 * va sobrado, pero si algun dia queda permanentemente encendida conviene
 * subirlo a 1000 ms y aceptar medio segundo mas de latencia.
 *
 * A esto hay que sumarle el refresco LoRa de NAV_REFRESH_MS, asi que del
 * dedo en el movil al propulsor pasan como mucho FB_POLL_MS + 500 ms.
 */
#define FB_POLL_MS   500

static bool started = false;
static bool online  = false;   /* ultima lectura buena */

static FbCommands cmds  = { NAV_STOP, 0, 0, false };
static bool       feedPending  = false;   /* flanco sin consumir     */
static bool       clearPending = false;   /* /motores por poner en false */
static bool       stopPending      = false;   /* flanco de /paro sin consumir */
static bool       clearStopPending = false;   /* /paro por poner en false     */

static FbStats stats;
static char    lastErr[128] = "";

/* Volcado crudo de la ultima lectura, para poder ver que hay de verdad en
 * la base en vez de deducirlo. */
static char    lastDump[256] = "";

static uint32_t lastPollMs = 0;

static void setError(const char *what, const char *detail) {
  snprintf(lastErr, sizeof(lastErr), "%s: %s", what, detail ? detail : "?");
  stats.errors++;
}

/* ==================================================================
 *  Navegacion
 * ================================================================== */

static bool navAdelante = false;
static bool navAtras    = false;
static bool navHo       = false;
static bool navAho      = false;

/*
 * Resuelve las cuatro banderas en una sola intencion.
 *
 * La app las escribe juntas con updateChildren, asi que por construccion no
 * deberia haber dos en true nunca. Pero "no deberia" no basta cuando al otro
 * lado hay dos propulsores: si llegan dos, la orden es contradictoria y la
 * respuesta segura es PARAR, no elegir una por orden de aparicion.
 */
static NavCmd resolveNav() {
  const uint8_t n = (navAdelante ? 1 : 0) + (navAtras ? 1 : 0) +
                    (navHo ? 1 : 0) + (navAho ? 1 : 0);
  if (n != 1) {
    return NAV_STOP;   /* ninguna, o varias a la vez */
  }
  if (navAdelante) { return NAV_FORWARD; }
  if (navAtras)    { return NAV_REVERSE; }
  if (navHo)       { return NAV_CW; }
  return NAV_CCW;
}

/* ==================================================================
 *  Aplicacion del arbol leido
 * ================================================================== */

static void applyTree(FirebaseJson *json) {
  if (json == NULL) {
    return;
  }
  FirebaseJsonData r;

  /*
   * Alimentacion.
   *
   * El `!clearPending` NO sobra. Esta funcion corre en cada sondeo, y si el
   * borrado de /motores todavia no ha prosperado la clave sigue en true:
   * sin la guarda, cada vuelta dispararia OTRO ciclo. Es el espejo del fallo
   * del seq — en vez de no comer, el camaron comeria sin parar.
   */
  if (!clearPending &&
      json->get(r, "motores") && r.typeNum == FirebaseJson::JSON_BOOL && r.boolValue) {
    feedPending  = true;
    clearPending = true;
  }

  /* PARO. Misma guarda que /motores y por la misma razon: mientras su
   * borrado no haya prosperado, la clave sigue en true y cada vuelta
   * volveria a dispararlo. */
  if (!clearStopPending &&
      json->get(r, "paro") && r.typeNum == FirebaseJson::JSON_BOOL && r.boolValue) {
    stopPending      = true;
    clearStopPending = true;
  }

  if (json->get(r, "pwm")) {
    /*
     * Gramos, hasta GRAMS_MAX. El recorte estaba en 100 de cuando la app
     * ofrecia raciones en gramos: al pasar a kilos, un 2500 se convertia en
     * 100 y el sistema entregaba 25 veces menos informando de exito.
     */
    int v = r.intValue;
    if (v < 0)          { v = 0; }
    if (v > GRAMS_MAX)  { v = GRAMS_MAX; }
    cmds.grams = (uint16_t)v;
  }

  if (json->get(r, "aspersor")) {
    int v = r.intValue;
    if (v < 0)                 { v = 0; }
    if (v > SPRAYER_LEVEL_MAX) { v = SPRAYER_LEVEL_MAX; }
    cmds.sprayer = (uint8_t)v;
  }

  if (json->get(r, "nav/adelante")) { navAdelante = r.boolValue; }
  if (json->get(r, "nav/atras"))    { navAtras    = r.boolValue; }
  if (json->get(r, "nav/ho"))       { navHo       = r.boolValue; }
  if (json->get(r, "nav/aho"))      { navAho      = r.boolValue; }
  cmds.nav = resolveNav();
}

/* ==================================================================
 *  Arranque
 * ================================================================== */

bool firebaseBegin() {
  memset(&stats, 0, sizeof(stats));
  lastErr[0]  = '\0';
  lastDump[0] = '\0';

  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);

  /*
   * La reconexion WiFi la lleva wifi_link, no la libreria. Con las dos
   * intentando reconectar se pisan: una llama a WiFi.begin() mientras la
   * otra esta a mitad de un intento, y la placa tarda mucho mas en volver.
   */
  Firebase.reconnectWiFi(false);

  config.timeout.socketConnection = 5000;
  config.timeout.serverResponse   = 10000;

  /* Afecta a los flotantes que la libreria serializa por su cuenta, pero NO
   * a los que van dentro de un FirebaseJson — comprobado en placa. De esos
   * se encarga round2() al escribir. */
  Firebase.setDoubleDigits(2);

  started = true;
  return true;
}

/* ==================================================================
 *  Bucle
 * ================================================================== */

/* Envuelve una llamada midiendo lo que tarda. Ya no decide la arquitectura
 * — eso lo zanjo la particion en tareas — pero sigue siendo el termometro
 * de como va la red. */
static uint32_t callStart() {
  return millis();
}

static void callEnd(uint32_t t0) {
  const uint32_t dt = millis() - t0;
  stats.calls++;
  stats.totalCallMs += dt;
  if (dt >= FB_SLOW_CALL_MS) {
    stats.slowCalls++;
  }
  if (dt > stats.maxCallMs) {
    stats.maxCallMs = dt;
  }
}

void firebasePoll() {
  if (!started || !wifiConnected()) {
    /* Sin WiFi no hay nada que hacer, y llamar a la libreria en ese estado
     * solo consume tiempo esperando timeouts. */
    online = false;
    return;
  }
  if (!Firebase.ready()) {
    return;
  }

  /*
   * Borrado pendiente de /motores, antes de la lectura.
   *
   * En este orden a proposito: si se leyera primero, la lectura devolveria
   * el true todavia sin borrar y habria que confiar solo en la guarda de
   * applyTree. Borrando antes, la siguiente lectura ya ve el false.
   */
  if (clearPending) {
    const uint32_t t0 = callStart();
    if (Firebase.setBool(fbWrite, "/motores", false)) {
      clearPending = false;
    } else {
      setError("no se pudo borrar /motores", fbWrite.errorReason().c_str());
    }
    callEnd(t0);
  }

  if (clearStopPending) {
    const uint32_t t0 = callStart();
    if (Firebase.setBool(fbWrite, "/paro", false)) {
      clearStopPending = false;
    } else {
      setError("no se pudo borrar /paro", fbWrite.errorReason().c_str());
    }
    callEnd(t0);
  }

  if ((millis() - lastPollMs) < FB_POLL_MS) {
    return;
  }
  lastPollMs = millis();

  const uint32_t t0 = callStart();
  const bool ok = Firebase.getJSON(fbRead, "/");
  callEnd(t0);

  if (!ok) {
    setError("no se pudo leer el arbol", fbRead.errorReason().c_str());
    online = false;
    return;
  }

  FirebaseJson *json = fbRead.jsonObjectPtr();
  if (json == NULL) {
    setError("la raiz no es un objeto", fbRead.dataType().c_str());
    online = false;
    return;
  }

  online = true;
  stats.events++;
  stats.lastEventMs = millis();

  String raw;
  json->toString(raw, false);
  strncpy(lastDump, raw.c_str(), sizeof(lastDump) - 1);
  lastDump[sizeof(lastDump) - 1] = '\0';

  applyTree(json);
}

/* ==================================================================
 *  Telemetria hacia la base
 * ================================================================== */

/*
 * Ritmo minimo entre escrituras. Piscina manda telemetria cada
 * TLM_PERIOD_MS, asi que en marcha normal esto no recorta nada; esta como
 * tope por si la cadencia sube o llegan paquetes repetidos.
 */
#define FB_WRITE_MIN_MS   1500
static uint32_t lastWriteMs = 0;

/*
 * Redondeo a dos decimales antes de escribir.
 *
 * Firebase.setDoubleDigits() no alcanza a los flotantes que van dentro de un
 * FirebaseJson: sin esto, un 26.98 acaba en la base como 26.97989.
 *
 * Y no es solo estetica. El DS18B20 tiene una exactitud de +-0.5 grados;
 * publicar cinco decimales anuncia una resolucion que el sensor no tiene y
 * que en una memoria de titulacion es una afirmacion falsa.
 */
static float round2(float v) {
  return roundf(v * 100.0f) / 100.0f;
}

static bool tlmEnabled = true;

void firebaseSetTlmEnabled(bool on) {
  tlmEnabled = on;
}

bool firebaseTlmEnabled() {
  return tlmEnabled;
}

bool firebaseWriteTlm(const TlmPacket *tlm) {
  if (!tlmEnabled) {
    return false;
  }
  if (tlm == NULL || !started || !wifiConnected() || !Firebase.ready()) {
    return false;
  }
  if (lastWriteMs != 0 && (millis() - lastWriteMs) < FB_WRITE_MIN_MS) {
    return false;
  }

  FirebaseJson json;
  uint8_t fields = 0;

  if (!isnan(tlm->temperature)) {
    json.set("temperatura", round2(tlm->temperature));
    fields++;
  } else {
    stats.skippedNan++;
  }

  if (!isnan(tlm->ph)) {
    json.set("ph", round2(tlm->ph));
    fields++;
  } else {
    stats.skippedNan++;
  }

  /*
   * El nivel de sonido es un entero 0-100 y no puede llevar NAN, asi que su
   * centinela es el bit ST_SOUND_FAULT. Se omite igual que las otras dos
   * claves: publicar el 0 que manda Piscina con el modulo caido lo leeria
   * la app como una piscina en silencio, que es justo la conclusion
   * contraria a la verdadera.
   */
  if ((tlm->status & ST_SOUND_FAULT) == 0) {
    json.set("nivelSonido", (int)tlm->soundLevel);
    fields++;
  } else {
    stats.skippedNan++;
  }

  if (fields == 0) {
    return false;
  }

  lastWriteMs = millis();

  const uint32_t t0 = callStart();
  const bool ok = Firebase.updateNode(fbWrite, "/", json);
  callEnd(t0);

  if (!ok) {
    setError("no se pudo escribir la telemetria", fbWrite.errorReason().c_str());
    stats.writeFails++;
    return false;
  }
  stats.writes++;
  return true;
}

/* ==================================================================
 *  Consulta
 * ================================================================== */

bool firebaseReady() {
  return started && online && wifiConnected();
}

const FbCommands *firebaseCommands() {
  return &cmds;
}

bool firebaseTakeFeed() {
  if (!feedPending) {
    return false;
  }
  feedPending = false;
  return true;
}

bool firebaseTakeStop() {
  if (!stopPending) {
    return false;
  }
  stopPending = false;
  return true;
}

const FbStats *firebaseStats() {
  return &stats;
}

const char *firebaseLastError() {
  return lastErr;
}

const char *firebaseInitialDump() {
  return lastDump;
}

void firebaseStreamDebug(char *out, size_t n) {
  snprintf(out, n, "sondeo cada %d ms  |  ultima lectura %s",
           FB_POLL_MS, online ? "OK" : "FALLIDA");
}
