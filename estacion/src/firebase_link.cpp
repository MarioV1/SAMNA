/*
 * firebase_link.cpp — ESTACION
 * ------------------------------------------------------------------
 * Ver firebase_link.h para el contrato y para que se borra y que no.
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
 * Dos objetos FirebaseData a proposito, no uno.
 *
 * Un stream monopoliza su conexion: si se reutilizara el mismo objeto para
 * escribir el borrado de /motores, la escritura cerraria el stream y habria
 * que reabrirlo en cada ciclo de alimentacion. Es el patron que documenta la
 * propia libreria.
 */
static FirebaseData   fbStream;
static FirebaseData   fbWrite;
static FirebaseAuth   auth;
static FirebaseConfig config;

static bool started    = false;   /* Firebase.begin() hecho          */
static bool streamOpen = false;   /* beginStream() hecho y vivo      */

static FbCommands cmds  = { NAV_STOP, 0, 0, false };
static bool       feedPending  = false;   /* flanco sin consumir     */
static bool       clearPending = false;   /* /motores por poner en false */

static FbStats stats;
static char    lastErr[128] = "";

/* Volcado crudo de la primera lectura, para poder ver que hay de verdad en
 * la base en vez de deducirlo. */
static char    initialDump[256] = "";

/* Reintento de apertura del stream, para no machacar la red si falla. */
#define FB_RETRY_MS   5000
static uint32_t lastTryMs = 0;

/* Cada cuanto se sondea el stream. Ver la nota en firebasePoll(). */
#define FB_STREAM_POLL_MS   100
static uint32_t lastStreamMs = 0;

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
 *  Aplicacion de valores
 * ================================================================== */

static void applyKey(const String &key, bool boolVal, int intVal, bool isBool) {
  if (key == "motores") {
    if (isBool && boolVal) {
      /* Flanco de alimentacion. Se marca y se pide el borrado: si la clave
       * se quedara en true, cada evento posterior volveria a dispararla. */
      feedPending  = true;
      clearPending = true;
    }
    return;
  }
  if (key == "pwm") {
    int v = intVal;
    if (v < 0)   { v = 0; }
    if (v > 100) { v = 100; }
    cmds.grams = (uint8_t)v;
    return;
  }
  if (key == "aspersor") {
    int v = intVal;
    if (v < 0)                  { v = 0; }
    if (v > SPRAYER_LEVEL_MAX)  { v = SPRAYER_LEVEL_MAX; }
    cmds.sprayer = (uint8_t)v;
    return;
  }
  if (key == "nav/adelante") { navAdelante = boolVal; cmds.nav = resolveNav(); return; }
  if (key == "nav/atras")    { navAtras    = boolVal; cmds.nav = resolveNav(); return; }
  if (key == "nav/ho")       { navHo       = boolVal; cmds.nav = resolveNav(); return; }
  if (key == "nav/aho")      { navAho      = boolVal; cmds.nav = resolveNav(); return; }
  /* Cualquier otra clave es telemetria nuestra rebotando por el stream de
   * raiz, o algo que no nos incumbe. Se ignora en silencio. */
}

/* Lee el volcado completo que llega al abrir el stream. */
static void applyWholeTree(FirebaseJson *json) {
  if (json == NULL) {
    return;
  }
  FirebaseJsonData r;

  if (json->get(r, "motores") && r.typeNum == FirebaseJson::JSON_BOOL && r.boolValue) {
    feedPending  = true;
    clearPending = true;
  }
  if (json->get(r, "pwm")) {
    applyKey("pwm", false, r.intValue, false);
  }
  if (json->get(r, "aspersor")) {
    applyKey("aspersor", false, r.intValue, false);
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
  lastErr[0] = '\0';

  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);

  /*
   * La reconexion WiFi la lleva wifi_link, no la libreria. Con las dos
   * intentando reconectar se pisan: una llama a WiFi.begin() mientras la
   * otra esta a mitad de un intento, y el resultado es una placa que tarda
   * mucho mas en volver que si mandara una sola.
   */
  Firebase.reconnectWiFi(false);

  /*
   * Timeouts recortados. Los de fabrica son 10 s tanto para abrir el socket
   * como para esperar respuesta, y eso convierte una llamada bloqueante en
   * una parada de diez segundos del bucle entero — cinco veces el deadman.
   *
   * Recortarlos no resuelve el problema de fondo, solo acota lo peor que
   * puede pasar mientras se decide como partir el trabajo.
   */
  config.timeout.socketConnection = 3000;
  config.timeout.serverResponse   = 3000;

  started = true;
  return true;
}

/*
 * OJO con la forma de llamar a la libreria.
 *
 * FirebaseESP32 (la variante especifica de ESP32, que es la que fijamos)
 * deja el miembro .RTDB PRIVADO y expone los metodos directamente sobre el
 * objeto Firebase, tomando FirebaseData por REFERENCIA.
 *
 *   correcto:   Firebase.beginStream(fbStream, "/")
 *   NO compila: Firebase.RTDB.beginStream(&fbStream, "/")
 *
 * La segunda forma es la del cliente unificado FirebaseClient y sale en casi
 * toda la documentacion de la red. Aqui da "RTDB is private within this
 * context".
 */
/*
 * Lectura inicial del arbol completo.
 *
 * Hace falta y no es un lujo: un stream solo entrega CAMBIOS. Si las claves
 * ya estaban escritas antes de que Estacion arrancara — que es el caso
 * normal, porque la app lleva ahi mas tiempo que la placa — no llega ningun
 * evento y Estacion se queda creyendo que todo vale cero hasta que alguien
 * toque el movil.
 *
 * Se hace por la conexion de escritura, no por la del stream, para no
 * interferir con el.
 */
static void readInitial() {
  const uint32_t t0 = millis();
  const bool ok = Firebase.getJSON(fbWrite, "/");
  const uint32_t dt = millis() - t0;
  stats.calls++;
  stats.totalCallMs += dt;
  if (dt > stats.maxCallMs) {
    stats.maxCallMs = dt;
  }

  if (!ok) {
    setError("no se pudo leer el estado inicial", fbWrite.errorReason().c_str());
    return;
  }

  FirebaseJson *json = fbWrite.jsonObjectPtr();
  if (json == NULL) {
    setError("estado inicial vacio o no es un objeto", fbWrite.dataType().c_str());
    return;
  }

  /* Se deja el JSON crudo accesible para el diagnostico: si la base no tiene
   * las claves que esperamos, esto es lo que lo demuestra. */
  String raw;
  json->toString(raw, false);
  strncpy(initialDump, raw.c_str(), sizeof(initialDump) - 1);
  initialDump[sizeof(initialDump) - 1] = '\0';

  applyWholeTree(json);
  stats.events++;
  stats.lastEventMs = millis();
}

static void openStream() {
  if (!Firebase.beginStream(fbStream, "/")) {
    setError("no se pudo abrir el stream", fbStream.errorReason().c_str());
    streamOpen = false;
    return;
  }
  streamOpen = true;
  stats.reconnects++;
  lastErr[0] = '\0';

  /* Con el stream ya abierto, se sincroniza el estado actual. En este orden
   * a proposito: si se leyera antes de abrir, un cambio ocurrido entre la
   * lectura y la apertura se perderia para siempre. */
  readInitial();
}

/* ==================================================================
 *  Bucle
 * ================================================================== */

/* Envuelve una llamada de la libreria midiendo lo que tarda, que es el dato
 * que decide si el bucle unico aguanta o hay que partir en tareas. */
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
     * solo consume tiempo de bucle esperando timeouts. */
    streamOpen = false;
    return;
  }

  if (!Firebase.ready()) {
    return;
  }

  if (!streamOpen) {
    if ((millis() - lastTryMs) < FB_RETRY_MS) {
      return;
    }
    lastTryMs = millis();
    const uint32_t t0 = callStart();
    openStream();
    callEnd(t0);
    return;
  }

  /* Borrado pendiente de /motores. Se hace aqui, fuera del manejo del
   * stream, para no escribir en mitad de la lectura de un evento. */
  if (clearPending) {
    const uint32_t t0 = callStart();
    if (Firebase.setBool(fbWrite, "/motores", false)) {
      clearPending = false;
    } else {
      setError("no se pudo borrar /motores", fbWrite.errorReason().c_str());
    }
    callEnd(t0);
  }

  /*
   * Ritmo de sondeo del stream.
   *
   * Sin esto, el bucle llama a readStream() unas 12 000 veces por segundo.
   * La libreria no esta pensada para eso: cada llamada toca el socket, y a
   * ese ritmo la cuenta de llamadas lentas se dispara.
   *
   * 100 ms es de sobra para la navegacion. El comando tarda ademas 145 ms en
   * el aire, asi que afinar por debajo de eso no se nota en el catamaran.
   */
  if ((millis() - lastStreamMs) < FB_STREAM_POLL_MS) {
    return;
  }
  lastStreamMs = millis();

  const uint32_t t0 = callStart();
  const bool ok = Firebase.readStream(fbStream);
  callEnd(t0);

  if (!ok) {
    setError("fallo leyendo el stream", fbStream.errorReason().c_str());
    streamOpen = false;
    return;
  }

  if (fbStream.streamTimeout()) {
    /* La libreria lo reabre sola, pero se anota: si esto crece mucho, la
     * cobertura WiFi o el enlace a internet no dan. */
    stats.reconnects++;
    return;
  }

  if (!fbStream.streamAvailable()) {
    return;
  }

  stats.events++;
  stats.lastEventMs = millis();

  String path = fbStream.dataPath();
  if (path.startsWith("/")) {
    path.remove(0, 1);
  }

  if (path.length() == 0) {
    /* Volcado completo: pasa al abrir el stream y cuando se reescribe la
     * raiz entera. */
    applyWholeTree(fbStream.jsonObjectPtr());
    return;
  }

  const String type = fbStream.dataType();
  if (type == "json") {
    /* Un subarbol, tipicamente /nav completo cuando la app escribe las
     * cuatro banderas juntas con updateChildren. */
    FirebaseJson *json = fbStream.jsonObjectPtr();
    FirebaseJsonData r;
    if (json != NULL) {
      if (json->get(r, "adelante")) { navAdelante = r.boolValue; }
      if (json->get(r, "atras"))    { navAtras    = r.boolValue; }
      if (json->get(r, "ho"))       { navHo       = r.boolValue; }
      if (json->get(r, "aho"))      { navAho      = r.boolValue; }
      cmds.nav = resolveNav();
    }
    return;
  }

  applyKey(path,
           (type == "boolean") ? fbStream.boolData() : false,
           (type == "int" || type == "float" || type == "double")
             ? (int)fbStream.intData() : 0,
           type == "boolean");
}

/* ==================================================================
 *  Consulta
 * ================================================================== */

bool firebaseReady() {
  return started && streamOpen && wifiConnected();
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

const FbStats *firebaseStats() {
  return &stats;
}

const char *firebaseLastError() {
  return lastErr;
}

const char *firebaseInitialDump() {
  return initialDump;
}
