/*
 * wifi_portal.cpp — ESTACION
 * ------------------------------------------------------------------
 * Ver wifi_portal.h para el contrato.
 *
 * El HTML va embebido y sin una sola referencia externa: ni fuentes, ni CSS
 * de CDN, ni iconos. El telefono conectado a este AP NO tiene internet, asi
 * que cualquier recurso remoto se quedaria colgando hasta agotar el tiempo
 * de espera y la pagina se veria rota justo cuando mas falta hace que
 * funcione.
 * ------------------------------------------------------------------
 */

#include "wifi_portal.h"

#include <Arduino.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>

#include "wifi_link.h"

static DNSServer dns;
static WebServer server(80);
static bool active = false;

/* Escaneo asincrono: -1 sin resultados, -2 en curso (constantes de la
 * libreria WiFi del nucleo). */
static bool scanRequested = false;

/* ==================================================================
 *  HTML
 * ================================================================== */

static const char PAGE_HEAD[] PROGMEM =
  "<!DOCTYPE html><html lang=es><head><meta charset=utf-8>"
  "<meta name=viewport content='width=device-width,initial-scale=1'>"
  "<title>Alimentadora</title><style>"
  "body{font-family:system-ui,sans-serif;margin:0;padding:20px;"
  "background:#101418;color:#e6edf3}"
  "h1{font-size:20px;margin:0 0 4px}"
  "p.sub{margin:0 0 20px;color:#8b949e;font-size:13px}"
  "label{display:block;margin:14px 0 6px;font-size:14px}"
  "input,select{width:100%;box-sizing:border-box;padding:12px;font-size:16px;"
  "border:1px solid #30363d;border-radius:8px;background:#0d1117;color:#e6edf3}"
  "button{width:100%;padding:14px;margin-top:20px;font-size:16px;font-weight:600;"
  "border:0;border-radius:8px;background:#238636;color:#fff}"
  "a.sec{display:block;text-align:center;margin-top:14px;color:#58a6ff;font-size:14px}"
  ".box{max-width:420px;margin:0 auto}"
  ".st{padding:12px;border-radius:8px;margin-bottom:16px;font-size:14px}"
  ".ok{background:#0f2f1a;border:1px solid #238636}"
  ".err{background:#3d1519;border:1px solid #a5303a}"
  ".inf{background:#161b22;border:1px solid #30363d}"
  "</style></head><body><div class=box>";

static const char PAGE_TAIL[] PROGMEM = "</div></body></html>";

/* Escapa lo justo para no romper el HTML con un SSID raro. */
static String esc(const String &in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    const char c = in[i];
    if      (c == '&')  { out += "&amp;"; }
    else if (c == '<')  { out += "&lt;"; }
    else if (c == '>')  { out += "&gt;"; }
    else if (c == '"')  { out += "&quot;"; }
    else if (c == '\'') { out += "&#39;"; }
    else                { out += c; }
  }
  return out;
}

static String statusBox() {
  String s;
  if (wifiConnected()) {
    s += "<div class='st ok'><b>Conectada</b><br>Red: ";
    s += esc(String(wifiSsid()));
    s += "<br>IP: ";
    s += esc(String(wifiIp()));
    s += "</div>";
    return s;
  }

  const char *err = wifiLastAttemptError();
  if (err[0] != '\0') {
    s += "<div class='st err'><b>No se pudo conectar</b><br>";
    s += esc(String(err));
    s += "</div>";
  } else {
    s += "<div class='st inf'>Estado: ";
    s += esc(String(wifiStateName()));
    s += "</div>";
  }
  return s;
}

static String networkPicker() {
  const int n = WiFi.scanComplete();

  if (n == WIFI_SCAN_RUNNING) {
    return "<p class=sub>Buscando redes...</p>"
           "<meta http-equiv=refresh content=2>";
  }
  if (n <= 0) {
    return "<a class=sec href='/scan'>Buscar redes cercanas</a>";
  }

  String s = "<label for=lista>Redes encontradas</label>"
             "<select id=lista onchange=\"document.getElementById('ssid').value=this.value\">"
             "<option value=''>-- elegir --</option>";
  for (int i = 0; i < n && i < 20; i++) {
    const String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) {
      continue;
    }
    s += "<option value='" + esc(ssid) + "'>" + esc(ssid) +
         "  (" + String(WiFi.RSSI(i)) + " dBm)</option>";
  }
  s += "</select><a class=sec href='/scan'>Volver a buscar</a>";
  return s;
}

static void handleRoot() {
  String p = FPSTR(PAGE_HEAD);
  p += "<h1>Alimentadora acuicola</h1>"
       "<p class=sub>Estacion base &middot; configuracion de red</p>";
  p += statusBox();
  p += networkPicker();
  p += "<form method=POST action='/save'>"
       "<label for=ssid>Nombre de la red (2,4 GHz)</label>"
       "<input id=ssid name=ssid maxlength=32 autocapitalize=off autocorrect=off required>"
       "<label for=pass>Contrasena</label>"
       "<input id=pass name=pass type=password maxlength=63>"
       "<button type=submit>Conectar</button></form>";

  /* Las guardadas se muestran para que se vea que la placa recuerda dos, y
   * cual va a probar primero. */
  const char *s0 = wifiStoredSsid(0);
  const char *s1 = wifiStoredSsid(1);
  if (s0[0] != '\0' || s1[0] != '\0') {
    p += "<p class=sub style='margin-top:24px'>Recordadas: ";
    if (s0[0] != '\0') { p += esc(String(s0)); }
    if (s1[0] != '\0') { p += ", " + esc(String(s1)); }
    p += "</p>";
  }

  p += FPSTR(PAGE_TAIL);
  server.send(200, "text/html", p);
}

static void handleSave() {
  const String ssid = server.arg("ssid");
  const String pass = server.arg("pass");

  if (ssid.length() == 0) {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  /*
   * Se prueba ANTES de guardar. Guardar primero y probar despues llenaria
   * NVS de redes con la contrasena mal escrita, y como la ranura 0 se prueba
   * primero, cada arranque perderia 8 s con una red que nunca va a entrar.
   * El AP sigue arriba durante el intento, asi que el movil no se cae y
   * puede ver el resultado.
   */
  wifiTryCredentials(ssid.c_str(), pass.c_str());

  String p = FPSTR(PAGE_HEAD);
  p += "<h1>Conectando...</h1>"
       "<p class=sub>Probando <b>" + esc(ssid) + "</b>. Puede tardar unos segundos.</p>"
       "<div class='st inf'>Si conecta, esta red de configuracion se apagara "
       "sola y tu telefono volvera a su red normal. Eso significa que salio "
       "bien.</div>"
       "<meta http-equiv=refresh content='3;url=/'>"
       "<a class=sec href='/'>Ver estado</a>";
  p += FPSTR(PAGE_TAIL);
  server.send(200, "text/html", p);
}

static void handleScan() {
  portalStartScan();
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

/*
 * Cualquier otra URL va a la raiz. Es lo que hace que Android, iOS y Windows
 * detecten el portal: sus comprobaciones de conectividad piden una URL
 * concreta y, al no recibir lo que esperan, muestran el aviso de "iniciar
 * sesion en la red".
 */
static void handleNotFound() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString());
  server.send(302, "text/plain", "");
}

/* ==================================================================
 *  API
 * ================================================================== */

bool portalStart() {
  if (active) {
    return true;
  }

  dns.setErrorReplyCode(DNSReplyCode::NoError);
  /* El comodin manda todas las consultas a la IP del AP. */
  dns.start(53, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/scan", HTTP_GET, handleScan);
  server.onNotFound(handleNotFound);
  server.begin();

  active = true;
  scanRequested = false;
  return true;
}

void portalStop() {
  if (!active) {
    return;
  }
  server.stop();
  dns.stop();
  WiFi.scanDelete();
  active = false;
  scanRequested = false;
}

void portalPoll() {
  if (!active) {
    return;
  }
  dns.processNextRequest();
  server.handleClient();

  /* El escaneo se lanza aqui y no dentro del manejador HTTP: arrancarlo
   * mientras se esta respondiendo una peticion deja la respuesta a medias. */
  if (scanRequested) {
    scanRequested = false;
    WiFi.scanDelete();
    WiFi.scanNetworks(true, false);   /* asincrono, sin redes ocultas */
  }
}

bool portalActive() {
  return active;
}

void portalStartScan() {
  scanRequested = true;
}
