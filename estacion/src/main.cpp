/*
 * main.cpp — ESTACION · banco de prueba del enlace LoRa (paso 3)
 * ------------------------------------------------------------------
 * ESTO NO ES EL FIRMWARE DEFINITIVO. Sustituye a Firebase con el teclado:
 * lo que en el paso 4 vendra de las claves de navegacion, /pwm y /motores,
 * aqui se teclea en el monitor serial.
 *
 * Sin WiFi todavia. Se puede flashear con la placa sola y la antena puesta.
 * ------------------------------------------------------------------
 */

#include <Arduino.h>

#include "lora_link.h"
#include "secrets.h"
#include "wifi_link.h"

/* ------------------------------------------------------------------
 *  Estado del banco
 * ------------------------------------------------------------------ */

static NavCmd   curNav      = NAV_STOP;
static uint8_t  curGrams    = 60;   /* el nivel mas bajo que ofrece la app */

/* Estado de la radio, para poder reintentar sin colgar la placa. */
static bool     radioReady     = false;
static uint32_t lastRadioTryMs = 0;
static uint32_t lastRadioMsgMs = 0;

/*
 * Vigilancia del tiempo de bucle.
 *
 * Este es el numero que decide si el paso 4 se puede quedar en un solo hilo
 * o hay que partirlo en tareas de FreeRTOS. La navegacion se refresca cada
 * NAV_REFRESH_MS (500 ms) y el deadman de Piscina salta a los 2000 ms, asi
 * que una iteracion que se acerque a 500 ms ya se esta comiendo el margen.
 *
 * El umbral esta en 400 ms: por debajo, el refresco llega a tiempo aunque se
 * junten un envio LoRa y una llamada de red en la misma vuelta.
 */
#define LOOP_WARN_MS   400

static uint32_t maxLoopMs = 0;

/* Para detectar cambios de estado del WiFi y contarlos una sola vez. */
static WifiState lastWifiState = WIFI_ST_IDLE;

static const char *navName(NavCmd nav) {
  switch (nav) {
    case NAV_STOP:    return "STOP";
    case NAV_FORWARD: return "ADELANTE";
    case NAV_REVERSE: return "ATRAS";
    case NAV_CW:      return "HORARIO";
    case NAV_CCW:     return "ANTIHORARIO";
    default:          return "?";
  }
}

static void printHelp() {
  Serial.println(F("\n--- teclas -------------------------------------"));
  Serial.println(F("  w / s     adelante / atras"));
  Serial.println(F("  d / a     giro horario / antihorario"));
  Serial.println(F("  x         parar"));
  Serial.println(F("  + / -     racion +-10 g  (clave /pwm, son gramos)"));
  Serial.println(F("  f         alimentar (con ACK y reintentos)"));
  Serial.println(F("  i         estadisticas del enlace"));
  Serial.println(F("  n         estado del WiFi"));
  Serial.println(F("  p         abrir el portal de configuracion"));
  Serial.println(F("  o         olvidar las redes guardadas"));
  Serial.println(F("  h         esta ayuda"));
  Serial.println(F("------------------------------------------------"));
  Serial.printf("Mientras la navegacion no sea STOP se reenvia el comando\n"
                "cada %d ms. En el banco la direccion se mantiene hasta\n"
                "pulsar 'x': el comportamiento momentaneo de verdad llega\n"
                "con Firebase en el paso 4.\n\n", NAV_REFRESH_MS);
}

static void printStats() {
  const LinkStats *s = linkStats();
  Serial.println(F("\n--- enlace -------------------------------------"));
  Serial.printf("  telemetrias   %lu\n", (unsigned long)s->rxTlm);
  Serial.printf("  ACK           %lu\n", (unsigned long)s->rxAck);
  Serial.printf("  descartados   %lu\n", (unsigned long)s->rxBad);
  Serial.printf("  perdidos      %lu   (huecos de secuencia)\n", (unsigned long)s->lost);
  Serial.printf("  transmitidos  %lu ok / %lu fallidos\n",
                (unsigned long)s->txOk, (unsigned long)s->txFail);
  Serial.printf("  reintentos de alimentacion  %lu\n", (unsigned long)s->feedRetries);
  Serial.printf("  peor tiempo de bucle  %lu ms   (aviso a partir de %d)\n",
                (unsigned long)maxLoopMs, LOOP_WARN_MS);
  Serial.printf("  ultimo paquete  RSSI %d dBm   SNR %.1f dB\n", s->rssi, s->snr);

  const uint32_t age = linkRxAgeMs();
  if (age == UINT32_MAX) {
    Serial.println(F("  ultimo paquete  NUNCA - Piscina no ha contestado"));
  } else {
    Serial.printf("  ultimo paquete  hace %lu ms\n", (unsigned long)age);
  }
  Serial.println(F("------------------------------------------------\n"));
}

static void printWifi() {
  Serial.println(F("\n--- WiFi ---------------------------------------"));
  Serial.printf("  estado    %s\n", wifiStateName());
  if (wifiConnected()) {
    Serial.printf("  red       %s\n", wifiSsid());
    Serial.printf("  IP        %s\n", wifiIp());
    Serial.printf("  senal     %d dBm\n", (int)wifiRssi());
  } else if (wifiState() == WIFI_ST_TRYING) {
    Serial.printf("  probando  %s\n", wifiSsid());
  }
  Serial.printf("  rondas fallidas  %lu\n", (unsigned long)wifiFailedRounds());
  Serial.println(F("  redes guardadas:"));
  for (uint8_t i = 0; i < WIFI_SLOTS; i++) {
    const char *s = wifiStoredSsid(i);
    Serial.printf("    [%u] %s\n", i, (s[0] != '\0') ? s : "(vacia)");
  }
  if (wifiPortalUp()) {
    Serial.printf("  PORTAL ABIERTO -> red \"%s\", abre http://%s\n",
                  AP_SSID, wifiApIp());
  } else {
    Serial.println(F("  portal cerrado (tecla 'p' para abrirlo)"));
  }
  const char *e = wifiLastAttemptError();
  if (e[0] != '\0') {
    Serial.printf("  ultimo fallo: %s\n", e);
  }
  Serial.println(F("------------------------------------------------\n"));
}

/* Avisa solo cuando el estado cambia, para no llenar el monitor. */
static void reportWifi() {
  const WifiState st = wifiState();
  if (st == lastWifiState) {
    return;
  }
  lastWifiState = st;

  switch (st) {
    case WIFI_ST_ONLINE:
      Serial.printf("[WiFi] conectado a %s  IP %s  %d dBm\n",
                    wifiSsid(), wifiIp(), (int)wifiRssi());
      break;
    case WIFI_ST_TRYING:
      Serial.printf("[WiFi] probando %s...\n", wifiSsid());
      break;
    case WIFI_ST_BACKOFF:
      if (wifiPortalUp()) {
        Serial.printf("[WiFi] sin red. PORTAL ABIERTO: conectate a \"%s\" "
                      "y abre http://%s\n", AP_SSID, wifiApIp());
      } else {
        Serial.println(F("[WiFi] ninguna red respondio; reintento en 15 s"));
      }
      break;
    case WIFI_ST_IDLE:
      break;
  }
}

/* ------------------------------------------------------------------
 *  Arranque
 * ------------------------------------------------------------------ */

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("=================================================="));
  /* Solo ASCII en lo que sale por serial: los guiones largos salen como
   * interrogaciones en varios monitores. */
  Serial.println(F(" ESTACION - banco de prueba del enlace LoRa"));
  Serial.println(F("=================================================="));
  Serial.printf("Radio: %.1f MHz  SF%d  BW %.0f kHz  CR 4/%d  %d dBm\n",
                LORA_FREQ_MHZ, LORA_SF, LORA_BW_KHZ, LORA_CR, LORA_TX_DBM);
  Serial.printf("Sync word: 0x%02X (privado)\n", LORA_SYNC_WORD);
  Serial.printf("ACK de alimentacion: %d ms de espera, %d reintentos\n",
                FEED_ACK_TIMEOUT_MS, FEED_ACK_RETRIES);

  radioReady = linkBegin();
  linkSetNav(curNav, curGrams);
  if (radioReady) {
    Serial.println(F("Radio lista."));
  }

  /*
   * El WiFi arranca DESPUES de la radio y a proposito: si arrancara antes y
   * algo fuera mal, el enlace LoRa — que es el que lleva el requisito de
   * seguridad — se quedaria esperando a un subsistema que no lo condiciona.
   * wifiBegin() no espera a conectar, solo lanza el intento.
   */
  wifiBegin();
  Serial.println(F("WiFi lanzado (sin esperar). Tecla 'n' para ver el estado."));

  if (radioReady) {
    printHelp();
  }
}

/*
 * Radio caida: informar y reintentar, en vez de colgarse en silencio.
 * Ver la nota equivalente en piscina/src/main.cpp.
 */
static bool radioRetry() {
  const uint32_t now = millis();

  if ((now - lastRadioMsgMs) >= 2000) {
    lastRadioMsgMs = now;
    Serial.print(F("ERROR: "));
    Serial.println(linkLastError());
    Serial.println(F("  Comprueba la antena. Si el fallo es de chip o SPI,"));
    Serial.println(F("  prueba LORA_TCXO_V a 1.8 en shared/protocol.h."));
  }

  if ((now - lastRadioTryMs) >= 5000) {
    lastRadioTryMs = now;
    radioReady = linkBegin();
    if (radioReady) {
      linkSetNav(curNav, curGrams);
      Serial.println(F("Radio lista tras reintento."));
      printHelp();
    }
  }
  return radioReady;
}

/* ------------------------------------------------------------------
 *  Teclado
 * ------------------------------------------------------------------ */

static void applyNav(NavCmd nav) {
  curNav = nav;
  linkSetNav(curNav, curGrams);
  linkSendCmd();                /* que salga ya, sin esperar al refresco */
  Serial.printf("[TX cmd] nav=%s racion=%u g\n", navName(curNav), curGrams);
}

static void handleKey(char c) {
  switch (c) {
    case 'w': case 'W': applyNav(NAV_FORWARD); break;
    case 's': case 'S': applyNav(NAV_REVERSE); break;
    case 'd': case 'D': applyNav(NAV_CW);      break;
    case 'a': case 'A': applyNav(NAV_CCW);     break;
    case 'x': case 'X': applyNav(NAV_STOP);    break;

    case '+':
      curGrams = (curGrams >= 100) ? 100 : (uint8_t)(curGrams + 10);
      applyNav(curNav);
      break;

    case '-':
      curGrams = (curGrams <= 10) ? 0 : (uint8_t)(curGrams - 10);
      applyNav(curNav);
      break;

    case 'f': case 'F':
      if (linkFeedState() == FEED_PENDING) {
        Serial.println(F("[FEED] ya hay una alimentacion esperando ACK"));
      } else {
        linkFeedClear();
        if (linkStartFeed()) {
          Serial.println(F("[FEED] enviado, esperando ACK..."));
        } else {
          Serial.println(F("[FEED] fallo el primer envio; se reintentara igual"));
        }
      }
      break;

    case 'i': case 'I': printStats(); break;
    case 'n': case 'N': printWifi();  break;
    case 'h': case 'H': printHelp();  break;

    case 'p': case 'P':
      wifiForcePortal();
      Serial.printf("[WiFi] portal abierto. Conectate a \"%s\" y abre http://%s\n",
                    AP_SSID, wifiApIp());
      break;

    case 'o': case 'O':
      wifiForgetAll();
      Serial.println(F("[WiFi] redes olvidadas. Aprovisiona desde el portal."));
      break;

    default: break;                   /* saltos de linea y demas, ignorados */
  }
}

/* ------------------------------------------------------------------
 *  Bucle
 * ------------------------------------------------------------------ */

static void reportFeed() {
  const FeedState st = linkFeedState();
  if (st == FEED_IDLE || st == FEED_PENDING) {
    return;
  }

  switch (st) {
    case FEED_DONE:
      Serial.printf("[FEED] confirmado: la racion salio (%s)\n",
                    linkFeedLastAck() == ACK_DUPLICATE ? "por reintento" : "a la primera");
      break;
    case FEED_BUSY:
      Serial.println(F("[FEED] rechazado: Piscina tenia un ciclo en curso"));
      break;
    case FEED_FAILED:
      /* Ojo con el matiz: no sabemos que NO salio, sabemos que no hubo
       * respuesta. La racion pudo salir y perderse el ACK de vuelta. */
      Serial.println(F("[FEED] sin respuesta tras los reintentos: SE DESCONOCE si salio"));
      break;
    default:
      break;
  }
  linkFeedClear();
}

static void reportTlm() {
  TlmPacket tlm;
  if (!linkTakeTlm(&tlm)) {
    return;
  }

  const LinkStats *s = linkStats();
  Serial.printf("[RX tlm  seq=%-5u] %.2f C  pH %.2f  ruido %u  estado 0x%02X"
                "  RSSI %d dBm  SNR %.1f dB  perdidos %lu\n",
                tlm.hdr.seq, tlm.temperature, tlm.ph, tlm.soundLevel, tlm.status,
                s->rssi, s->snr, (unsigned long)s->lost);
}

/* Mide lo que tardo la vuelta y avisa si aparece un peor caso preocupante. */
static void watchLoopTime(uint32_t startedMs) {
  const uint32_t dt = millis() - startedMs;
  if (dt <= maxLoopMs) {
    return;
  }
  maxLoopMs = dt;
  if (dt >= LOOP_WARN_MS) {
    Serial.printf("[BUCLE] nuevo peor caso: %lu ms. Con %d ms de refresco y "
                  "%d ms de deadman, esto se esta comiendo el margen.\n",
                  (unsigned long)dt, NAV_REFRESH_MS, NAV_DEADMAN_MS);
  }
}

void loop() {
  const uint32_t loopStart = millis();

  /* El WiFi se atiende SIEMPRE, tambien con la radio caida: son dos
   * subsistemas independientes y uno no debe secuestrar al otro. */
  wifiPoll();
  reportWifi();

  if (!radioReady) {
    /* Se vacia el teclado igualmente: si no, las teclas pulsadas mientras la
     * radio estaba caida se ejecutarian todas de golpe al recuperarse. */
    while (Serial.available() > 0) {
      (void)Serial.read();
    }
    radioRetry();
    watchLoopTime(loopStart);
    return;
  }

  linkPoll();

  while (Serial.available() > 0) {
    handleKey((char)Serial.read());
  }

  reportFeed();
  reportTlm();

  /*
   * Refresco de navegacion. Solo se reenvia mientras haya navegacion activa:
   * con STOP no hace falta alimentar el deadman, y callar libera el canal
   * para la telemetria.
   *
   * El plazo se cuenta desde la ULTIMA transmision de cualquier tipo, no
   * desde el ultimo refresco. Contarlo aparte hacia que un comando de
   * alimentacion no retrasara el refresco siguiente, y ese refresco salia
   * encima del ACK que veniamos esperando.
   */
  if (curNav != NAV_STOP && (millis() - linkLastTxMs()) >= NAV_REFRESH_MS) {
    linkSendCmd();
  }

  watchLoopTime(loopStart);
}
