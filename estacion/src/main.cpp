/*
 * main.cpp — ESTACION · paso 4
 * ------------------------------------------------------------------
 * Dos tareas de FreeRTOS, una por subsistema, comunicadas por colas.
 *
 * POR QUE ESTA PARTIDO, con los numeros que lo obligaron:
 *
 * En un solo bucle, medido en placa, la libreria de Firebase bloqueaba 200 ms
 * o mas en 68 de cada 416 llamadas — aproximadamente una por segundo — con
 * picos de 483 ms, y dejaba el bucle entero en 624 ms. El refresco de
 * navegacion es de 500 ms y el deadman de Piscina salta a los 2000 ms, asi
 * que cada bloqueo se comia el margen de seguridad. Y el peor caso real no
 * son 624 ms sino el timeout de red, 3 s, que si dispara el deadman y para
 * el catamaran a mitad de maniobra.
 *
 * Bajar el ritmo de sondeo de 12 000 a 10 llamadas por segundo redujo las
 * llamadas 2000 veces y NO quito los bloqueos: son inherentes a como la
 * libreria lee el socket. Por eso se parte, y no se ajusta.
 *
 * REPARTO
 *
 *   Nucleo 1, prioridad alta   tarea LoRa. Unica dueña de la radio.
 *                              Nada de lo que hace bloquea mas de lo que
 *                              dura un paquete en el aire.
 *
 *   Nucleo 0, prioridad baja   tarea de red. WiFi y Firebase. Que bloquee
 *                              lo que quiera: ahi ya vive la pila de red.
 *
 *   Bucle de Arduino           consola serial y diagnostico.
 *
 * La tarea de red NO toca la capa LoRa. Empuja comandos a una cola y la
 * tarea LoRa los aplica. Un solo dueño de la radio, cero estado mutable
 * compartido.
 * ------------------------------------------------------------------
 */

#include <Arduino.h>

#include "firebase_link.h"
#include "lora_link.h"
#include "secrets.h"
#include "wifi_link.h"

/* ==================================================================
 *  Mensajeria entre tareas
 * ================================================================== */

/* Red -> LoRa. Se manda solo cuando algo cambia. */
typedef struct {
  uint8_t nav;
  uint8_t grams;
  uint8_t sprayer;
  uint8_t feed;    /* 1 = disparar un ciclo */
} NetCmd;

/* LoRa -> consola, y en el paso 4d tambien a la red para escribirla. */
static QueueHandle_t qCmd = NULL;
static QueueHandle_t qTlm = NULL;

/*
 * Serial lo escriben las dos tareas y el bucle. Sin proteccion, dos printf
 * simultaneos salen entrelazados a mitad de palabra y el log deja de servir
 * justo cuando hace falta para depurar.
 */
static SemaphoreHandle_t serialMux = NULL;

static void logf(const char *fmt, ...) {
  if (serialMux == NULL) {
    return;
  }
  /* 512 y no 200: con 200 los bloques de diagnostico se cortaban a la mitad
   * y desaparecian justo las lineas del final, que son las que llevan el
   * ultimo error. Un log que trunca en silencio te esconde lo que buscas. */
  char buf[512];
  va_list va;
  va_start(va, fmt);
  const int n = vsnprintf(buf, sizeof(buf), fmt, va);
  va_end(va);

  if (xSemaphoreTake(serialMux, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.print(buf);
    /* Si aun asi no cupo, que se vea. Nunca truncar callando. */
    if (n >= (int)sizeof(buf)) {
      Serial.printf("...[cortado, faltan %d caracteres]\n", n - (int)sizeof(buf) + 1);
    }
    xSemaphoreGive(serialMux);
  }
}

/* ==================================================================
 *  Estado de consola
 * ================================================================== */

static NavCmd   curNav     = NAV_STOP;
static uint8_t  curGrams   = 60;   /* el nivel mas bajo que ofrece la app */
static uint8_t  curSprayer = 5;    /* medio recorrido del aspersor, 0-10  */

static bool     radioReady     = false;
static uint32_t lastRadioTryMs = 0;
static uint32_t lastRadioMsgMs = 0;

/*
 * Vigilancia del tiempo de la TAREA LORA, que es la que lleva el requisito
 * de seguridad. El umbral sigue en 400 ms: por debajo, el refresco de
 * navegacion llega a tiempo aunque coincida con un envio en el aire.
 *
 * La tarea de red ya no se mide aqui: puede tardar lo que quiera sin
 * consecuencias, y su peor llamada sale en las estadisticas de Firebase.
 */
#define LOOP_WARN_MS   400

static volatile uint32_t loraMaxMs     = 0;
static volatile uint32_t loraOverCount = 0;
static volatile uint32_t loraIters     = 0;

static WifiState lastWifiState = WIFI_ST_IDLE;
static bool      lastFbReady   = false;

/* Ultimo comando publicado por la tarea de red, para no repetir envios. */
static NavCmd  fbLastNav     = NAV_STOP;
static uint8_t fbLastGrams   = 0;
static uint8_t fbLastSprayer = 0;
static bool    fbEverApplied = false;

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

/* ==================================================================
 *  Consola
 * ================================================================== */

static void printHelp() {
  logf("\n--- teclas -------------------------------------\n"
       "  w / s     adelante / atras\n"
       "  d / a     giro horario / antihorario\n"
       "  x         parar\n"
       "  + / -     racion +-10 g  (clave /pwm, son gramos)\n"
       "  , / .     aspersor -+1   (clave /aspersor, nivel 0-10)\n"
       "  f         alimentar (con ACK y reintentos)\n"
       "  i         estadisticas del enlace\n"
       "  n         estado del WiFi\n"
       "  b         estado de Firebase\n"
       "  t         tareas y pila libre\n"
       "  p         abrir / cerrar el portal de configuracion\n"
       "  o         olvidar las redes guardadas\n"
       "  y         encender/apagar la escritura de telemetria\n"
       "  h         esta ayuda\n"
       "------------------------------------------------\n"
       "Las teclas conviven con Firebase: lo que llegue de la base\n"
       "sobrescribe lo tecleado en cuanto cambie.\n\n");
}

static void printStats() {
  const LinkStats *s = linkStats();
  logf("\n--- enlace -------------------------------------\n"
       "  telemetrias   %lu\n"
       "  ACK           %lu\n"
       "  descartados   %lu\n"
       "  perdidos      %lu   (huecos de secuencia)\n"
       "  transmitidos  %lu ok / %lu fallidos\n"
       "  reintentos de alimentacion  %lu\n"
       "  ultimo paquete  RSSI %d dBm   SNR %.1f dB\n",
       (unsigned long)s->rxTlm, (unsigned long)s->rxAck,
       (unsigned long)s->rxBad, (unsigned long)s->lost,
       (unsigned long)s->txOk, (unsigned long)s->txFail,
       (unsigned long)s->feedRetries, s->rssi, s->snr);

  const uint32_t age = linkRxAgeMs();
  if (age == UINT32_MAX) {
    logf("  ultimo paquete  NUNCA - Piscina no ha contestado\n");
  } else {
    logf("  ultimo paquete  hace %lu ms\n", (unsigned long)age);
  }
  logf("------------------------------------------------\n\n");
}

static void printTasks() {
  logf("\n--- tareas -------------------------------------\n"
       "  tarea LoRa (nucleo 1, prioridad alta)\n"
       "    iteraciones      %lu\n"
       "    peor iteracion   %lu ms   (aviso a partir de %d)\n"
       "    por encima       %lu\n",
       (unsigned long)loraIters, (unsigned long)loraMaxMs,
       LOOP_WARN_MS, (unsigned long)loraOverCount);
  logf("  Este es el numero que importa: la tarea LoRa lleva el\n"
       "  refresco de navegacion y el deadman. Lo que tarde la red\n"
       "  ya no la afecta.\n");

  /*
   * Memoria. Dos conexiones TLS simultaneas — el stream y la de escritura —
   * reservan buffers grandes, y si el monton se queda corto la libreria no
   * avisa con claridad: simplemente se le cae la conexion.
   */
  logf("\n  monton libre     %lu B\n"
       "  minimo historico %lu B   <- si esto baja mucho, el stream se cae por memoria\n"
       "  bloque mayor     %lu B\n",
       (unsigned long)ESP.getFreeHeap(),
       (unsigned long)ESP.getMinFreeHeap(),
       (unsigned long)ESP.getMaxAllocHeap());
  logf("------------------------------------------------\n\n");
}

static void printWifi() {
  logf("\n--- WiFi ---------------------------------------\n"
       "  estado    %s\n", wifiStateName());
  if (wifiConnected()) {
    logf("  red       %s\n  IP        %s\n  senal     %d dBm\n",
         wifiSsid(), wifiIp(), (int)wifiRssi());
  } else if (wifiState() == WIFI_ST_TRYING) {
    logf("  probando  %s\n", wifiSsid());
  }
  logf("  rondas fallidas  %lu\n  redes guardadas:\n",
       (unsigned long)wifiFailedRounds());
  for (uint8_t i = 0; i < WIFI_SLOTS; i++) {
    const char *s = wifiStoredSsid(i);
    logf("    [%u] %s\n", i, (s[0] != '\0') ? s : "(vacia)");
  }
  if (wifiPortalUp()) {
    logf("  PORTAL ABIERTO -> red \"%s\", abre http://%s\n", AP_SSID, wifiApIp());
  } else {
    logf("  portal cerrado (tecla 'p' para abrirlo)\n");
  }
  const char *e = wifiLastAttemptError();
  if (e[0] != '\0') {
    logf("  ultimo fallo: %s\n", e);
  }
  logf("------------------------------------------------\n\n");
}

static void printFirebase() {
  const FbStats *f = firebaseStats();
  const FbCommands *c = firebaseCommands();
  logf("\n--- Firebase -----------------------------------\n"
       "  lectura       %s\n"
       "  sondeos ok    %lu\n"
       "  reconexiones  %lu\n"
       "  errores       %lu\n"
       "  llamadas      %lu   (%lu lentas, >=%d ms)\n"
       "  media         %lu ms\n"
       "  peor llamada  %lu ms   (ya no afecta al enlace LoRa)\n"
       "  telemetria    %lu escritas / %lu fallidas / %lu claves NAN omitidas\n"
       "  comandos: nav=%s  racion=%u g  aspersor=%u/10\n",
       firebaseReady() ? "OK" : "sin conexion",
       (unsigned long)f->events, (unsigned long)f->reconnects,
       (unsigned long)f->errors, (unsigned long)f->calls,
       (unsigned long)f->slowCalls, FB_SLOW_CALL_MS,
       (unsigned long)(f->calls ? f->totalCallMs / f->calls : 0),
       (unsigned long)f->maxCallMs,
       (unsigned long)f->writes, (unsigned long)f->writeFails,
       (unsigned long)f->skippedNan,
       navName(c->nav), c->grams, c->sprayer);

  if (f->lastEventMs != 0) {
    logf("  ultimo evento hace %lu ms\n", (unsigned long)(millis() - f->lastEventMs));
  } else {
    logf("  NINGUNA lectura completada todavia\n");
  }

  char dbg[200];
  firebaseStreamDebug(dbg, sizeof(dbg));
  logf("  %s\n", dbg);

  const char *d = firebaseInitialDump();
  logf("  lectura inicial: %s\n", (d[0] != '\0') ? d : "(sin datos)");

  const char *e = firebaseLastError();
  if (e[0] != '\0') {
    logf("  ultimo error: %s\n", e);
  }
  logf("------------------------------------------------\n\n");
}

/* ==================================================================
 *  Tarea LoRa — nucleo 1, prioridad alta
 * ================================================================== */

/*
 * Radio caida: informar y reintentar en vez de colgarse en silencio.
 * La primera version imprimia el error una vez y se quedaba en un bucle
 * vacio; con el monitor cerrado en ese instante, la placa quedaba muda sin
 * forma de saber que le pasaba.
 */
static bool radioRetry() {
  const uint32_t now = millis();

  if ((now - lastRadioMsgMs) >= 2000) {
    lastRadioMsgMs = now;
    logf("ERROR: %s\n"
         "  Comprueba la antena. Si el fallo es de chip o SPI,\n"
         "  prueba LORA_TCXO_V a 1.8 en shared/protocol.h.\n", linkLastError());
  }
  if ((now - lastRadioTryMs) >= 5000) {
    lastRadioTryMs = now;
    radioReady = linkBegin();
    if (radioReady) {
      logf("Radio lista tras reintento.\n");
    }
  }
  return radioReady;
}

static void reportFeed() {
  const FeedState st = linkFeedState();
  if (st == FEED_IDLE || st == FEED_PENDING) {
    return;
  }
  switch (st) {
    case FEED_DONE:
      logf("[FEED] confirmado: la racion salio (%s)\n",
           linkFeedLastAck() == ACK_DUPLICATE ? "por reintento" : "a la primera");
      break;
    case FEED_BUSY:
      logf("[FEED] rechazado: Piscina tenia un ciclo en curso\n");
      break;
    case FEED_FAILED:
      /* No sabemos que NO salio: sabemos que no hubo respuesta. La racion
       * pudo salir y perderse el ACK de vuelta. */
      logf("[FEED] sin respuesta tras los reintentos: SE DESCONOCE si salio\n");
      break;
    default:
      break;
  }
  linkFeedClear();
}

static void taskLora(void *arg) {
  (void)arg;
  for (;;) {
    const uint32_t t0 = millis();

    if (!radioReady) {
      radioRetry();
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    linkPoll();

    /* Comandos que llegan de la red o de la consola. */
    NetCmd c;
    while (xQueueReceive(qCmd, &c, 0) == pdTRUE) {
      /*
       * El PARO se atiende ANTES de tocar nada mas y sin pasar por el resto
       * del flujo: es lo unico de esta cola que alguien puede estar pidiendo
       * con la maquina delante.
       */
      if (c.feed == FEED_ABORT) {
        linkSendAbort();
        logf("[PARO] abortar enviado (3 veces, sin ACK)\n");
        continue;
      }

      curNav     = (NavCmd)c.nav;
      curGrams   = c.grams;
      curSprayer = c.sprayer;
      linkSetNav(curNav, curGrams, curSprayer);
      linkSendCmd();

      if (c.feed == FEED_START) {
        if (linkFeedState() == FEED_PENDING) {
          logf("[FEED] ignorado: ya hay una alimentacion esperando ACK\n");
        } else {
          linkFeedClear();
          linkStartFeed();
          logf("[FEED] %u g enviados, esperando ACK...\n", curGrams);
        }
      }
    }

    reportFeed();

    TlmPacket tlm;
    if (linkTakeTlm(&tlm)) {
      const LinkStats *s = linkStats();
      logf("[RX tlm  seq=%-5u] %.2f C  pH %.2f  ruido %u  estado 0x%02X"
           "  RSSI %d dBm  SNR %.1f dB  perdidos %lu\n",
           tlm.hdr.seq, tlm.temperature, tlm.ph, tlm.soundLevel, tlm.status,
           s->rssi, s->snr, (unsigned long)s->lost);
      /* Se pasa a la red para el paso 4d. Si la cola esta llena se descarta:
       * mas vale perder una muestra que frenar la tarea de la radio. */
      xQueueSend(qTlm, &tlm, 0);
    }

    /* Refresco de navegacion, contado desde la ultima transmision de
     * cualquier tipo para no salir encima de un ACK que esperamos. */
    if (curNav != NAV_STOP && (millis() - linkLastTxMs()) >= NAV_REFRESH_MS) {
      linkSendCmd();
    }

    const uint32_t dt = millis() - t0;
    loraIters++;
    if (dt > loraMaxMs) {
      loraMaxMs = dt;
    }
    if (dt >= LOOP_WARN_MS) {
      loraOverCount++;
      logf("[LORA] iteracion larga: %lu ms\n", (unsigned long)dt);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ==================================================================
 *  Tarea de red — nucleo 0, prioridad baja
 * ================================================================== */

static void taskNet(void *arg) {
  (void)arg;
  for (;;) {
    wifiPoll();

    const WifiState st = wifiState();
    if (st != lastWifiState) {
      lastWifiState = st;
      switch (st) {
        case WIFI_ST_ONLINE:
          logf("[WiFi] conectado a %s  IP %s  %d dBm\n",
               wifiSsid(), wifiIp(), (int)wifiRssi());
          break;
        case WIFI_ST_TRYING:
          logf("[WiFi] probando %s...\n", wifiSsid());
          break;
        case WIFI_ST_BACKOFF:
          if (wifiPortalUp()) {
            logf("[WiFi] sin red. PORTAL ABIERTO: conectate a \"%s\" y abre http://%s\n",
                 AP_SSID, wifiApIp());
          } else {
            logf("[WiFi] ninguna red respondio; reintento en 15 s\n");
          }
          break;
        default:
          break;
      }
    }

    firebasePoll();

    if (firebaseReady() != lastFbReady) {
      lastFbReady = firebaseReady();
      logf(lastFbReady ? "[FB] stream abierto, escuchando comandos\n"
                       : "[FB] stream cerrado\n");
    }

    /*
     * PARO: se mira antes que cualquier otra cosa y se manda solo, sin
     * esperar a que cambien nav ni gramos.
     */
    if (firebaseTakeStop()) {
      NetCmd stop = { (uint8_t)NAV_STOP, 0, 0, (uint8_t)FEED_ABORT };
      if (xQueueSend(qCmd, &stop, 0) != pdTRUE) {
        logf("[PARO] cola llena; el paro NO salio\n");
      } else {
        logf("[PARO] recibido de Firebase, encolado\n");
      }
    }

    if (firebaseReady()) {
      const FbCommands *c = firebaseCommands();
      const bool changed = !fbEverApplied ||
                           c->nav     != fbLastNav ||
                           c->grams   != fbLastGrams ||
                           c->sprayer != fbLastSprayer;
      const bool feed = firebaseTakeFeed();

      if (changed || feed) {
        fbLastNav     = c->nav;
        fbLastGrams   = c->grams;
        fbLastSprayer = c->sprayer;
        fbEverApplied = true;

        NetCmd out = { (uint8_t)c->nav, c->grams, c->sprayer, feed ? (uint8_t)FEED_START : (uint8_t)FEED_NONE };
        if (xQueueSend(qCmd, &out, 0) != pdTRUE) {
          logf("[FB] cola de comandos llena; se descarta\n");
        } else {
          logf("[FB->LoRa] nav=%s racion=%u g aspersor=%u/10%s\n",
               navName(c->nav), c->grams, c->sprayer, feed ? "  + ALIMENTAR" : "");
        }
      }
    }

    /*
     * Telemetria hacia la base.
     *
     * Se vacia la cola entera y solo se escribe la MAS RECIENTE. Si la red
     * estuvo lenta y se acumularon tres muestras, escribir las tres seria
     * gastar tres viajes para dejar en la base exactamente el mismo valor
     * final. La app quiere el dato de ahora, no el historico.
     */
    TlmPacket tlm;
    bool haveTlm = false;
    while (xQueueReceive(qTlm, &tlm, 0) == pdTRUE) {
      haveTlm = true;
    }
    if (haveTlm) {
      firebaseWriteTlm(&tlm);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ==================================================================
 *  Consola — bucle de Arduino
 * ================================================================== */

/* Publica el estado de consola por la misma cola que usa la red, para que la
 * radio siga teniendo un unico dueño. */
static void pushCmd(bool feed) {
  NetCmd out = { (uint8_t)curNav, curGrams, curSprayer, feed ? (uint8_t)FEED_START : (uint8_t)FEED_NONE };
  xQueueSend(qCmd, &out, 0);
  logf("[TX cmd] nav=%s racion=%u g aspersor=%u/10%s\n",
       navName(curNav), curGrams, curSprayer, feed ? "  + ALIMENTAR" : "");
}

static void handleKey(char c) {
  switch (c) {
    case 'w': case 'W': curNav = NAV_FORWARD; pushCmd(false); break;
    case 's': case 'S': curNav = NAV_REVERSE; pushCmd(false); break;
    case 'd': case 'D': curNav = NAV_CW;      pushCmd(false); break;
    case 'a': case 'A': curNav = NAV_CCW;     pushCmd(false); break;
    case 'x': case 'X': curNav = NAV_STOP;    pushCmd(false); break;

    case '+': curGrams = (curGrams >= 100) ? 100 : (uint8_t)(curGrams + 10); pushCmd(false); break;
    case '-': curGrams = (curGrams <= 10)  ? 0   : (uint8_t)(curGrams - 10); pushCmd(false); break;

    case '.': curSprayer = (curSprayer >= SPRAYER_LEVEL_MAX)
                             ? SPRAYER_LEVEL_MAX : (uint8_t)(curSprayer + 1);
              pushCmd(false); break;
    case ',': curSprayer = (curSprayer == 0) ? 0 : (uint8_t)(curSprayer - 1);
              pushCmd(false); break;

    case 'f': case 'F': pushCmd(true); break;

    case 'i': case 'I': printStats();    break;
    case 'n': case 'N': printWifi();     break;
    case 'b': case 'B': printFirebase(); break;
    case 't': case 'T': printTasks();    break;
    case 'h': case 'H': printHelp();     break;

    case 'p': case 'P':
      /* Interruptor, no solo encendido: si no, la unica forma de bajar un
       * portal abierto a mano seria reiniciar la placa. */
      if (wifiPortalUp()) {
        wifiClosePortal();
        logf("[WiFi] portal cerrado.\n");
      } else {
        wifiForcePortal();
        logf("[WiFi] portal abierto. Conectate a \"%s\" y abre http://%s\n"
             "       Se cerrara solo en 5 min mientras haya red.\n",
             AP_SSID, wifiApIp());
      }
      break;

    case 'o': case 'O':
      wifiForgetAll();
      logf("[WiFi] redes olvidadas. Aprovisiona desde el portal.\n");
      break;

    case 'y': case 'Y':
      firebaseSetTlmEnabled(!firebaseTlmEnabled());
      logf("[FB] escritura de telemetria %s\n",
           firebaseTlmEnabled() ? "ENCENDIDA" : "APAGADA (solo para aislar el stream)");
      break;

    default: break;
  }
}

/* ==================================================================
 *  Arranque
 * ================================================================== */

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) {
    delay(10);
  }

  serialMux = xSemaphoreCreateMutex();

  Serial.println();
  Serial.println(F("=================================================="));
  /* Solo ASCII en lo que sale por serial: los guiones largos salen como
   * interrogaciones en varios monitores. */
  Serial.println(F(" ESTACION - paso 4 (WiFi + Firebase + LoRa)"));
  Serial.println(F("=================================================="));
  Serial.printf("Radio: %.1f MHz  SF%d  BW %.0f kHz  CR 4/%d  %d dBm\n",
                LORA_FREQ_MHZ, LORA_SF, LORA_BW_KHZ, LORA_CR, LORA_TX_DBM);
  Serial.printf("Sync word: 0x%02X (privado)\n", LORA_SYNC_WORD);

  qCmd = xQueueCreate(8, sizeof(NetCmd));
  qTlm = xQueueCreate(4, sizeof(TlmPacket));
  if (qCmd == NULL || qTlm == NULL) {
    Serial.println(F("ERROR: no hay memoria para las colas"));
    while (true) { delay(1000); }
  }

  radioReady = linkBegin();
  linkSetNav(curNav, curGrams, curSprayer);
  Serial.println(radioReady ? F("Radio lista.") : F("Radio NO arranco; se reintentara."));

  wifiBegin();
  firebaseBegin();
  Serial.println(F("WiFi y Firebase lanzados, sin esperar a que conecten."));

  /*
   * La tarea LoRa va al nucleo 1 con prioridad alta, y la de red al nucleo 0
   * con prioridad baja. El nucleo 0 es donde ya corre la pila WiFi del
   * ESP32, asi que la red queda agrupada y el nucleo 1 se dedica al enlace
   * que lleva el requisito de seguridad.
   *
   * Pila de 10 kB para la red: TLS y el cliente de Firebase la consumen sin
   * miramientos. La tarea LoRa se apaña con 4 kB.
   */
  xTaskCreatePinnedToCore(taskLora, "lora", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskNet,  "net", 10240, NULL, 1, NULL, 0);

  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    handleKey((char)Serial.read());
  }
  vTaskDelay(pdMS_TO_TICKS(20));
}
