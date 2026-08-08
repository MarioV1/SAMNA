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

/* ------------------------------------------------------------------
 *  Estado del banco
 * ------------------------------------------------------------------ */

static NavCmd   curNav      = NAV_STOP;
static uint8_t  curPwm      = 50;

/* Estado de la radio, para poder reintentar sin colgar la placa. */
static bool     radioReady     = false;
static uint32_t lastRadioTryMs = 0;
static uint32_t lastRadioMsgMs = 0;

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
  Serial.println(F("  + / -     velocidad del aspersor +-10 %"));
  Serial.println(F("  f         alimentar (con ACK y reintentos)"));
  Serial.println(F("  i         estadisticas del enlace"));
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
  Serial.printf("  ultimo paquete  RSSI %d dBm   SNR %.1f dB\n", s->rssi, s->snr);

  const uint32_t age = linkRxAgeMs();
  if (age == UINT32_MAX) {
    Serial.println(F("  ultimo paquete  NUNCA - Piscina no ha contestado"));
  } else {
    Serial.printf("  ultimo paquete  hace %lu ms\n", (unsigned long)age);
  }
  Serial.println(F("------------------------------------------------\n"));
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
  linkSetNav(curNav, curPwm);
  if (radioReady) {
    Serial.println(F("Radio lista."));
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
      linkSetNav(curNav, curPwm);
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
  linkSetNav(curNav, curPwm);
  linkSendCmd();                /* que salga ya, sin esperar al refresco */
  Serial.printf("[TX cmd] nav=%s pwm=%u%%\n", navName(curNav), curPwm);
}

static void handleKey(char c) {
  switch (c) {
    case 'w': case 'W': applyNav(NAV_FORWARD); break;
    case 's': case 'S': applyNav(NAV_REVERSE); break;
    case 'd': case 'D': applyNav(NAV_CW);      break;
    case 'a': case 'A': applyNav(NAV_CCW);     break;
    case 'x': case 'X': applyNav(NAV_STOP);    break;

    case '+':
      curPwm = (curPwm >= 100) ? 100 : (uint8_t)(curPwm + 10);
      applyNav(curNav);
      break;

    case '-':
      curPwm = (curPwm <= 10) ? 0 : (uint8_t)(curPwm - 10);
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
    case 'h': case 'H': printHelp();  break;
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

void loop() {
  if (!radioReady) {
    /* Se vacia el teclado igualmente: si no, las teclas pulsadas mientras la
     * radio estaba caida se ejecutarian todas de golpe al recuperarse. */
    while (Serial.available() > 0) {
      (void)Serial.read();
    }
    radioRetry();
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
}
