# 🦐 SAMNA- SISTEMA DE ALIMENTACIÓN Y MONITOREO CON NAVEGACIÓN AUTÓNOMA

Sistema autónomo de **alimentación, monitoreo y navegación** para piscinas
camaroneras. Un catamarán flotante dosifica balanceado, mide los parámetros del
agua y se opera a distancia desde una app Android. En este repositorio consta el código correspondiente a los microcontroladores utilizados para gobernar el hardware.

Proyecto de titulación — **ESPOL**.

> **Estado:** ✅ Prototipo terminado y verificado con hardware real.

---

## 🛰️ Arquitectura

Dos placas **Heltec WiFi LoRa 32 V3** enlazadas por **LoRa 915 MHz**
bidireccional:

```
   📱 App Android
        │  Firebase RTDB
        ▼
   🏠 ESTACION  ──────  LoRa 915 MHz  ──────  🚤 PISCINA
   puente WiFi           SF9 · 125 kHz          catamarán flotante
   alimentación de red   CR 4/5 · 22 dBm        batería + solar
```

| Unidad | Rol |
|---|---|
| 🚤 **Piscina** | Catamarán flotante. Sensores, dosificación por tornillo sinfín, aspersión y propulsión diferencial. Batería LiFePO4 con aporte solar. |
| 🏠 **Estacion** | Puente entre Firebase y el enlace LoRa. Lee los comandos de la app, los reenvía por radio y publica la telemetría de vuelta. |

**Comunicación:** paquetes binarios de tamaño fijo, con ACK solo en los
comandos de alimentación. Telemetría cada 2 s. *Deadman* de 2 s en
navegación: si se cae el enlace, los propulsores vuelven a neutro solos.

---

## 🔧 Hardware

### 🧠 Cómputo y radio

| Componente | Cantidad | Detalle |
|---|:---:|---|
| Heltec WiFi LoRa 32 V3 | 2 | ESP32-S3FN8 + SX1262 · OLED integrado |
| Antena 915 MHz | 2 | una por placa |

### 🌡️ Sensores — unidad Piscina

| Componente | Interfaz | Mide | Notas |
|---|---|---|---|
| **DS18B20** | 1-Wire · GPIO 7 | Temperatura del agua (°C) | Pull-up de 4,7 kΩ externa |
| **Sonda de pH B09H1MJS4S** | Analógica · GPIO 5 | pH 0–14 | Módulo de 5 V + divisor 10k/10k |
| **MAX4466** | Analógica · GPIO 6 | Actividad de alimentación (índice 0–100) | Alimentar a 3V3, nunca a 5 V |

### ⚙️ Actuadores de alimentación

| Componente | Cantidad | Función |
|---|:---:|---|
| **Motorreductor JGB37-520** 12 V · 60 rpm | 1 | Tornillo sinfín dosificador — un solo sentido, siempre al 100 % |
| **Motorreductor JGB37-520** 12 V · 600 rpm | 1 | Disco aspersor — velocidad variable, regula el radio de esparcido |
| **Driver BTS7960** | 2 | Medio puente por motor · solo se gobierna `RPWM` |

### 🚀 Propulsión

| Componente | Cantidad | Detalle |
|---|:---:|---|
| **APISQUEEN U2 MINI** | 2 | Brushless sumergible · 12–16 V · 8 A · 130 W |
| **ESC bidireccional** | 2 | Señal de servo 1000–2000 µs · 1500 µs = neutro |

Gobierno **diferencial** (estilo tanque): avante, atrás y giro sobre el eje
por empuje opuesto de ambos propulsores.

### 🔋 Energía

| Componente | Valor | Función |
|---|---|---|
| **Batería LiFePO4 4S** | 12,8 V · BMS ≥ 30 A | Alimentación principal |
| **Panel solar** | 20 W | Recarga en operación |
| **Controlador MPPT** | perfil LiFePO4 | Gestión de carga |
| **Convertidor buck** | LM2596 / MP1584EN · 12 → 5 V | Alimenta la Heltec |
| **Fusible** | 25–30 A | Pegado al borne positivo, antes que nada |
| **Interruptor** | ≥ 25 A | Corte general |

> ⚠️ Con el buck montado, **no conectar el USB a la vez**: son dos fuentes
> enfrentadas en el mismo raíl de 5 V. Para flashear, cortar antes los 12 V.

### 📌 Asignación de GPIO — Piscina

| GPIO | Destino | Régimen |
|:---:|---|---|
| `5` | Sonda de pH | ADC1_CH4 |
| `6` | MAX4466 | ADC1_CH5 |
| `7` | DS18B20 | 1-Wire |
| `47` | BTS7960 #1 — sinfín | LEDC 1 kHz · 8 bits |
| `48` | BTS7960 #2 — aspersor | LEDC 1 kHz · 8 bits |
| `2` | ESC babor | LEDC 50 Hz · 14 bits |
| `4` | ESC estribor | LEDC 50 Hz · 14 bits |
| `8·12·13·14` | SX1262 — CS, RST, BUSY, DIO1 | soldado en placa |

---

## 📂 Estructura

```
piscina/      proyecto PlatformIO — unidad flotante
estacion/     proyecto PlatformIO — estación base
shared/       protocol.h — contrato de mensajes LoRa, incluido por ambos
datos/        registros de calibración y pruebas de campo
docs/         mapa de conexiones, esquemas, energía y proceso de diseño
```

Son **dos proyectos PlatformIO separados** que comparten `shared/protocol.h`.

---

## 📖 Documentación

Páginas HTML autocontenidas — se abren con doble clic, sin servidor.

| Documento | Contenido |
|---|---|
| 🗺️ [`docs/mapa.html`](docs/mapa.html) | Mapa completo de conexiones con la Heltec en el centro |
| 🔌 [`docs/esquemas.html`](docs/esquemas.html) | Cableado detallado de cada bloque, con y sin desacoplo |
| 🔋 [`docs/energia.html`](docs/energia.html) | Picos de corriente, criterio de batería y BMS, masas en estrella |
| 📐 [`docs/proceso.html`](docs/proceso.html) | Proceso de diseño: 8 pasos, 27 sub-pasos, mediciones y fallos encontrados |
| 🔧 [`docs/conexiones.html`](docs/conexiones.html) | Inventario de cables y orden de montaje |
| 💻 [`docs/setup.md`](docs/setup.md) | Montar el proyecto en otra máquina |

---

## 🚀 Puesta en marcha

**1. Requisitos**

- 🧰 VS Code
- 🔌 Extensión **PlatformIO IDE** (instalar primero; la primera ejecución
  descarga el toolchain y tarda unos minutos)

**2. Abrir el proyecto**

Abrir `alimentadora.code-workspace` — no la carpeta suelta. El workspace
multi-raíz hace que PlatformIO reconozca los dos proyectos por separado.

**3. Credenciales**

```bash
cp estacion/include/secrets.h.example estacion/include/secrets.h
```

Rellenar `AP_PASSWORD` y `FIREBASE_AUTH`. Las credenciales **WiFi no van
aquí**: se aprovisionan desde el móvil por el portal SoftAP y viven en la NVS
de la Estacion. `secrets.h` está en `.gitignore`; **nunca se sube**.

**4. Compilar**

Selector de entorno de PlatformIO en la barra inferior → elegir `piscina` o
`estacion` → Build (✓) → Upload (→).

---

## ⚖️ Calibración de dosificación

El control es de **lazo abierto a velocidad fija**: `t_on = M_objetivo / ṁ`.
El sinfín corre siempre al 100 % — la masa se controla con el tiempo, no con
la velocidad — así que `ṁ` es una sola constante.

Para obtener `ṁ`:

1. 🔄 Correr el sinfín 10 s al 100 % descargando sobre una balanza.
2. ⚖️ Pesar lo dosificado y dividir entre 10 → g/s.
3. 🔁 Repetir 3 veces y promediar.
4. 💾 Guardarlo por serial: `ar <ms>` corre el sinfín ese tiempo con la rampa
   real, `ag <g>` le dice cuánto pesó lo que salió y calcula `ṁ`, y
   `am <g/s>` lo fija a mano si se midió fuera.

El valor queda en la **NVS de Piscina**, no en una constante compilada:
sobrevive a un reflasheo y no depende de la máquina desde la que se compile.

> Usar la misma rampa de arranque de ~200 ms que el firmware de producción:
> los gramos que salen durante la rampa entran en el promedio, y calibrar con
> un arranque distinto al de uso mete un sesgo sistemático.

**Valor medido:** `ṁ` = 32,5 g/s — promedio de 5 ensayos.
**Verificación:** ración de 60 g pedida desde la app → 59 g pesados
(1,7 % de error).

---

## 🧪 Registro de pruebas

Los datos de campo van en `datos/` — un CSV por ensayo, nombrado
`AAAA-MM-DD_ensayo.csv`.

| Ensayo | Qué mide | Criterio |
|---|---|---|
| 📡 Alcance LoRa | RSSI, SNR, paquetes perdidos vs distancia | ≥ 95 % entrega a 500 m |
| ⚖️ Dosificación | masa dosificada vs objetivo | ± 5 % |
| 🌡️ Temperatura | DS18B20 vs termómetro de referencia | ± 0.5 °C |
| 🧪 pH | lecturas en soluciones buffer | estable |
| 🔋 Autonomía | consumo y voltaje de batería en 24 h | ≥ 24 h |

---

## 👥 Autores

**Mario Viteri** · **Erick Salorzano**
Escuela Superior Politécnica del Litoral (ESPOL)
