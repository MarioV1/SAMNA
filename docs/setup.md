# Montar el proyecto en otra computadora

Qué hace falta llevar de una máquina a otra, y qué no.

---

## Lo único que falta al clonar

Un solo archivo: **`estacion/include/secrets.h`**.

Piscina no lo usa — no tiene WiFi ni Firebase, así que no hay nada que llevarle.

```bash
git clone https://github.com/MarioV1/alimentadora.git
cd alimentadora
cp estacion/include/secrets.h.example estacion/include/secrets.h
```

La plantilla ya trae las cuatro claves que el código realmente usa. Solo hay
que rellenar dos:

| Clave | De dónde sale |
|---|---|
| `AP_SSID` | ya viene buena en la plantilla |
| `AP_PASSWORD` | la eliges tú — **mínimo 8 caracteres** |
| `FIREBASE_HOST` | ya viene buena en la plantilla |
| `FIREBASE_AUTH` | consola de Firebase → Configuración del proyecto → Cuentas de servicio → Secretos de base de datos |

El mínimo de 8 caracteres de `AP_PASSWORD` no es un consejo: con menos, WPA2 no
puede aplicarse y el punto de acceso quedaría **abierto**. Hay un
`static_assert` en `wifi_link.cpp` que lo impide al compilar.

### Mejor cópialo en un USB que regenerarlo

Funcionan las dos, pero copiar el archivo evita un despiste: si en la máquina
nueva pones un `AP_PASSWORD` **distinto** al que tiene la placa ya flasheada,
el portal de aprovisionamiento seguirá pidiendo el viejo hasta que la
reflashees desde la máquina nueva. Es el único archivo del USB.

---

## Tus credenciales WiFi no están en el repositorio

Ni en `secrets.h`. El SSID y la clave de tu red viven en la **NVS de la
Estacion**, donde entraron por el portal desde el celular. No hay nada que
transportar: la placa se sigue conectando sola aunque cambies de computadora.

Lo mismo vale para toda la calibración — `ṁ` = 32,5 g/s, el pH, el duty mínimo
del aspersor, el empuje de los propulsores. Está en la **NVS de Piscina**.
Viaja con el hardware, no con el PC.

> ⚠️ **No hagas `erase` al estrenar máquina.**
> Reflashear normal (`pio run -t upload`) no toca la NVS y la calibración
> sobrevive. Pero `pio run -t erase` y el *Erase Flash* de PlatformIO la borran
> entera — y ese es justo el reflejo cuando algo no sube en un equipo nuevo.
> Se perderían las cinco pruebas de dosificación y las de pH.

---

## Lo que NO hay que copiar

| Carpeta / archivo | Por qué |
|---|---|
| `.pio/` | se regenera sola, y guarda rutas absolutas de la otra máquina |
| `.vscode/c_cpp_properties.json`, `launch.json` | igual: rutas absolutas que PlatformIO reescribe |

Están en `.gitignore` por esto mismo. Copiarlas da errores de compilación que
parecen del código y son del entorno.

El proyecto puede clonarse en **cualquier carpeta**: los `platformio.ini`
referencian `shared/` en relativo (`-I../shared`), así que no dependen de que
esté en `C:\dev`.

---

## Lo que sí hay que instalar

- **VS Code** y la extensión **PlatformIO IDE**
- El driver **CP2102 de Silicon Labs**, si no aparecen los puertos COM al
  conectar las placas. El conector USB de la Heltec V3 va a un puente CP2102,
  no al USB nativo del ESP32‑S3
- La primera compilación descarga el toolchain de Xtensa y las librerías —
  varios cientos de MB. Hazla con internet y sin prisa

Todas las versiones están fijadas en los `platformio.ini`
(`espressif32 @ 6.10.0`, `RadioLib @ 6.6.0`, `Firebase ESP32 Client @ 4.4.17`…),
así que la máquina nueva compila exactamente lo mismo que la vieja.

Abrir **`alimentadora.code-workspace`**, no la carpeta suelta: el workspace
multi‑raíz es lo que hace que PlatformIO reconozca los dos proyectos por
separado.

---

## Trabajando en dos máquinas

`git pull` antes de empezar, en cualquiera de las dos. Ahora hay dos sitios
desde donde tocar lo mismo, y `shared/protocol.h` es el archivo donde una
divergencia duele más: si las dos placas se flashean con versiones distintas
del protocolo, los `static_assert` no lo detectan — cada una compila coherente
consigo misma y el desajuste solo aparece en el aire, como paquetes
descartados.
