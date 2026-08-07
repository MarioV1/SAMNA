# Alimentadora Acuícola

Sistema de alimentación, monitoreo y navegación para piscina camaronera.
Proyecto de titulación — ESPOL.

Dos placas Heltec WiFi LoRa 32 V3 enlazadas por LoRa 915 MHz:

- **Piscina** — catamarán flotante. Sensores (temperatura, pH, sonido),
  dosificación por tornillo sinfín, aspersión y propulsión diferencial.
- **Estacion** — puente WiFi/Firebase entre la app Android y la piscina.

El contexto técnico completo (hardware, pinout, esquema Firebase, modelo de
control) está en [`CLAUDE.md`](CLAUDE.md).

---

## Estructura

```
piscina/      proyecto PlatformIO — unidad flotante
estacion/     proyecto PlatformIO — estación base
shared/       protocol.h — contrato de mensajes LoRa, incluido por ambos
datos/        registros de calibración y pruebas de campo
docs/         esquemáticos, fichas técnicas, capítulos de tesis
```

---

## Puesta en marcha

**1. Requisitos**

- VS Code
- Extensión **PlatformIO IDE** (instalar primero; la primera ejecución
  descarga el toolchain y tarda unos minutos)
- Extensión **Claude Code**

**2. Abrir el proyecto**

Abrir `alimentadora.code-workspace` — no la carpeta suelta. El workspace
multi-raíz hace que PlatformIO reconozca los dos proyectos por separado.

**3. Credenciales**

```bash
cp estacion/include/secrets.h.example estacion/include/secrets.h
```

Rellenar WiFi y Firebase. `secrets.h` está en `.gitignore`; nunca se sube.

**4. Compilar**

Selector de entorno de PlatformIO en la barra inferior → elegir `piscina` o
`estacion` → Build (✓) → Upload (→).

---

## Calibración de dosificación

El control es de lazo abierto: `t_on = M_objetivo / ṁ`.

Para obtener `ṁ`:

1. Correr el sinfín 10 s al 100 % descargando sobre una balanza.
2. Pesar lo dosificado y dividir entre 10 → g/s.
3. Repetir 3 veces y promediar.
4. Escribir el valor en la constante correspondiente del firmware.

Repetir al 50 % y al 30 % para verificar linealidad. Por debajo de ~25 % el
motor puede calar bajo carga.

---

## Registro de pruebas

Los datos de campo van en `datos/` — un CSV por ensayo, nombrado
`AAAA-MM-DD_ensayo.csv`. Sirven de anexo para la tesis:

| Ensayo | Qué mide | Criterio |
|---|---|---|
| Alcance LoRa | RSSI, SNR, paquetes perdidos vs distancia | ≥ 95 % entrega a 500 m |
| Dosificación | masa dosificada vs objetivo | ± 5 % |
| Temperatura | DS18B20 vs termómetro de referencia | ± 0.5 °C |
| pH | lecturas en soluciones buffer | estable |
| Autonomía | consumo y voltaje de batería en 24 h | ≥ 24 h |
