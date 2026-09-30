# TID — Firmware del Nodo Motor Diésel (RIO-DSL)

Firmware para STM32G431KBT6U (Nucleo-32) que mide RPM de un motor diésel
vía el alternador, controla el acelerador con un lazo PID sobre un
servo, y se comunica por LoRaWAN (RAK3172), con calibración/configuración
remota mediante un protocolo de parámetros propio. Pendiente: ganancias
reales del PID (sintonización empírica en campo, ver sección 8), la
máquina de modos del gobernador (0/1/2), y el sensor de presión de
entrada 4-20mA.

---

## 1. Hardware

La documentación de hardware (mapa de pines completo, alimentación,
circuitos de acondicionamiento de señal, y hallazgos de campo sobre
componentes físicos) vive en un repo/documento separado:

📄 **[`TIDHardware/TID/README.md`](../../TIDHardware/TID/README.md)**

Este README (firmware) referencia ese documento cada vez que hace
falta el detalle físico de un circuito — busca los enlaces "ver
hardware" a lo largo de las secciones siguientes.

---

## 2. Módulos del firmware

```
tacometro.c/h          -> Medicion de RPM (Input Capture + filtro EMA)
rak3172.c/h             -> Comunicacion con el modulo LoRaWAN + sincronizacion de hora
calibracion_flash.c/h   -> Persistencia de parametros + dispatcher de downlinks
servo.c/h               -> Control PWM del servo del acelerador
pid.c/h                 -> Lazo de control PID de RPM -> pulso del servo
rtc_reloj.c/h           -> Reloj interno (RTC sobre LSI) + conversion epoch<->calendario
gps.c/h                 -> Posicion GPS/GNSS (SIM7600X) + disciplina del RTC
comando_serial.c/h      -> Mando manual TEMPORAL por serial (sustituto de LoRa en banco)
main.c                  -> Orquestacion general
```

### 2.1 `tacometro.c/h`

Mide la frecuencia del alternador con **TIM2 en modo PWM Input** (Slave
Mode Reset en TI1FP1): el canal directo captura el período completo en
cada flanco de subida, el canal indirecto captura el ancho de pulso en
el mismo instante — sin condición de carrera entre dos capturas
separadas.

- **Filtro de ruido en 2 capas**: filtro de hardware del timer
  (`ICFilter=8`) + guarda de período mínimo por software
  (`TACOMETRO_PERIODO_MINIMO_US`).
- **Filtro de duty cycle** (`TACOMETRO_DUTY_MINIMO/MAXIMO`): descarta
  picos angostos que no representan un semiciclo real de la señal.
- **Detección de motor detenido**: timeout sin capturas nuevas
  (`TACOMETRO_TIMEOUT_DETENIDO_MS`) → `Tacometro_EstaDetenido()`. Este
  es el criterio único para saber si el motor opera — **no depende de
  ningún switch físico de encendido**.
- **Filtro EMA** (media móvil exponencial) sobre la RPM instantánea,
  coeficiente configurable en caliente (`ALPHA`, ver protocolo de
  parámetros).

Circuito de entrada (H11AA1 + acondicionamiento de la señal `W` del
alternador) → ver hardware, sección 3. Factor de calibración de campo
confirmado: **`SET_RATIO = 17.5`** pulsos/revolución (incluye relación
de poleas alternador:motor, con 1 pulso/ciclo AC confirmado en campo
con el motor real) — es el default de `TACOMETRO_PULSOS_POR_REVOLUCION`.

⚠️ **Ruido con la entrada del tacómetro sin conectar (caso de banco,
2026-09-23)**: con la línea totalmente al aire (sin alternador ni
generador de señal), se confirmó en campo, analizando el log completo
de una sesión (~43,800 líneas) con los campos de diagnóstico de la
subsección siguiente, que capta ruido electromagnético ambiente y lo
convierte en flancos aleatorios en el pin de Input Capture — sin
correlación con el TX del RAK3172 ni con los reportes del GPS (se
descartó que el propio nodo se auto-induzca el ruido). El filtro de 2
capas de arriba rechaza la gran mayoría (contador
`Tacometro_GetContadorRuidoFiltrado()`, visible en el log como
`RuidoFiltrado`), pero por diseño valida **pulsos individuales**, no el
contexto de varios segundos — así que, muy ocasionalmente (2 veces en
esa sesión), 1-2 flancos de ruido caen por azar dentro de la ventana de
período/duty cycle válida y producen una lectura de RPM físicamente
imposible (2000+ RPM) sostenida solo una fracción de segundo. Sin el
debounce de estado de la subsección siguiente, esto reiniciaba
`inicio_operacion` sin que el motor hubiera cambiado de estado de
verdad. Con la línea conectada a una fuente real (alternador o
generador de señal), se espera que este camino de ruido desaparezca o
sea mucho menos frecuente.

### 2.2 `rak3172.c/h`

Comunicación con el módulo LoRaWAN por USART1, DMA en modo Normal.

- **Recepción**: buffer circular propio, ensamblado de líneas por
  `\n`, cola de líneas pendientes para procesar fuera de contexto de
  interrupción (`RAK3172_Update()`).
- **Envío de comandos AT**: bloqueante (`HAL_UART_Transmit`), con
  timeout configurable.
- **Application ACK**: si el canal AT está ocupado al momento de
  mandar un ACK, se encola para reintento automático (hasta
  `RAK3172_ACK_REINTENTO_TIMEOUT_MS`, 5s).
- **Parser de eventos `+EVT:`**: formato real confirmado en campo
  (firmware RUI_4.2.4_RAK3172-E):
  ```
  +EVT:RX_1:-28:8:UNICAST:2:00000710
       |    |   |    |    |    +-- payload en hex: [ID][valor]
       |    |   |    |    +------- FPort
       |    |   |    +------------ tipo (UNICAST)
       |    |   +----------------- SNR
       |    +--------------------- RSSI
       +-------------------------- ventana de recepcion
  ```

**Configuración crítica que resolvió un bug de cuelgue en reset**
(con el RAK3172 conectado): USART1 (no LPUART1), DMA modo Normal (no
Circular), `Overrun`/`DMA on RX Error` deshabilitados en el `.ioc`,
pull-up en el pin RX (ver hardware, sección 4), y limpieza de flags de
error antes de rearmar la recepción en cada `RAK3172_Init()`.

**FPorts usados**:

| FPort | Uso |
|---|---|
| 1 | Uplink LIVE (28 bytes: RPM + presión + estado + timestamps + lat/lon + código de alerta) |
| 2 | Downlink de parámetro (ver protocolo abajo) |
| 3 | Application ACK |

**Estado LoRaWAN**: Clase C, join OTAA. ⚠️ El RAK3172 **no cambia de
clase automáticamente** solo porque el Device Profile en el servidor
diga "admite Clase C" — hay que mandarle explícitamente `AT+CLASS=C`
(default de fábrica es Clase A).

⚠️ **`AutoJoin` en `0`, no `1`, como prueba temporal en curso**
(`RAK3172_Join()` manda `AT+JOIN=1:0:10:8`): se cambió para aislar si
ese parámetro específico causaba el `AT_ERROR` de join visto en campo.
La causa real resultó ser el bug de sub-banda de abajo (`AT+MASK`), ya
resuelto — falta decidir si `AutoJoin` vuelve a `1` en producción
(reintento automático interno del propio módulo tras un power-cycle,
sin que el host tenga que pedir el join de nuevo) o si conviene dejarlo
en `0` y manejar todos los reintentos desde el host, como ya se hace
ahora (ver más abajo). Ver pendientes, sección 8.

#### Orden de arranque, join, y recuperación ante caídas — LoRa como prioridad

Requisito explícito del proyecto: **el join LoRaWAN debe intentarse
antes que cualquier otra cosa en el arranque**, y el nodo debe
reconectarse solo si pierde el join en cualquier momento, sin necesidad
de un reinicio manual del STM32. Así quedó implementado en `main.c` y
`rak3172.c`:

**1. Arranque — LoRa antes que el GPS.** `RAK3172_Init()` corre
inmediatamente después de `Tacometro_Init()`, seguido de `AT+MASK=0002`
(fix de sub-banda, ver 2.2 más abajo) y `RAK3172_Join()` — **todo esto
antes de `GPS_Init()`**. Antes, el GPS se inicializaba primero y sus
~2.5s de `HAL_Delay()` bloqueantes retrasaban el primer intento de join
sin necesidad; ahora el join sale lo antes posible y el GPS se
inicializa después, sin competir por tiempo de arranque.

**2. El propio módulo reintenta el join solo, 8 veces cada 10s**
(`AT+JOIN=1:0:10:8`, sin que el host tenga que hacer nada) antes de
declarar la primera ronda fallida. Un evento
**`+EVT:JOIN_FAILED_RX_TIMEOUT`** aislado es normal (una ventana de
recepción perdida) — no es una falla del sistema mientras algún
intento de esos 8 termine en `+EVT:JOINED`, que es justo lo que se ve
en campo la mayoría de las veces.

**3. Confirmación**: `+EVT:JOINED` pone `RAK3172_EstaUnido()` en
`true`. A partir de ahí se habilita el envío periódico del uplink LIVE
(cada 30s) y la solicitud de hora por red (ver sincronización de hora
más abajo).

**4. Detección de caída a mitad de sesión (agregado 2026-08-29, tras un
caso real en campo).** El primer diseño no tenía ninguna forma de
notarlo: `RAK3172_EstaUnido()` solo se ponía en `false` cuando el
propio host llamaba a `RAK3172_Join()`, así que si el módulo perdía el
join por su cuenta (ej. un reinicio espontáneo — ver hallazgo de
hardware abajo), el firmware seguía creyendo que todo estaba bien para
siempre, mandando uplinks al vacío sin parar. Ahora `rak3172.c`
reconoce la respuesta **`AT_NO_NETWORK_JOINED`** (la que da el módulo
cuando se le pide enviar sin estar unido) como prueba definitiva de que
ya no hay join, y de inmediato (sin esperar los 2s del timeout) pone
`RAK3172_EstaUnido()` en `false` y lo reporta por log.

**5. Reintento automático, sin bloquear el resto del loop.** Con
`RAK3172_EstaUnido()` en `false`, el loop principal dispara una máquina
de 2 pasos no bloqueante: reafirma `AT+MASK=0002` (por si ese comando
había fallado silenciosamente en el arranque original) y, en cuanto
responde, manda `AT+JOIN` de nuevo. Si no se logra unir, este ciclo se
repite cada `INTERVALO_REINTENTO_JOIN_MS` (2 minutos, con margen sobre
los ~80s que tarda el módulo en agotar sus propios 8 intentos) de forma
indefinida, hasta que el join se recupere. Antes de este cambio, un
fallo de join al arrancar (o una caída a mitad de sesión) dejaba al
nodo sin LoRa **para siempre**, sin ningún reintento — este era
justamente el reporte original que motivó todo este trabajo.

**6. Recuperación de errores de UART (`HAL_UART_ErrorCallback`).**
Aparte de la lógica de arriba, se encontró que un solo error de UART
(overrun/framing/ruido — ej. el ruido del cable USB-a-PC ya documentado
en 2.6) dejaba la recepción por DMA de USART1 o USART2 muerta para el
resto de la sesión: HAL aborta la recepción internamente al entrar en
error, y sin un `HAL_UART_ErrorCallback()` que la rearme, el callback
normal de datos nunca vuelve a dispararse — todo comando futuro por esa
UART expira por `TIMEOUT` sin que el host jamás vea la respuesta real,
aunque el módulo del otro lado siga funcionando bien. Se agregaron
`RAK3172_ErrorCallback()`/`GPS_ErrorCallback()` (limpian los flags de
error y rearman la recepción) y un `HAL_UART_ErrorCallback()` en
`main.c` que las despacha según la instancia — mismo patrón que
`HAL_UARTEx_RxEventCallback()`. ⚠️ Actualmente instrumentado con prints
de diagnóstico temporal (`DIAGNOSTICO TEMPORAL` en el código) para
confirmar en campo si este camino llega a dispararse; quitar una vez
confirmado.

**7. Logging de resultados AT: por flanco, no por cambio de valor.**
El reporte `RAK3172: resultado=...` del loop principal originalmente
solo se imprimía cuando el resultado **cambiaba** respecto al último
mostrado. Eso ocultaba fallos reales: si un comando fallaba con
`TIMEOUT` dos veces seguidas, la segunda vez no se imprimía nada (mismo
valor que la anterior), y en el log parecía que el nodo "dejó de
intentar" cuando en realidad seguía intentando y fallando cada vez
igual. Corregido para detectar el **flanco** de "comando recién
terminado" (en curso → listo), que reporta cada finalización real sin
importar si el resultado se repite.

⚠️ **Caso de campo abierto (2026-08-29): reinicios espontáneos del
RAK3172** (banner de arranque reaparece en medio de la sesión) — la
lógica de los puntos 4-5 de arriba maneja esto correctamente (detecta
la caída y reintenta sin quedarse trabado), pero la causa raíz **no es
de software** → ver hardware, sección 4, y pendientes (sección 8).

#### Diagnóstico de reinicios espontáneos del STM32 (agregado 2026-09-23, `main.c`)

Motivado por un caso real en banco: se observó `segundos_transcurridos`
(el contador de "cuánto lleva el motor en el estado actual", ver 4.2)
volver a `0` sin que `nombre_operacion` hubiera cambiado en ningún
uplink — y no había forma de saber, solo con ese dato, si fue (a) un
reset real del STM32 (`estadoInicializado` se pierde, es una variable
en RAM) o (b) un cambio de estado real o falso positivo del tacómetro
que ocurrió y se revirtió entre dos uplinks de 30s, sin que el MCU se
haya reiniciado en absoluto — ambos casos producen el mismo síntoma
visto desde AWS/MQTT. Tres campos nuevos en el log de depuración (línea
de 1Hz, junto a `Frecuencia:`) resuelven la ambigüedad sin depurador
conectado:

1. **`Causa del reset`** — impreso una sola vez, al arranque, leyendo
   las banderas de `RCC->CSR` (`__HAL_RCC_GET_FLAG(RCC_FLAG_...)`)
   antes de limpiarlas con `__HAL_RCC_CLEAR_RESET_FLAGS()` (si no se
   limpian, se acumulan de un reset a otro y el próximo arranque
   muestra causas viejas mezcladas con la nueva). `BOR`+`PIN` juntos =
   ciclo de energía real (arranque en frío); `PIN` solo (sin `BOR`) =
   `NRST` se disparó con la alimentación ya estable (botón físico, o
   ruido/glitch en esa línea — ver el hallazgo de hardware sobre
   ground-bounce). `IWDG`/`WWDG`/`LOW_POWER` no deberían aparecer
   **nunca** en este proyecto (no hay watchdog configurado, no se usan
   modos Stop/Standby) — si aparecen, es un hallazgo importante por sí
   solo, no ruido.
2. **`Uptime`** — `HAL_GetTick()` desde el arranque, ajeno por completo
   al RTC/GPS/hora de red. Solo puede volver a `00:00:00` si el MCU se
   reinició de verdad.
3. **`TiempoOperacion`** (con `estado=` al lado) — el mismo cálculo que
   `segundos_transcurridos` del uplink (`fechaHoraLocalIterActual -
   inicioEstadoLocal`), pero leído cada 1s en vez de solo cuando sale
   un uplink — para poder verlo resetearse EN VIVO sin esperar 30s.

**Cómo leerlos juntos**: si `TiempoOperacion` vuelve a `0` y `Uptime`
**también** vuelve a `0` en la misma línea → reset real del MCU (y
`Causa del reset` va a reaparecer arriba, en el próximo banner de
arranque, con el motivo). Si `TiempoOperacion` vuelve a `0` pero
`Uptime` **sigue corriendo** normal → no fue un reset, fue un cambio de
`estado` (real o glitch del tacómetro) — revisar `TACOMETRO_TIMEOUT_DETENIDO_MS`
(2.1) y la instalación de la señal de entrada en ese caso, no el
firmware de reset.

**Causa raíz confirmada + arreglo (2026-09-23/24)**: usando estos tres
campos, se analizó el log completo de una sesión de banco (~43,800
líneas) con el tacómetro sin conectar (línea al aire, ver 2.1) — cinco
eventos de ruido reales identificados, sin correlación con el TX del
RAK3172 ni con los reportes del GPS (no es el propio nodo
induciéndose el ruido, es ambiente). Dos de esos cinco fueron lo
bastante fuertes como para colar 1-2 pulsos por el filtro de
`tacometro.c` y producir una lectura de RPM físicamente absurda
(2000+ RPM) sostenida solo una fracción de segundo — suficiente para
reiniciar `inicio_operacion` sin que el motor arrancara de verdad.
**Arreglo**: `main.c` ahora exige que el estado instantáneo
(`estadoInstantaneo`) se sostenga `ESTADO_DEBOUNCE_MS` (3s,
`estadoCandidato`/`estadoCandidatoDesdeTick`) antes de confirmarlo
como `estadoActual` — el valor que alimenta la telemetría del uplink
Y el ancla de `inicio_operacion`. Un motor diésel real no puede pasar
de detenido a régimen en menos de 3s, así que esto filtra el glitch
sin retrasar de forma perceptible ninguna transición real. ⚠️
Deliberadamente **no** toca `Tacometro_EstaDetenido()` crudo, que
sigue usándose sin debounce en la lógica de seguridad que fuerza
`CALIB` a `0` en cuanto el motor arranca (sección 4.4) —
esa sí necesita reaccionar de inmediato, sin los 3s de margen.

#### Sincronización de hora: GPS (primaria) + LoRaWAN DeviceTimeReq (respaldo)

El RTC corre del LSI interno (ver 2.5), sin cristal LSE — deriva
**medida en campo (2026-08-26): ~0.84%** (equivale a ~12 min/día sin
corrección). Por eso la hora no se sincroniza una sola vez al
arrancar, sino que se resincroniza durante toda la operación contra
**dos fuentes redundantes**, ambas llamando a `Reloj_SetHoraUtc()`:

1. **GPS** (`GPS_GetFechaHoraUtc()`, primaria): `main.c` la consulta
   cada `INTERVALO_RESYNC_GPS_MS` (30s, igual al envío de RPM) si hay
   fix vivo (`GPS_TieneFix()`). No compite por el canal AT del
   RAK3172 ni gasta duty cycle — el único límite real es que el propio
   SIM7600X solo refresca su reporte `+CGPSINFO:` cada 10s (ver 2.6),
   así que preguntar más seguido no trae dato más fresco.
2. **Red LoRaWAN** (`DeviceTimeReq`, respaldo): antes del primer sync
   (de cualquier fuente) compite libremente con el GPS — el que llegue
   primero gana. Una vez que ya hubo un sync exitoso, se limita a
   activarse solo si pasan `INTERVALO_FALLBACK_LORA_MS` (30 min) sin
   ningún resync nuevo — típicamente porque el GPS no tiene vista al
   cielo por un rato largo. Ver `necesitaFallbackLora` en `main.c`.

El flujo del `DeviceTimeReq` en sí (RUI3: `AT+TIMEREQ`/`AT+LTIME`) no
cambia:

1. `RAK3172_SolicitarHoraRed()` manda `AT+TIMEREQ=1` — a diferencia
   del diseño original, se puede (y se debe) llamar repetidas veces,
   no solo una vez por arranque; cada llamada limpia internamente el
   flag del ciclo anterior (`RAK3172_HoraDeRedDisponible()`) para no
   confundirlo con el evento de un ciclo previo.
2. La hora **no llega de inmediato** — viaja "montada" como MAC
   command en el `FOpts` del próximo uplink exitoso (no es un uplink
   dedicado, sin costo de duty cycle extra). Cuando eso pasa, el
   módulo emite el evento **`+EVT:TIMEREQ_OK`**, detectado en
   `RAK3172_ProcesarLinea()` y expuesto como
   `RAK3172_HoraDeRedDisponible()`.
3. Con la hora ya disponible, se consulta el valor con `AT+LTIME=?`
   (`RAK3172_ConsultarHoraRed()`), y la respuesta se lee por el
   mecanismo genérico `RAK3172_GetUltimaRespuesta()`.
4. `main.c` parsea esa respuesta y llama a `Reloj_SetHoraUtc()`.

Formato de `AT+LTIME=?` **confirmado en campo** (2026-08-17): texto
legible, no un epoch plano —
`"AT+LTIME=15h08m55s on 08/17/2026"` (`MM/DD/YYYY`), parseado con
`sscanf(respuesta, "AT+LTIME=%2uh%2um%2us on %2u/%2u/%4u", ...)` en
`main.c`.

⚠️ El uplink LIVE (`RAK3172_EnviarUplinkLive()`) **NO espera** a que el
reloj sincronice — sale desde el primer ciclo con la mejor hora
disponible (la persistida en flash de un arranque anterior, o el
default de `MX_RTC_Init()` si es el primerísimo arranque). Esto evita
el ciclo circular "sin uplink no hay hora, sin hora no hay uplink" —
ver `CalibFlash_GetUltimaHoraUtcConocida()`/`Reloj_CargarHoraAproximada()`
en 2.5.

**Preservación del contador de estado al (re)sincronizar** (bug
corregido 2026-08-18, generalizado a todo resync 2026-08-26): cada vez
que el reloj se corrige — el primer sync desde el placeholder de
flash, o cualquier resync periódico posterior, por GPS o por red — el
código en `main.c` **desplaza** el ancla de `inicio_operacion`
(`inicioEstadoLocal`) por el mismo salto (bandera
`relojFueCorregidoEsteCiclo`), en vez de reiniciarla a "ahora". Si no,
cada corrección (incluido un resync rutinario cada 30s por GPS)
borraría el tiempo ya transcurrido en el estado actual, aunque el
motor llevara horas sin cambiar de estado.

⚠️ **No confundir latencia de pipeline con imprecisión del RTC**
(observado en campo, 2026-08-26): comparar `fecha_hora` del payload
(la pone el dispositivo al armar el uplink) contra la marca de tiempo
de recepción en AWS/MQTT casi siempre muestra un desfase de varios
segundos — eso es el tiempo real que tarda el mensaje en viajar
LoRaWAN → gateway → network server → Lambda decoder → MQTT, **no**
error del reloj. La señal de que sí es el RTC es que ese desfase
**crezca** con el tiempo entre mensajes sucesivos; si se mantiene
plano (aunque no sea exactamente 0s), el reloj está bien.

### 2.3 `calibracion_flash.c/h`

Almacena los parámetros configurables en la **última página de flash**
del STM32G431KB (2KB, dirección `0x0801F800`, página 63), y centraliza
la lógica de aplicar cualquier downlink de configuración.

**⚠️ Regla crítica**: cada vez que se agregue, quite o reordene un
campo de `CalibFlash_Datos_t`, hay que **subir `CALIB_FLASH_MAGIC`**.
Si no se hace, `CalibFlash_Init()` copia bytes de flash borrada (0xFF)
hacia los campos nuevos, dando valores basura (típicamente 65535) —
esto ya causó un bug real de servo atascado en el máximo. Magic actual:
`"CALE"` (subido desde `"CALD"` al agregar
`ultimaLatitudConocida`/`ultimaLongitudConocida`, ver 2.6).

**Punto de entrada único**: `CalibFlash_ProcesarParametroConEstado()`
recibe `(id, datos, longitud, motorOperando)`, valida, aplica (o
rechaza), persiste si corresponde, y devuelve el `STATUS` + el valor
que quedó realmente vigente (en 2 bytes, listo para el ACK).

**Categorías de parámetro** (`CalibFlash_CategoriaDe()`), determinan si
se puede cambiar con el motor operando:

| Categoría | Se permite con el motor operando | Parámetros |
|---|---|---|
| CALIBRACION | Sí, siempre por categoría; candado propio adentro del `case` si hace falta uno más fino | `SET_RATIO`, `ALPHA`, `PID_RPM_KP`, `PID_RPM_KI`, `CALIB` (este último rechaza `1`/`2` con el motor operando desde adentro del `case`, no por la categoría — ver ID 13 más abajo) |
| PROCESO | Sí, siempre (es su función) | `SET_RPM`, `PRESION_REMOTO` |
| COMANDO | Evaluado aparte | `FORZAR_REPORTE`, `RESTAURAR_DEFAULTS`, `RESET_REMOTO` |
| CONFIGURACION | **No** — se rechaza | Todos los demás (default conservador) |

### 2.4 `servo.c/h`

Control PWM del servo del acelerador vía **TIM3_CH3 (PB0)**, 50Hz
(20ms), resolución de 1µs por tick (Prescaler=169, Period=19999).

`Servo_SetPulsoUs(microsegundos)` recorta el valor contra
`SERVO_PULSO_MIN/MAX` antes de aplicarlo — nunca se manda un pulso
fuera de esos límites, sin importar qué tan fuera de rango venga la
solicitud (ni del downlink, ni de un futuro PID).

`Servo_MoverHacia(destinoUs)` mueve el servo gradualmente hacia
`destinoUs` sin bloquear, limitando la velocidad a
`SERVO_VELOCIDAD_MAX_US_S` (µs/segundo, constante fija en `servo.h`,
no configurable remoto — protección mecánica contra saltos bruscos
del pulso, ej. ante una corrección fuerte del futuro PID). Calcula el
paso por tiempo real transcurrido (`HAL_GetTick()`), no por conteo de
llamadas, así que no depende de la cadencia exacta del loop principal.

**Modo calibración** (`main.c`): con el motor detenido, un downlink
`CALIB=2` activa un barrido continuo del servo entre
`SERVO_PULSO_MIN` y `SERVO_PULSO_MAX` (vía `Servo_MoverHacia()`) — para
observar en banco el recorrido físico real mientras se ajustan esos
dos límites por downlink, con efecto inmediato en el barrido en curso
(se leen en cada vuelta del loop, no se cachean). Al llegar a cada
extremo se hace una pausa de `SERVO_BARRIDO_PAUSA_EXTREMOS_MS` (1000ms,
`servo.h` — fija, no configurable por downlink; ajustar y reflashear si
hace falta) antes de invertir la dirección — el pulso PWM comandado
llega exacto al límite en cuanto se cumple `SERVO_VELOCIDAD_MAX_US_S`,
pero el servo físico tiene su propio tiempo de asentamiento; sin esa
pausa, el firmware invertía la marcha en la misma vuelta del loop en
que el pulso llegaba al extremo, y el brazo real nunca alcanzaba a
terminar de llegar al tope mecánico antes de que se le pidiera ir al
otro lado (los modos `0`/`1` no tienen este problema porque el destino
se queda fijo indefinidamente, dándole tiempo de sobra al servo).
`CALIB=1`
es el modo manual: el servo se queda quieto en su posición actual (no
salta a ningún límite al entrar) y solo se mueve cuando llega un
downlink de `SERVO_PULSO_MIN` o `SERVO_PULSO_MAX`, yendo directo a ese
valor — útil para verificar un punto exacto del recorrido sin esperar
una vuelta completa del barrido. El "llegó un downlink" se marca con
una bandera en el instante en que `calibracion_flash.c` aplica el
parámetro con éxito
(`CalibFlash_HayObjetivoManualServo()`/`GetObjetivoManualServoUs()`/
`LimpiarObjetivoManualServo()`), **no** comparando contra el valor
anterior — si se comparara, un downlink que repite el valor ya guardado
(ej. los defaults de fábrica, `SERVO_PULSO_MIN=1000`/`MAX=2000`) nunca
se vería como "cambio" y el servo no se movería, aunque el downlink sí
haya llegado. Ambos (`1` y `2`) son el único
momento en que `SERVO_PULSO_MIN/MAX` se pueden cambiar (fuera de modo
calibración, se rechazan con `APPLY_ERROR` aunque el motor esté
apagado). Medida de seguridad: si el motor arranca mientras
`CALIB` es `1` o `2`, el firmware lo fuerza a `0`
localmente en el mismo ciclo (vía `Tacometro_EstaDetenido()`), sin
esperar ningún downlink — nunca se deja el servo bajo control remoto de
calibración con el motor operando.

**Log `SERVO_CAL`**: mientras `CALIB` sea `1` o `2`, cada
1s se imprime `SERVO_CAL,min=<us>,max=<us>,pulso_actual=<us>` — antes
de esto, la única forma de ver los límites vigentes en banco era el ACK
de cada downlink de `SERVO_PULSO_MIN/MAX` (`valor vigente=X`); esta
línea muestra en vivo, mientras se observa el servo moverse, dónde
están los límites configurados y qué pulso se le está aplicando en ese
instante. Fuera de esos dos modos no imprime nada (a diferencia de
`PID_TEST`, sección 9, esto es solo para lectura humana en banco, no
pensado para analizarse después con un script).

Este bloque de barrido queda como el mecanismo permanente para
recalibrar `SERVO_PULSO_MIN/MAX` en banco (ej. si se cambia de servo o
de geometría de la varilla) — no se retira al existir el PID real
(`pid.c/h`, ver abajo), simplemente son mutuamente excluyentes: uno
corre en `CALIB=2`, el otro con el motor operando y
`CALIB=0` (ver 4.4).

**Posición segura = `SERVO_PULSO_MIN`** (confirmado en el montaje
físico real: ese extremo deja el acelerador sin aceleración/ralentí,
no es un valor arbitrario). Con `CALIB=0` y el motor
detenido (arranque, o justo después del apagado de seguridad de
arriba), el servo siempre va o se mantiene ahí — al arrancar se manda
de una vez (no se conoce la posición previa, quedó como estuviera al
perder alimentación); al salir del barrido por el apagado de
seguridad, regresa gradualmente vía `Servo_MoverHacia()` (respetando
`SERVO_VELOCIDAD_MAX_US_S`, no de un salto), sin importar en qué punto
del recorrido se haya quedado. Si en cambio el motor está operando
(y no se está calibrando), el servo lo maneja el PID (ver abajo), no
esta posición fija.

**PID (`pid.c/h`)**: lazo de control real, activo en `main.c` solo
cuando el motor opera, `CALIB=0` (no se está
calibrando el servo -- desde 2026-09-29 la sesión de sintonización del
PID ya no es un valor `CALIB` dedicado, corre dentro del propio `=0`,
ver 4.4)
**y** se comandó explícitamente un `SET_RPM` por encima de `RPM_MIN`
— mutuamente excluyente con el barrido de banco por diseño (ver 4.4).
`PID_CalcularSalidaUs(setpointRpm, rpmMedida)` devuelve el pulso en µs
directos que se le pasa a `Servo_MoverHacia()`, usando
`SERVO_PULSO_MIN` como línea base (la corrección del PID se suma sobre
ella, no la reemplaza) y recortando contra `SERVO_PULSO_MIN/MAX`.
Anti-windup por integración condicional (la integral se congela si ya
está saturando en la misma dirección del error) y se resetea sola
mientras `PID_RPM_KI=0`, para que no acumule en silencio y aparezca de
golpe cuando se active esa ganancia. `PID_Init()` se llama una sola vez
en el flanco de entrada a este modo, para que el primer cálculo no use
un `dt` inflado por el tiempo inactivo.

**`RPM_MIN` es el umbral de "¿debe intervenir el control?", no solo un
límite duro** — `RPM_MIN` ya está documentado como "límite duro /
ralentí" (tabla de parámetros), y el ralentí real es distinto en cada
motor. `MODO=0`/Ralentí (el motor sube solo de 0Hz a su ralentí
natural, sección 4.3) es deliberadamente **sin control**: el setpoint
que produce ese modo es siempre `0`, por debajo de `RPM_MIN`, así que
el PID no corre y el servo se queda en `SERVO_PULSO_MIN` — nunca se
recorta ningún setpoint hacia arriba hasta `RPM_MIN` como si fuera
válido. El lazo solo se activa cuando el setpoint elegido según `MODO`
(sección 4.3: `SET_RPM` en `MODO=1`/Local, fórmula presión→RPM en
`MODO=2`/Remoto) supera `RPM_MIN`; una vez activo, se recorta hacia
abajo contra `RPM_MAX` si se pide algo más alto.

El setpoint sale de `CalibFlash_GetSetRpm()` (`MODO=1`) o de la fórmula
lineal presión→RPM (`MODO=2`) según corresponda -- ver la máquina
`MODO` completa en sección 4.3. Ganancias (`PID_RPM_KP/KI/KD`) siguen en
sus defaults (`Kp=1.0, Ki=0,
Kd=0`) a la espera de sintonización con el motor real (Ziegler-Nichols
en lazo cerrado, ver sección 9; solo se pueden tocar con
`MODO=CALIBRACIÓN` y `CALIB=0`, ver sección 4.4) — suficientes para validar que el lazo mueve
el servo en la dirección correcta, no para un control ya afinado.

**Alimentación del servo** → ver hardware, sección 2.

### 2.5 `rtc_reloj.c/h`

Reloj interno del STM32G431 sobre el periférico **RTC**, con **LSI**
(~32kHz nominal, confirmado en el datasheet del G431 y en el diagrama
de árbol de reloj de CubeMX) como fuente — **no hay cristal LSE
poblado** en este Nucleo-32, ni pila de respaldo (VBAT) dedicada.

**Consecuencia práctica**: la hora se pierde en cada corte real de
energía (no en un `NVIC_SystemReset()` mientras VDD no se interrumpa)
— por eso el firmware **busca resincronizarse (por GPS o por red
LoRaWAN, ver 2.2) en cada arranque**, en vez de asumir que el RTC
"recuerda" la hora entre encendidos. Además, al no tener cristal LSE,
el LSI deriva con el tiempo (**~0.84% medido en campo**, ver 2.2) — por
eso la resincronización tampoco es única al arrancar, sino periódica
durante toda la operación.

- El RTC guarda la hora en **UTC** (la misma que entrega
  `DeviceTimeReq`) — el offset de **-6h de Guatemala se aplica solo al
  leer la hora para el payload de salida** (`Reloj_GetUnixTimeLocal()`),
  nunca se le resta al RTC en sí.
- Conversión epoch↔calendario implementada **a mano** (sin
  `time.h`/`gmtime` de la libc), para no depender de que la
  newlib-nano del proyecto tenga esas funciones enlazadas. Verificada
  contra `datetime` de Python en varios casos, incluyendo bordes de fin
  de año.
- **Prescalers del RTC**: `AsynchPrediv=99` / `SynchPrediv=319`
  (`100 × 320 = 32000`), calculados para el LSI de 32kHz nominal del
  G431 — **no usar el default de CubeMX (127/255, pensado para un LSE
  de 32.768kHz)**, da un error de base de ~2.3% además de la
  imprecisión propia del LSI. Configurado directamente en el `.ioc`
  (panel RTC), no hardcodeado en código.
- `Reloj_EstaSincronizado()` es `false` hasta el primer
  `Reloj_SetUnixTimeUtc()`/`Reloj_SetHoraUtc()` exitoso (por GPS o por
  red, ver 2.2) — el uplink LIVE **no** espera este flag para empezar a
  mandarse (ver 2.2), solo lo usa para saber si vale la pena reportar
  `RelojSync=1` en el log de depuración.

### 2.6 `gps.c/h`

Posición GPS/GNSS del módulo **SIM7600X (Waveshare 4G HAT)** sobre
USART2, DMA en modo Circular + detección de línea inactiva (IDLE) —
mismo patrón de recepción que `rak3172.c`, pero para un flujo periódico
en vez de comando/respuesta.

⚠️ **Cambio de diseño importante (confirmado en campo, 2026-08-18)**:
este módulo **NO transmite NMEA crudo espontáneamente** por esta UART
(probado a mano vía USB-TTL directo). Lo que sí soporta, documentado en
el *AT Command Manual* de SIMCom, es `AT+CGPSINFO=<1-255>`: configurado
una sola vez en `main.c`, el módulo empieza a mandar una línea
`+CGPSINFO: ...` por su cuenta cada N segundos (URC, igual que los
`+EVT:` del RAK3172) — no es necesario volver a pedirla.

Formato de `+CGPSINFO:` (sin checksum, a diferencia de NMEA):
```
+CGPSINFO: <lat>,<N/S>,<lon>,<E/W>,<fecha ddmmyy>,<hora hhmmss.s>,<alt>,<vel>,<rumbo>
```
Sin fix, todos los campos vienen vacíos: `+CGPSINFO: ,,,,,,,,`.

**Secuencia de inicio** (`main.c`): `GPS_Init(&huart2)` arranca la
recepción, una pausa de 500ms, `AT+CGPS=1` (enciende el motor GNSS;
responde `ERROR` —inofensivo— si el módulo ya estaba encendido de una
sesión previa), una pausa de 1.5s, y por último `AT+CGPSINFO=10`
(habilita el auto-reporte cada 10s). Simple, sin lógica adicional —
el bug de abajo **no era esto**.

✅ **RESUELTO (2026-08-25) — causa raíz real: el cable USB del Nucleo
a la PC, no el firmware ni el módulo** → ver hardware, sección 6.2 para
el análisis completo (prueba A/B, hipótesis del mecanismo, y las vías
descartadas antes de encontrar la causa real).

**Prioridad de posición para el uplink LIVE** (`main.c`): GPS con fix
vivo (`GPS_TieneFix()`) → última posición conocida persistida en flash
(`CalibFlash_GetUltimaLatitudConocida()`/`...Longitud...()`, se guarda
una sola vez por arranque, la primera vez que hay fix — no en cada
reporte, para no desgastar la flash) → `LATITUD_FIJA`/`LONGITUD_FIJA`
como último respaldo si el GPS nunca ha conseguido fix (ni en este
arranque ni en ninguno anterior).

**Disciplina del RTC**: `GPS_GetFechaHoraUtc()` expone la fecha/hora
UTC del último `+CGPSINFO:` con fix (campos `fecha ddmmyy`/`hora
hhmmss.s` del formato de arriba — antes descartados, solo se usaba
lat/lon). Es la fuente **primaria** de sincronización del RTC, con la
red LoRaWAN como respaldo — ver sección 2.2 para el mecanismo completo.

⚠️ **Hallazgos de hardware del Waveshare SIM7600X-H 4G HAT** (jumpers,
alimentación, antena) → ver hardware, sección 6.1.

**Bug corregido (2026-08-18)**: el ciclo de consumo del buffer circular
en `GPS_RxEventCallback()` se colgaba indefinidamente cuando el DMA
completaba una vuelta exacta del buffer (HAL reporta `Size==tamaño del
buffer` en vez de `0` en ese caso) — congelaba el `while(1)` completo
del `main()` (incluida la impresión de `Frecuencia`), no solo al GPS.
Corregido acotando `Size` a `0` cuando llega igual al tamaño del
buffer.

**Reintento automático tras reportes vacíos seguidos**: si llegan
`GPS_REPORTES_VACIOS_ANTES_DE_REENVIAR` (10, `gps.h`) reportes
`+CGPSINFO: ,,,,,,,,` **seguidos** (~100s sin fix a razón de uno cada
10s), `GPS_ProcesarCGPSInfo()` reenvía `AT+CGPS=1` por su cuenta y
reinicia el contador. Es una red de seguridad barata, no una solución
al bug de arriba (ese ya se resolvió y no era del módulo) — si el GPS
está sano y solo tarda en conseguir fix (mal clima, sin vista al
cielo), el reenvío es inofensivo: con el GPS ya encendido, `AT+CGPS=1`
solo contesta `ERROR` sin interrumpir la adquisición en curso. El
contador se reinicia también en cuanto llega un fix real.

### 2.7 `comando_serial.c/h` — mando manual TEMPORAL por serial

Mientras no hay red LoRa disponible en campo para probar downlinks
reales, este módulo permite escribir a mano un "downlink" por el mismo
puerto de debug (LPUART1) que ya se usa para ver los logs. Recibe **por
interrupción** (`HAL_UART_Receive_IT`, un byte a la vez, en un ring
buffer — requiere `LPUART1 global interrupt` habilitada en el `.ioc`,
NVIC), arma la línea con eco local, y al recibir Enter la pasa por
`CalibFlash_ProcesarParametroConEstado()` — **el mismo punto
de entrada que usa `rak3172.c` para un downlink real** (misma
validación de rango, mismo bloqueo por categoría con el motor
operando, misma persistencia en flash). La única diferencia real es el
transporte: no manda el Application ACK por LoRaWAN
(`RAK3172_EnviarAck`), el resultado se imprime directo al mismo
puerto, como si fuera el ACK.

⚠️ **Cambiado de sondeo a interrupción el 2026-09-18**: la versión
original sondeaba la bandera `RXNE` una vez por vuelta del loop
principal (sin IT/DMA) — funcionaba bien en general, pero se confirmó
en campo que perdía bytes en silencio si dos caracteres llegaban
justo cuando el loop tardaba más de lo normal en volver a sondear (ej.
justo después de `CALIB`, que dispara una escritura real
a flash). Síntoma: comandos con un carácter faltante a mitad de la
palabra (`ALIB`, `CALIB2` sin el espacio) —
la firma de un byte pisado en el registro de datos antes de leerse, no
un error de tipeo. Ahora cada byte se captura de inmediato en
`HAL_UART_RxCpltCallback()`, inmune a qué tan ocupado esté el loop.

Formato de línea: `NOMBRE_PARAMETRO [VALOR]`, con el mismo nombre y el
mismo valor "humano" (sin escalar) que se manda por MQTT vía
`RIO-DSL-SendDownlink` (ver sección 6) — ej. `CALIB 1`,
`SET_RPM 900`, `SERVO_PULSO_MIN 1050`. Los comandos
(`RESTAURAR_DEFAULTS`, `FORZAR_REPORTE`, `RESET_REMOTO`) no llevan
VALOR, el módulo manda el byte de confirmación `0xA5` automáticamente.

#### Pass-through AT hacia el RAK3172 (permanente)

Además de los parámetros, la consola acepta consultas `AT+XXX=?` y las
reenvía al RAK3172 por `RAK3172_EnviarComandoAT()`. Sirve para leer
`AT+APPKEY=?`, `AT+DEVEUI=?`, `AT+APPEUI=?`, `AT+VER=?`, etc. sin cablear
un USB-serial directo al módulo. También permite una lista corta de
escrituras de provisionamiento: `AT+NJM=<v>`, `AT+BAND=<v>`,
`AT+MASK=<v>`, `AT+CLASS=<v>` y `ATZ` (reset del RAK). Salida:

```
[AT] <valor devuelto por el RAK>
[AT] resultado=0 (0=OK,1=ERROR,2=TIMEOUT,3=BUSY)
```

- **Lista blanca**: toda línea que empiece con `AT` y no sea consulta
  `=?`, ni una de las 4 escrituras / `ATZ` de arriba, se rechaza
  (`[AT] no permitido...`) — `AT+FACTORY`, `AT+NWM`, `AT+BOOT`, `AT+JOIN`,
  `AT+APPKEY=<v>` etc. no pasan. Para permitir otra escritura, agregarla a
  `AT_ESCRITURAS_PERMITIDAS` en `comando_serial.c`.
- **Configuración LoRaWAN del RAK3172 aplicada por consola (2026-09-29)**,
  en este orden, cada uno con `[AT] resultado=0` y verificado con su
  consulta `=?`:
  ```
  AT+NJM=1      (join OTAA)
  AT+BAND=6     (banda 6 = AU915 en la tabla de RUI3)
  AT+MASK=0002  (sub-banda; el firmware también lo reafirma en cada arranque)
  AT+CLASS=C
  ATZ           (reset del RAK; puede dar resultado=2 esa vez, es normal)
  ```
  Estos valores viven en la memoria del RAK: un nodo nuevo o un RAK
  reemplazado necesita repetirlos, además de `APPEUI/APPKEY/DEVEUI`
  (provisionados aparte).
- La línea se transmite desde `ComandoSerial_Update()` (loop principal),
  no desde la ISR de recepción.
- Comparte el canal AT con los uplinks/ACKs: una consulta lo ocupa hasta
  3 s (`PASSTHROUGH_TIMEOUT_MS`). Si el RAK está ocupado, responde
  `[AT] ocupado` y se repite.
- `resultado=2` (TIMEOUT) sin línea `RAK3172 RX crudo` = el RAK no
  contestó nada: apagar y encender toda la placa (el NRST solo reinicia
  el STM32, no el RAK) y probar primero `AT+VER=?`.
- Exposición: permite leer las claves LoRaWAN, pero requiere acceso
  físico al puerto de debug, desde el cual ya se pueden leer conectándose
  al RAK.

Los comandos de parámetros de este módulo son un mando temporal — quitar su llamada en `main.c`
(`ComandoSerial_Init()`/`ComandoSerial_Update()`) cuando exista un
mando local real (pantalla/botonera) o ya no se necesite probar sin
red LoRa.

### Formato del downlink (FPort 2)

```
[PARAMETER_ID: 1 byte][VALUE_H: 1 byte][VALUE_L: 1 byte]   (3 bytes)
```

### Formato del Application ACK (FPort 3)

```
[PARAMETER_ID: 1 byte][STATUS: 1 byte][VALUE_H: 1 byte][VALUE_L: 1 byte]   (4 bytes)
```

`VALUE` en el ACK es el valor que **realmente quedó vigente** — si el
downlink fue rechazado, es el valor anterior, no el solicitado.

### Códigos de STATUS

| Código | Nombre | Significado |
|---|---|---|
| 0 | `OK` | Se aplicó correctamente |
| 1 | `OUT_OF_RANGE` | Valor fuera del rango válido |
| 2 | `UNKNOWN_PARAMETER_ID` | ID no reconocido, o longitud de datos insuficiente |
| 3 | `STORAGE_ERROR` | Falló la escritura en flash (hardware) |
| 4 | `APPLY_ERROR` | Precondición no cumplida (ej. `SERVO_PULSO_MIN/MAX` con `CALIB=0`) |
| 5 | `REJECTED_ENGINE_RUNNING` | Parámetro de categoría CONFIGURACION, motor operando |
| 6 | `REJECTED_ENGINE_STOPPED` | `MODO=1`/`2` (control automático) con el motor detenido (agregado 2026-09-22, sección 4.3) |
| 7 | `REJECTED_OBJETIVO_REMOTO_NO_CONFIGURADO` | `MODO=2` con `PRESION_OBJETIVO_REMOTO` en `0` (sin configurar) — agregado 2026-09-23, sección 4.3. No es un candado de seguridad (a diferencia del `6`), es para evitar que `MODO=2` quede "andando" sin hacer nada útil, en silencio |

### Tabla completa de parámetros

| ID | Nombre | Escala | Ancho | Categoría | Notas |
|---|---|---|---|---|---|
| 1 | `SET_RATIO` | x100 | 2B uint16 | Calibración | Pulsos/revolución |
| 2 | `ALPHA` | x1000 | 2B uint16 | Calibración | Coef. filtro EMA (0-1) |
| 3 | `SET_RPM` | x10 | 2B uint16 | Proceso | Setpoint directo -- solo tiene efecto con `MODO=3` (`MANUAL_BANCO`, sección 4.3); en cualquier otro `MODO` se rechaza con `APPLY_ERROR` (agregado 2026-09-17, antes se aceptaba en silencio sin hacer nada si `MODO` no era el correcto) |
| 4 | `RPM_MAX` | x10 | 2B uint16 | Configuración | Límite duro superior -- techo MECÁNICO del motor/motobomba, fijo por modelo (no depende de la instalación). Exige `MODO=CALIBRACIÓN` además de motor detenido, ver sección 4.4 |
| 5 | `RPM_MIN` | x10 | 2B uint16 | Configuración | Límite duro / ralentí |
| 32 | `RPM_MAX_CARGA` | x10 | 2B uint16 | Configuración | Agregado 2026-09-28 -- techo CON CARGA (hidráulico, ej. no reventar tubería), específico de cada instalación, SIEMPRE `≤ RPM_MAX` (rechazado con `OUT_OF_RANGE` si se intenta poner más alto). Mismo candado que `RPM_MAX` (`MODO=CALIBRACIÓN` + motor detenido). Ver sección 4.4 para dónde se usa cada uno (embrague conectado/desconectado) |
| 34 | `PRESION_MAX` | x10 | 2B uint16 | Configuración | Agregado 2026-09-29 -- guarda dura de presión LOCAL, análoga a `RPM_MAX_CARGA` (límite de la instalación, no del motor). Si la presión medida la supera, el setpoint se fuerza a ralentí y se emite la alerta `5` (`PRESION_MAX_EXCEDIDA`); se libera al bajar al 90 %. Aplica solo con la bomba cargada (`MODO=1/2/3` y `CALIB=9/10/11/12`); `MODO=0` queda exento. Rango `(0, 500]` PSI, default 100. Invariante: `PRESION_OBJETIVO_LOCAL` y `SET_PRESION` deben ser siempre `< PRESION_MAX` (`OUT_OF_RANGE` si no), y `PRESION_MAX` no puede fijarse en o por debajo de ellos. La presión comparada es la que lee el sensor local. Mismo candado que `RPM_MAX` (`MODO=CALIBRACIÓN` + motor detenido). Sube el magic de flash a `CALO` (el próximo reflash borra la calibración de campo). ⚠️ Agregar la alerta 5 a `NOMBRES_ALERTA` en `decoder.py` (AWS) |
| 6 | `PID_RPM_KP` | x100 | 2B int16 | Calibración | Con signo. ⚠️ Gate actualizado 2026-09-29 (`CALIB=10` ELIMINADO, ver sección 4.4): solo se acepta con `MODO=CALIBRACIÓN` Y `CALIB=0` (ningún `CALIB` automático activo) — `APPLY_ERROR` en cualquier otro caso, incluida la operación normal. Se permite con el motor operando (necesario para sintonizar en lazo cerrado, ver sección 9) |
| 7 | `PID_RPM_KI` | x1000 | 2B int16 | Calibración | Con signo. Igual que `PID_RPM_KP` |
| 8 | `TASA_LLENADO_PSI_S` | x100 | 2B uint16 | Proceso | Tasa de subida (PSI/s) de la rampa de llenado de `MODO=1` (sección 4.3). `0` = sin rampa, directo al objetivo. ID reusado 2026-09-23 -- era `PID_KD` (eliminado, ver sección 9: `Kd=0` confirmado en campo, sin uso real, se quitó el término derivativo del lazo interno en vez de dejarlo calibrado en 0). NO aplica durante `CALIB=10` (agregado 2026-09-28) -- ese escalón necesita un salto limpio para la identificación, no una rampa, ver sección 4.4 |
| 33 | `TIEMPO_LLENADO_S` | directo (s) | 2B uint16 | Proceso | Agregado 2026-09-28. Conveniencia sobre `TASA_LLENADO_PSI_S` -- el operador manda cuántos SEGUNDOS quiere que dure el llenado completo (en vez de calcular una velocidad a mano), el firmware lee `PresionV_GetPresionPsi()` en vivo y `PRESION_OBJETIVO_LOCAL` ya configurado, calcula `tasa = (objetivo - presión_actual) / segundos`, y la guarda en `TASA_LLENADO_PSI_S` -- mismo patrón que `SET_RATIO_AUTO` (ID 25). Rechaza `OUT_OF_RANGE` si `segundos=0` o si la presión actual ya está en o por encima del objetivo. El "valor vigente" que devuelve el ACK es la tasa resultante (x100, no los segundos enviados). A diferencia de `SET_RATIO_AUTO`, SÍ persiste el valor crudo en segundos (campo `tiempoLlenadoS` en flash, default `1800`=30min) -- `CALIB=9` (ver sección 4.4/9) lee este valor en segundos directo: calcula su propio ritmo máximo seguro `(PRESION_OBJETIVO_LOCAL − presión base medida) / TIEMPO_LLENADO_S` con la base que mide al arrancar, y usa `1.5 × TIEMPO_LLENADO_S` como timeout de último recurso |
| 9 | `SERVO_PULSO_MIN` | directo (µs) | 2B uint16 | Configuración | Límite mecánico. Además requiere `CALIB=1` o `=2` (modo calibración del servo, NO `=10`) — si no, `APPLY_ERROR` aunque el motor esté apagado |
| 10 | `SERVO_PULSO_MAX` | directo (µs) | 2B uint16 | Configuración | Igual que `SERVO_PULSO_MIN` |
| 11 | `TIMEOUT_SIN_COMANDO_S` | directo (s) | 2B uint16 | Configuración | 60-3600s |
| 12 | `TASA_MAX_CAMBIO_RPM_S` | x10 | 2B uint16 | Configuración | Rampa normal |
| 13 | `CALIB` | 0-12 (numeración vigente desde 2026-09-29, sin huecos) | 1B | Calibración* | ⚠️ Esta celda quedó desactualizada tras `=11`/`=12`/`=13` y el gateo por `MODO=4` (CALIBRACIÓN) agregados 2026-09-24/25/28, Y tras la RENUMERACIÓN completa del 2026-09-29 (eliminó el viejo `CALIB=10` de sintonización manual y corrió todos los demás valores) — ver sección 4.4 para la lista completa y vigente de valores. Todos los números de valor mencionados en el resto de esta celda usan la numeración ANTERIOR a 2026-09-29 (histórica). Todo lo que sigue en esta celda que mencione `=10` como "sintonización de PID" está OBSOLETO: ese valor ya no existe, y `PID_RPM_KP/KI`/`PID_PSI_KP/KI`/`PID_ASP_KP/KI` ahora se aceptan con `MODO=CALIBRACIÓN` + `CALIB=0` (ningún `CALIB` activo), no con un valor `=10` dedicado. Enable/disable lazo de control + **modos de calibración**. Renumerado 2026-09-23 DOS VECES el mismo día (primero al agregarse los auto-escalón de presión, después al reordenarse la secuencia de puesta en marcha, ver sección 12) — todos los números de esta fila son los FINALES. Tres grupos, por seguridad: **apagado** (`1`/`2`, calibración de servo — mueven el servo directo, sin ninguna realimentación de RPM, así que exigen el motor detenido para entrar, con rechazo explícito `REJECTED_ENGINE_RUNNING` si no), **encendido** (`0`/`3`/`6`/`7`/`8`/`9`/`10` — no cambian cómo se maneja el servo respecto a la operación normal, así que no exigen detener el motor para entrar; de hecho `3`/`6`/`7`/`8`/`9`/`10` solo tienen sentido con el motor ya operando) y `4`/`5` (calibración de zona muerta / `SERVO_PULSO_MIN` y mapeo de curva de ganancia — mueven el servo directo como `1`/`2`, pero SÍ miran la RPM en cada paso, así que tampoco exigen el motor operando para entrar, simplemente esperan). Por eso este parámetro ya NO cae en la categoría Configuración genérica (que bloquearía *cualquier* valor con el motor andando) — tiene su propio candado adentro del `case`, igual que `PID_RPM_KP/KI`. Valores: `0` desactivado/operación normal (que además ES el "ralentí" cuando `SET_RPM` no supera `RPM_MIN` — no hay un modo ralentí aparte, sería redundante), `1` manual -- el servo se mantiene quieto en su posición, y cada downlink de `SERVO_PULSO_MIN` o `SERVO_PULSO_MAX` lo mueve directo a ese valor, `2` barrido automático continuo entre `SERVO_PULSO_MIN/MAX`, `3` **calibración de `ALPHA`** (mide el ruido real de RPM y calcula el filtro) -- ver sección 12, `4` **calibración de zona muerta / `SERVO_PULSO_MIN`** (umbral de aceleración; renumerado 2026-09-23, antes era el valor `5` -- se corrió antes del ralentí porque es la línea base de la propia fórmula de salida del PID interno) -- ver sección 11, `5` **mapeo de curva de ganancia** (barre el pulso en lazo abierto desde `SERVO_PULSO_MIN` hasta una RPM techo fija, logueando pulso vs RPM real en cada paso -- por ahora solo mide/loguea, no aplica ninguna corrección todavía; renumerado 2026-09-23, antes era el valor `6` -- mismo motivo que `4`, alimenta la sintonización del PID interno) -- ver sección 12, `6` **auto-escalón del lazo INTERNO de RPM** (PID#1, agregado 2026-09-23 en la segunda renumeración) -- automatiza mandar un escalón fijo de `SET_RPM` (`RPM_MIN` + 200) y loguear `PID_TEST` durante una ventana fija de 60s, mismo espíritu que `8`/`9` pero para el lazo interno, ver sección 4.4/9/12, `7` **calibración de ralentí** (renumerado 2026-09-23, antes era el valor `4` -- se corrió después de `4`/`5`/`6` porque el ralentí real cambia según haya o no carga hidráulica conectada, a diferencia de esos tres) -- ver sección 10, `8` **auto-escalón del lazo de presión LOCAL** (`MODO=1`, renumerado 2026-09-23, antes era el valor `7`) -- automatiza mandar un escalón fijo de `PRESION_OBJETIVO_LOCAL` y loguear `PRESION_PID_TEST` durante una ventana fija, ver sección 9/12, `9` **auto-escalón del lazo REMOTO** (`MODO=2`, probado en `MODO=3`, renumerado 2026-09-23, antes era el valor `8`) -- mismo espíritu que `8` pero con un escalón de `SET_RPM` y logueando `REMOTO_PID_TEST`, ver sección 9/12, `10` **sintonización de PID** (renumerado 2026-09-23 dos veces -- primero de `7` a `9`, después de `9` a `10`) -- el servo lo maneja el PID normal exactamente igual que en `0`, pero es el único modo en que `PID_RPM_KP/KI` se aceptan (y activa el log `PID_TEST`, ver sección 9); `3`, `4`, `5`, `6`, `7`, `8` y `9` solo se pueden pedir viniendo de modo `0` (rechazado con `OUT_OF_RANGE` si se piden desde `1`/`2`/`10`, o entre sí) -- `10` es el único modo de calibración sin esa restricción, se puede pedir directo. En `1`/`2`/`3`/`6`/`7`/`8`/`9`/`10` el servo nunca se comporta distinto a "sin control activo" salvo que `pidActivoAhora` esté armado (solo en `0`/`6`/`8`/`9`/`10` con un `SET_RPM` real) — en `1`/`2`/`3`/`7` siempre cae en la posición segura `SERVO_PULSO_MIN`; `4` y `5` son las excepciones que sí mueven el servo por encima de `SERVO_PULSO_MIN` de forma automática, en pasos chicos y acotados (ver sección 11 y 12). En `1` o `2` se habilita cambiar `SERVO_PULSO_MIN/MAX`. El firmware fuerza `CALIB=0` localmente en el instante que el motor arranca **solo si estaba en `1` o `2`** (nunca en `3`/`4`/`5`/`6`/`7`/`8`/`9`/`10`, que necesitan que el motor siga operando; `6`/`8`/`9` en cambio SÍ se auto-abortan a `0` si el motor se DETIENE a mitad del escalón, o si `MODO` cambia, ver sección 9/12), y también fuerza a `0` en cada arranque del firmware (`CalibFlash_Init()`), sin importar el valor que haya quedado guardado en flash de una sesión anterior — nunca reanuda ningún modo de calibración solo; los modos `3`, `4`, `5`, `6`, `7`, `8` y `9` además se auto-completan solos (vuelven a `0` sin downlink al terminar, ver secciones 9, 10, 11 y 12). **Medida de seguridad adicional**: al entrar a `1`-`10` (downlink aceptado con valor != 0), `SET_RPM` se limpia a `0` (su estado "sin comandar") — evita que un `SET_RPM` que haya quedado de una operación anterior active el PID solo al entrar a `10`, sin que el operador lo haya vuelto a pedir explícitamente para esa sesión (`6` y `9` se pisan solos con su propio escalón de todas formas, ver sección 9/12). Mismo criterio en el camino de regreso: cuando el apagado de seguridad de `main.c` fuerza `CALIB` de `1`/`2` de vuelta a `0` (motor arrancando durante calibración del servo), también limpia `SET_RPM` a `0` — por si se había mandado un `SET_RPM` mientras se calibraba (categoría Proceso, siempre se acepta, sin importar el modo), que no quede activando el PID solo al volver a operación normal |
| 14 | `INTERVALO_ENVIO_OPERATIVO_S` | directo (s) | 2B uint16 | Configuración | Uplink en operación |
| 15 | `INTERVALO_ENVIO_STANDBY_S` | directo (s) | 2B uint16 | Configuración | Uplink en standby |
| 16 | `MODO` | — | 1B | Configuración | 0=Ralentí, 1=Local, 2=Remoto |
| 17 | `PRESION_REMOTO` | x10 | 2B uint16 | Proceso | Presión del aspersor (remota) |
| 18 | `NODE_ID` | — | 1B | Configuración | Uso futuro (multicast) |
| 19 | `RESTAURAR_DEFAULTS` | — | 1B | Comando | Requiere byte confirmación `0xA5` |
| 20 | `FORZAR_REPORTE` | — | 1B | Comando | Requiere byte confirmación `0xA5` |
| 21 | `HISTERESIS_MODO_S` | directo (s) | 2B uint16 | Configuración | Anti-parpadeo intervalo envío |
| 22 | `RESET_REMOTO` | — | 1B | Comando | ⚠️ No conectado aún, ver pendientes |
| 23 | `PRESION_OBJETIVO_LOCAL` | x10 | 2B uint16 | Configuración | Setpoint del lazo externo de `MODO=1` (`presion_pid.c`, sección 4.3) -- también pensado como umbral fase llenado→régimen |
| 24 | `TASA_MAX_CAMBIO_RPM_LLENADO_S` | x10 | 2B uint16 | Configuración | Rampa conservadora (llenado tubería) |
| 25 | `SET_RATIO_AUTO` | x10 (entrada) | 2B uint16 | Calibración | El valor recibido es el RPM que marca un tacómetro de referencia externo en ese instante, NO el ratio — el firmware calcula `SET_RATIO = frecuenciaHz_actual × 60 / RPM_recibido` y lo aplica (misma validación de rango que `SET_RATIO`, mismo campo persistido). El valor vigente devuelto en el ACK es el ratio resultante, codificado x100 como `SET_RATIO` (no eco del RPM recibido). `OUT_OF_RANGE` si se manda `0` o si el motor no tiene lectura de tacómetro válida (`frecuenciaHz == 0`) — no hay nada que calcular sin motor girando. Ver sección 12 |
| 26 | `PID_ASP_KP` | x100 | 2B int16 | Calibración | Con signo. Ganancia proporcional del lazo EXTERNO de presión remota de `MODO=2` (sección 4.3, `presion_pid_remoto.c`) — solo se acepta con `MODO=CALIBRACIÓN` Y `CALIB=0`, igual que `PID_RPM_KP/KI` (gate actualizado 2026-09-29, ver sección 4.4). Default `0` (sin calibrar). ID reusado 2026-09-23 (antes `PRESION_GANANCIA_RPM`, formula lineal reemplazada por este PID — ver sección 4.3) |
| 27 | `PID_ASP_KI` | x1000 | 2B int16 | Calibración | Con signo. Ganancia integral del mismo lazo. Igual gate que `PID_ASP_KP`. Default `0`. ID reusado 2026-09-23 (antes `PRESION_OFFSET_RPM`) |
| 28 | `PID_PSI_KP` | x100 | 2B int16 | Calibración | Con signo. Ganancia proporcional del lazo EXTERNO de presión local de `MODO=1` (sección 4.3, `presion_pid.c`) — solo se acepta con `MODO=CALIBRACIÓN` Y `CALIB=0`, igual que `PID_RPM_KP/KI` (gate actualizado 2026-09-29, ver sección 4.4). Default `0` (sin calibrar). Antes libre (parte de `MECANISMO_*`, removido 2026-09-11), reasignado 2026-09-14 |
| 29 | `PID_PSI_KI` | x1000 | 2B int16 | Calibración | Con signo. Ganancia integral del lazo EXTERNO de presión local. Igual gate que `PID_PSI_KP`. Default `0`. Antes libre, reasignado 2026-09-14 |
| 30 | `PRESION_OBJETIVO_REMOTO` | x10 | 2B uint16 | Proceso | Presión que se ESPERA que reporte el aspersor remoto en `MODO=2` (distinta de `PRESION_REMOTO`, ID 17, que es la lectura real). Es el setpoint real del PID de `MODO=2` (`presion_pid_remoto.c`) Y alimenta el supervisor de "`MODO=2` no alcanza el objetivo" (sección 8). Default `0` = sin configurar (a diferencia de `PRESION_OBJETIVO_LOCAL` local, sí acepta `0` explícitamente, aunque con `0` el PID persigue 0 PSI). Agregado 2026-09-23, cambió de rol el mismo día al reemplazarse la fórmula lineal por el PID |
| 31 | `SET_PRESION` | x10 | 2B uint16 | Proceso | Equivalente de `SET_RPM` (ID 3) pero para presión -- solo tiene efecto (y solo se acepta) con `MODO=3` (`MANUAL_BANCO`, rechaza con `APPLY_ERROR` en cualquier otro `MODO`, mismo patrón que `SET_RPM`). Arma la misma cascada de `MODO=1` (`presion_pid.c`) sin rampa de llenado, ver sección 4.3.1. Mutuamente excluyente con `SET_RPM` (gana el último que llega, el otro se limpia a `0`); se limpia a `0` también al entrar a `MODO=3`. Agregado 2026-09-25 |

**Ganancias PID con signo**: se codifican como `int16` (complemento a
2), no `uint16` — un consumidor debe reinterpretar valores > 32767
restando 65536 antes de dividir por la escala.

---

## 4. Máquinas de estado

### 4.1 Bloqueo de configuración por operación (implementada)

```
Motor detenido  -> cualquier parametro se puede cambiar
Motor operando  -> solo CALIBRACION y PROCESO se aceptan;
                   CONFIGURACION se rechaza (STATUS=5)
```

Determinado por `!Tacometro_EstaDetenido()`, consultado en
`rak3172.c` y pasado a `CalibFlash_ProcesarParametroConEstado()`.

### 4.2 Estado del motor para telemetría (ENCENDIDO/APAGADO/ACTIVO — implementada 2026-09-09)

Distinto de la máquina de estados del gobernador (4.3) — este es el
campo `estado` que viaja en el uplink LIVE (ver sección 6), calculado
en `main.c`:

```
ESTADO_APAGADO   (2): rpm == 0
ESTADO_ENCENDIDO (3): rpm > 0, pero sin presión de salida suficiente
                       (arranque, motobomba sin carga/cebando)
ESTADO_ACTIVO    (1): rpm > 0 Y Presion_GetPresionPsi() > PRESION_UMBRAL_ACTIVO_PSI
                       (ver presion.c, sección 5)
```

⚠️ **`PRESION_UMBRAL_ACTIVO_PSI` (5.0 PSI, `main.c`) es un placeholder
sin dato de campo todavía** — distingue "presión real de bombeo" de
ruido/offset residual con la bomba sin carga, pero el valor correcto
depende del sistema hidráulico real (ver pendientes, sección 8). Si
`Presion_SensorValido()` es `false` (lazo 4-20mA abierto o en
corto/sobre-rango), el estado nunca sube a `ACTIVO` aunque el motor
esté girando — se trata como si no hubiera presión, no como un valor
de `0` confiable.

`inicio_operacion`/`segundos_transcurridos` se recalculan cada vez que
`estado` cambia, usando `Reloj_GetUnixTimeLocal()` (ver el detalle del
"desplazamiento en vez de reinicio" al sincronizar, sección 2.2).

### 4.3 Modos de operación del motor (`MODO` 0/1/2/3/4) — **implementado, renumerado 2026-09-14, extendido 2026-09-24**

`MODO` (ID 16) gobierna de dónde sale el setpoint de RPM del lazo
interno (`pid.c`). Es un parámetro real de protocolo (se manda por
downlink/serial), no solo un nombre interno -- la numeración sigue la
terminología que ya usa el cliente en campo:

```
MODO=0 (Ralentí):        sin control activo, sin importar SET_RPM/
                          PRESION_REMOTO/presión local -- el servo cae a
                          SERVO_PULSO_MIN (ralentí natural). Estado
                          seguro/neutro.
MODO=1 (Presión local):  setpoint = salida del lazo EXTERNO de presión
                          (presion_pid.c), comparando la presión real
                          medida por el sensor PROPIO de este TID
                          contra PRESION_OBJETIVO_LOCAL. Ver detalle abajo.
MODO=2 (Remoto):         setpoint = salida del lazo EXTERNO de presión
                          REMOTA (presion_pid_remoto.c, PID en cascada
                          evento-driven), comparando PRESION_OBJETIVO_REMOTO
                          contra PRESION_REMOTO recibida por downlink de un nodo
                          aspersor REMOTO (no del sensor local). Reemplaza
                          desde 2026-09-23 a la fórmula lineal abierta que
                          tenía antes. Ver detalle abajo.
MODO=3 (Manual/banco):   setpoint = SET_RPM (downlink/serial directo,
                          RPM cruda) O SET_PRESION (downlink directo,
                          agregado 2026-09-25 -- arma la MISMA cascada
                          de presión que MODO=1, sin rampa de llenado,
                          ver sección 4.4), sin ningún lazo automático
                          más allá de la elección del operador. USO
                          OPERATIVO REAL DE CAMPO ÚNICAMENTE (ej.
                          trasladar manguera/aspersor, o sostener una
                          presión puntual sin recomisionar
                          PRESION_OBJETIVO_LOCAL) -- un operador cualquiera
                          puede estar en este modo en plena operación
                          normal, con CALIB=0. Hasta el
                          2026-09-23 este mismo valor también se usaba
                          para las sesiones de calibración (toda la
                          sintonización del PID interno de esta
                          sección, Kp=0.047/Ki=0.11, se hizo en
                          MODO=3) -- desde el 2026-09-24 ESO se movió a
                          MODO=4 (ver abajo), justamente para no
                          mezclar "operador moviendo una manguera" con
                          "sesión de ingeniería" bajo el mismo valor.
MODO=4 (Calibración):    setpoint = SET_RPM para la mayoría de las
                          rutinas (mismo cálculo que MODO=3, ver
                          main.c) -- EXCEPTO con CALIB=10
                          activo, donde en cambio arma el lazo de
                          presión LOCAL real (idéntico a MODO=1, ver
                          nota de `presionLocalActivoAhora` en main.c
                          y sección 4.4/9/12). Agregado 2026-09-24,
                          decisión explícita del usuario: es el ÚNICO
                          MODO desde el que se acepta un
                          CALIB != 0 -- SIN excepciones,
                          ni siquiera `10` (un intento anterior lo
                          eximía aceptando también MODO=1, se descartó
                          a pedido explícito del usuario en favor de
                          esta regla única). Nunca se usa en operación
                          normal de campo, exclusivo de sesiones de
                          banco/ingeniería (puesta en marcha,
                          recalibración anual). Reemplaza a MODO=3 para
                          TODO lo que antes era "modo de banco para
                          calibrar" en esta misma sección 9/12 del
                          README (la sintonización del PID interno, los
                          auto-escalón 6/10/12/9, etc.).
```

⚠️ **RENUMERADO 2026-09-14**: antes `MODO=1` era "SET_RPM directo"
(hoy `MODO=3`) y `MODO=1` no existía como "presión local". Se
renumeró para que coincida con la terminología del cliente (0=ralentí,
1=presión local, 2=presión remota). Como este parámetro solo corrió
hasta ahora en la unidad de banco (sin flota desplegada), no hace
falta coordinar migración con nodos en campo -- pero verificar/mandar
`MODO` explícito en esta unidad después de cada reflasheo con este
cambio, no asumir que quedó en el valor correcto.

⚠️ **`MODO` NUNCA arranca en el valor persistido en flash -- siempre
arranca en `RALENTI` (0)** (decisión explícita del usuario, agregado
2026-09-22, mismo criterio que ya existía para `CALIB` en
`CalibFlash_Init()`). Motivo: se confirmó en campo que el equipo puede
resetearse solo (ver hallazgo de hardware sobre el riel de 3.3V
compartido con el RAK3172, sección de hardware) -- si eso pasara con la
tubería todavía sin llenar (aire adentro) y el equipo volviera directo
a `MODO=1`/`2` persistido, el PID de presión podría pedir RPM alta
persiguiendo una lectura de presión irreal y reventar la tubería.
Arrancar siempre en `RALENTI` exige que un operador confirme a mano
(mandando `MODO` explícito) que todo está en condiciones antes de pasar
a control automático. El valor de `MODO` SÍ se sigue guardando en flash
con cada `Set` (por si hace falta inspeccionarlo después), solo que no
se usa como punto de partida al arrancar.

⚠️ **`MODO=1`/`2` (control automático) se rechazan con
`STATUS=REJECTED_ENGINE_STOPPED` si el motor está detenido** (agregado
2026-09-22, mismo motivo que el punto anterior, decisión explícita del
usuario): si se pudiera "armar" `MODO=1`/`2` con el motor apagado,
apenas arrancara el motor el PID de presión entraría a controlar desde
el primer instante, sin que el operador haya podido confirmar a mano
que la tubería ya está llena. `MODO=0` (RALENTI, siempre seguro) y
`MODO=3` (MANUAL_BANCO, requiere `SET_RPM` explícito del operador, no
reacciona solo a ningún sensor) quedan exentos de este candado — se
pueden mandar en cualquier momento. Nuevo código de `STATUS` agregado
al protocolo: `CALIB_STATUS_REJECTED_ENGINE_STOPPED = 6` (ver tabla de
`STATUS` sección 3) — **actualizar también `NOMBRES_STATUS` en el
`decoder.py` de AWS** (`RIO-DSL-DecodeUplink`) si no se hizo ya.

⚠️ **`SET_RPM` rechaza con `APPLY_ERROR` si `MODO≠3`** (agregado
2026-09-17, tras una confusión real en campo con esta misma
renumeración: se mandó `SET_RPM` estando en `MODO=1` esperando el
comportamiento viejo, y el motor no reaccionaba porque `MODO=1` no lee
`SET_RPM` para nada). Antes de este cambio, `SET_RPM` se aceptaba
igual (`STATUS=OK`) sin importar `MODO`, simplemente no tenía ningún
efecto -- ahora el ACK avisa de una vez, mismo patrón que
`PID_RPM_KP/KI` exigiendo `MODO=CALIBRACIÓN` + `CALIB=0` (ver
sección 4.4).

⚠️ **Pregunta abierta para el cliente**: su equipo anterior
mencionaba una función donde se manda un `SET_RPM` que "se queda
seteado ahí" sin control automático -- falta confirmar si eso
corresponde a lo que hoy es `MODO=3` (probable, es la lectura más
directa) o si en su cabeza correspondía a lo que hoy es `MODO=1`. Por
ahora se dejó como `MODO=3`, sin cambiar código hasta confirmar.

**`MODO=1` (Presión local) — lazo en cascada, agregado 2026-09-14.**
El lazo interno de RPM ya validado (`pid.c`, Kp=0.047/Ki=0.11) no
cambia en nada -- se le agregó un lazo EXTERNO (`presion_pid.c`) que
calcula su setpoint automáticamente:

```
presión medida (PresionV_GetPresionPsi(), sensor propio del TID)
        │
        ▼
   PresionPid_CalcularSetpointRpm(PRESION_OBJETIVO_LOCAL, presión medida)
        │  (PI, ganancias PID_PSI_KP/KI, sin derivativo)
        ▼
   setpointRpm  ──────────►  PID_CalcularSalidaUs() (lazo interno, sin cambios)
```

| ID | Nombre | Escala | Notas |
|---|---|---|---|
| 28 | `PID_PSI_KP` | x100, con signo | Ganancia proporcional del lazo externo |
| 29 | `PID_PSI_KI` | x1000, con signo | Ganancia integral del lazo externo |

Mismo patrón de calibración que `PID_RPM_KP/KI`/`PRESION_GANANCIA_RPM`:
solo se aceptan con `MODO=CALIBRACIÓN` + `CALIB=0` (ver
sección 4.4), con el motor operando,
observando la RPM real contra la presión real -- **arrancan en `0`
(sin calibrar)**, así que `MODO=1` no hace nada hasta que se
caractericen en campo (mismo método usado para el mecanismo directo:
identificar antes de sintonizar, sección 9). Solo P+I, sin derivativo
a propósito -- mismo criterio que el lazo interno (`Kd=0` confirmado
en campo): el lazo externo de una cascada es más lento por diseño y
va a leer una señal probablemente más ruidosa que el tacómetro, un
derivativo ahí solo amplificaría ruido.

**Sensor de presión, TEMPORAL**: mientras se resuelve el circuito de
acondicionamiento del lazo 4-20mA (`presion.c`, sección 5), este lazo
usa `presion_voltaje.c/h` -- un sensor de salida en voltaje (0.5-2.5V,
0-150PSI) leído directo por `ADC2`/`PA4`, sin acondicionamiento propio.
Si el sensor cae en falla (`PresionV_SensorValido()==false`), el lazo
externo no calcula ciegamente sobre una lectura inválida -- el
setpoint se fuerza a "sin comandar" (ralentí natural), igual criterio
que el watchdog de `MODO=2` de abajo. Cuando el circuito 4-20mA esté
listo, sustituir por `Presion_GetPresionPsi()`/`Presion_SensorValido()`
-- la arquitectura de la cascada no depende de qué sensor la alimenta.

**Supervisor de "no se puede llegar a `PRESION_OBJETIVO_LOCAL`"** (agregado
2026-09-14, ver detalle completo en sección 8): si el lazo externo
queda 30s seguidos pidiendo `RPM_MAX` (el techo permitido) sin que la
presión real se acerque al objetivo (±5%), y eso pasa 3 veces seguidas,
imprime `ALERTA,PRESION_OBJETIVO_NO_ALCANZADA,...` por monitor serial
(todavía no por LoRa, no existe un ID de alerta en el protocolo). No es
una protección contra sobre-acelerar — `presion_pid.c` ya recorta su
propia salida a `[0,RPM_MAX]` sin ayuda de este supervisor, esto es
solo para enterarse si la meta pedida no es alcanzable.

**`MODO=2` (Remoto) — PID en cascada evento-driven, igual patrón que
`MODO=1`, pero contra un objetivo remoto — reemplaza desde 2026-09-23 a
la fórmula lineal abierta que tenía antes (`PRESION_OFFSET_RPM +
PRESION_GANANCIA_RPM * PRESION_REMOTO`).** Motivo del reemplazo (decisión
explícita del usuario): la fórmula lineal exigía calibrar dos
constantes **en cada instalación**, porque la relación real
RPM→presión remota depende de la tubería/bomba/aspersor físicos de ese
sitio en particular — si esas constantes no representaban bien esa
instalación, el sistema se asentaba en un punto de equilibrio que podía
no ser el objetivo real, sin ninguna forma de corregirse solo (no había
error calculado contra un objetivo, solo un mapeo fijo). Un PID no
necesita conocer esa relación de antemano: reacciona al error real
(`PRESION_OBJETIVO_REMOTO` menos lo que reporta el aspersor) y se
autoajusta a cualquier instalación, igual que ya hacía `presion_pid.c`
para `MODO=1`.

| ID | Nombre | Escala | Notas |
|---|---|---|---|
| 26 | `PID_ASP_KP` | x100, con signo | Ganancia proporcional |
| 27 | `PID_ASP_KI` | x1000, con signo | Ganancia integral |

Mismo patrón de calibración que `PID_RPM_KP/KI`/`PID_PSI_KP/KI`:
solo se aceptan con `MODO=CALIBRACIÓN` + `CALIB=0` (ver
sección 4.4), con el motor operando,
observando la RPM real contra la `PRESION_REMOTO` remota real. Default de
ambos: `0` (sin calibrar, `MODO=2` no empuja el motor hasta que se
sintonice deliberadamente).

⚠️ **IDs reusados 2026-09-23**: `26`/`27` eran antes `PRESION_GANANCIA_RPM`/
`PRESION_OFFSET_RPM` (la fórmula lineal). Como esa fórmula ya no existe
y el parámetro solo corrió hasta ahora en la unidad de banco (sin flota
desplegada), se reusaron los mismos IDs para las ganancias del PID
nuevo en vez de sumar IDs nuevos -- ver nota en `calibracion_flash.h`.

**Diferencia clave con el lazo de `MODO=1`: es EVENTO-DRIVEN, no corre
en cada vuelta del loop.** El aspersor remoto reporta con una cadencia
muy irregular según su propio estado (confirmado en campo 2026-09-23):
~20-25min con presión en 0, ~20s mientras está en rampa, ~90s ya
estabilizado en su propio objetivo. `presion_pid_remoto.c` solo se
llama cuando llega un reporte **nuevo** de `PRESION_REMOTO` (detectando
cambios en `CalibFlash_GetPresionUltimoTickMs()`, el mismo truco que ya
usa el supervisor de abajo) -- entre reportes, el motor sostiene el
último setpoint calculado, no se vuelve a calcular con el mismo dato
viejo. El paso de integración usa el tiempo REAL transcurrido entre
llamadas (no un ciclo fijo), para que el término integral sea correcto
pese a esa cadencia tan variable. Al entrar a `MODO=2` se calcula un
primer setpoint inmediato con el último dato de `PRESION_REMOTO` conocido (sin
esperar a un reporte nuevo), para no quedarse en `0` hasta 20-25min si
el aspersor está reportando presión baja.

⚠️ **`PRESION_OBJETIVO_REMOTO` (ID 30) cambió de rol el 2026-09-23**:
antes solo alimentaba el supervisor de abajo (opcional, default `0` =
"sin configurar" era inofensivo). Ahora es el setpoint REAL que recibe
el PID -- dejó de ser un dato opcional. Con `0` sin configurar, el PID
persigue literalmente `0` PSI (empuja el motor hacia ralentí la mayor
parte del tiempo) en vez de solo relajar el criterio del supervisor
como antes. **Configurar este valor antes de operar en `MODO=2` pasa a
ser obligatorio en la práctica** -- desde 2026-09-23 el firmware lo
rechaza explícitamente: intentar `MODO=2` con `PRESION_OBJETIVO_REMOTO=0`
devuelve `STATUS=REJECTED_OBJETIVO_REMOTO_NO_CONFIGURADO` (código `7`,
ver tabla de `STATUS` sección 3) y `MODO` no cambia. No es un candado
de seguridad como `REJECTED_ENGINE_STOPPED` (el PID persigue 0 PSI, el
motor cae solo cerca de ralentí, no hay riesgo de sobre-acelerar) --
es para evitar que `MODO=2` quede "andando" sin hacer nada útil, en
silencio, sin que el supervisor de abajo llegue a dispararse nunca
(nunca se satura en `RPM_MAX` si el objetivo es `0`).

**Watchdog de `TIMEOUT_SIN_COMANDO_S` (ID 11) — implementado, solo
aplica en `MODO=2`.** Si no llega una `PRESION_REMOTO` válida en más de ese
tiempo (cuenta desde el arranque si nunca llegó ninguna), el firmware
fuerza el setpoint a "sin comandar" -- el motor cae a ralentí natural
(misma rama física que `MODO=0`) en vez de sostener indefinidamente el
último valor de presión, ya viejo (ej. si la Lambda que reenvía
`PRESION_REMOTO` se cae, o los aspersores se quedan sin batería). Se loguea
por flanco (`MODO_REMOTO: sin PRESION_REMOTO valida...`/`...reanudando control
por presion`), no en cada vuelta del loop. `MODO=0`/`1`/`3` no usan
este parámetro para nada -- solo importa en `MODO=2`.

⚠️ **Default corregido 2026-09-09**: el aspersor reporta cada **90s**
mientras está activamente gobernando el motor en `MODO=2` (el
"15-20 min" documentado antes era del reporte en *standby* del
aspersor, no del estado activo relevante acá). `DEFAULT_TIMEOUT_SIN_COMANDO_S`
se corrigió de `1800s` a **`360s`** (margen de 4x sobre esos 90s,
tolera perder hasta 3 reportes seguidos antes de forzar ralentí) --
con el valor viejo, el motor podía seguir corriendo hasta 20x el
intervalo real de reporte (30 min) con presión completamente obsoleta
sin que el watchdog reaccionara. Sigue siendo un parámetro configurable
por downlink (rango `60-3600s`) -- ajustar por nodo si el patrón real
de pérdida de paquetes de su enlace LoRa lo justifica.

**`PRESION_OBJETIVO_REMOTO` (ID 30) — setpoint del PID de `MODO=2` Y
objetivo del supervisor, agregado 2026-09-23, cambió de rol el mismo
día (ver arriba).** Es la presión que **se espera** que reporte el
aspersor remoto -- distinta de `PRESION_REMOTO` (ID 17), que es la lectura
real que manda el remoto por downlink. Se configura en este mismo TID
(categoría PROCESO, se puede cambiar en caliente con el motor operando,
igual que `PRESION_OBJETIVO_LOCAL` local). Default `0.0` = "sin configurar" --
técnicamente sigue aceptándose (a diferencia de `PRESION_OBJETIVO_LOCAL`
local, que rechaza `0`), pero ya no es inofensivo: con el PID como
consumidor real de este valor, dejarlo en `0` hace que el motor
persiga `0` PSI.

**Supervisor de "`MODO=2` no alcanza el objetivo"** (agregado
2026-09-23, **rediseñado el mismo día** tras dos observaciones del
usuario sobre la primera versión):
1. La ventana de 30s de `PRESION_SUPERVISOR_VENTANA_MS` tiene sentido
   para `MODO=1` porque el sensor es local (lectura nueva cada vuelta
   del loop). En `MODO=2` la `PRESION_REMOTO` viene de un nodo remoto que
   reporta cada ~90s, más el retraso real de transporte por la tubería
   -- contar segundos de reloj no tiene sentido ahí, el setpoint ni
   cambia entre un reporte y el siguiente. **Corregido**: ahora se
   cuentan reportes nuevos consecutivos (detectando cambios en
   `CalibFlash_GetPresionUltimoTickMs()`, el mismo tick que ya usa el
   watchdog de arriba), no tiempo transcurrido -- se adapta solo a
   cualquier cadencia real del enlace remoto.
2. **Corregido**: ahora sí compara contra un objetivo real
   (`PRESION_OBJETIVO_REMOTO` de arriba) cuando está calibrado -- mismo
   criterio que `MODO=1`: saturado en `RPM_MAX` **y** la `PRESION_REMOTO`
   remota real sigue lejos del objetivo (±5%, `PRESION_SUPERVISOR_TOLERANCIA_FRACCION`).
   Si `PRESION_OBJETIVO_REMOTO` no está calibrado (`0`, default), cae al
   criterio más simple: solo "saturado en `RPM_MAX`" (igual que la
   primera versión, sin objetivo con qué comparar).

⚠️ **A diferencia del supervisor de `MODO=1` (solo alerta, nunca actúa
sobre el motor -- decisión original, ver arriba), el de `MODO=2` SÍ
retrocede activamente** (agregado 2026-09-23, a pedido explícito del
usuario, pensado para un escenario real: una fuga entre el motor y el
aspersor -- el aspersor nunca sube de presión por más RPM que se le
pida, así que sin esto el motor se quedaría forzando `RPM_MAX`
indefinidamente sin lograr nada, gastando combustible/motor por gusto).
Secuencia con `PRESION_OBJETIVO_REMOTO` calibrado:
1. **Reporte 1 fuera de tolerancia** (saturado + lejos del objetivo):
   se fuerza "sin comandar" (mismo mecanismo que ralentí, **sin cambiar
   `MODO` de verdad** -- no escribe flash) durante
   `PRESION_SUPERVISOR_REMOTO_RETROCESO_MS` (20s) antes de reintentar.
   Log: `MODO_REMOTO: intento 1/3 sin alcanzar el objetivo -- retrocediendo...`.
2. **Reporte 2 igual de fallido**: mismo retroceso, intento 2/3.
3. **Reporte 3 igual de fallido**: imprime
   `ALERTA,MODO_REMOTO_OBJETIVO_NO_ALCANZADO,intentos=3,presion_remota=...,objetivo_remoto=...,presion_local=...,rpm_max=...`
   Y esta vez sí pasa `MODO` a `RALENTI` de verdad (única escritura a
   flash de todo el ciclo) -- el motor se queda en ralentí hasta que un
   operador reevalúe la situación (probablemente la fuga) y mande
   `MODO=2` explícito de nuevo.

Si en cualquier momento un reporte SÍ cumple tolerancia, el contador se
resetea a `0` (imprime `ALERTA,MODO_REMOTO_OBJETIVO_RECUPERADO` si venía
de haber alertado). `presion_local` en la alerta es solo dato de
contexto -- la lectura del sensor propio del TID en ese instante, que
`MODO=2` no usa para controlar, pero sirve para que el operador compare
a ojo. Si `PRESION_OBJETIVO_REMOTO` no está calibrado (`0`), el criterio
cae a "solo saturado en `RPM_MAX`" pero el retroceso activo sigue
aplicando igual.

**El watchdog de `TIMEOUT_SIN_COMANDO_S` (arriba) tiene prioridad y es
independiente de este ciclo de reintentos** -- si la señal se cae del
todo a mitad de un ciclo (no "llega un reporte diciendo que sigue mal",
sino "no llega nada"), ese es un problema de enlace, no de fuga: el
watchdog ya fuerza ralentí por su cuenta, y este supervisor cancela
cualquier reintento en curso en vez de competir con él.

Misma limitación que el de `MODO=1`: solo monitor serial por ahora, no
existe un ID de alerta en el protocolo para mandarlo por LoRa.

**Pendientes de esta sección (2026-09-14, sin implementar todavía):**
- **Selección de modo por switch físico**: el cliente va a instalar un
  selector físico en el gabinete para cambiar `MODO` localmente, además
  del downlink remoto -- sin código todavía (ni lectura de GPIO, ni
  lógica de selector). Cuando se defina el pin/comportamiento, debería
  bastar con un módulo nuevo que llame a `CalibFlash_SetModo()`, el
  mismo punto de entrada que ya usa el downlink -- no debería requerir
  tocar la lógica de `main.c` de esta sección.
- **Prioridad control local (serial) sobre remoto (downlink)**,
  mencionada por el cliente con el criterio de un variador ABB por
  Modbus (el control local, cuando está activo, tiene prioridad sobre
  cualquier comando remoto). **Hoy no existe** -- `comando_serial.c` y
  `rak3172.c` llaman al mismo `CalibFlash_ProcesarParametroConEstado()`
  sin ningún concepto de "quién manda": el que escriba último gana, sin
  importar el canal. Falta diseñar el mecanismo de prioridad/bloqueo.
- **Reglas de transición entre MODOs**: hoy `CalibFlash_SetModo()`
  acepta cualquier salto directo entre 0/1/2/3, sin restricción de
  secuencia. Se discutió (2026-09-14) la posibilidad de exigir pasar
  por `MODO=0` como estado neutro obligatorio antes de saltar entre
  modos activos (1/2/3) -- pero el flujo real de operación (llenado de
  tubería en `MODO=1` sin necesariamente pasar por ralentí antes de
  `MODO=2`; traslado de equipo en `MODO=3` sin necesariamente bajar
  presión primero) todavía no está claro, así que **se decidió dejar
  pendiente y seguir cambiando `MODO` manualmente sin restricciones**
  hasta que el flujo operativo real esté más definido.

#### 4.3.1 `SET_PRESION` (ID 31) — sostener una presión puntual en `MODO=3` (agregado 2026-09-25)

Equivalente de `SET_RPM` (ID 3) pero para el lazo de presión -- un
downlink de PROCESO (no persistente, se puede mandar con el motor
operando), exclusivo de `MODO=3`, que le da al operador la opción de
elegir, estando en modo banco, entre mandar `SET_RPM` (RPM directa) o
`SET_PRESION` (que el sistema sostenga esa presión solo, vía la misma
cascada `presion_pid.c` que ya usa `MODO=1`, sin necesitar cambiar de
`MODO`).

**Por qué no reusar `PRESION_OBJETIVO_LOCAL` para esto**: `PRESION_OBJETIVO_LOCAL`
es el objetivo de campo ya comisionado para `MODO=1` -- pensado para
configurarse una vez y no andar cambiándolo de un downlink a otro
(decisión explícita del usuario). `SET_PRESION` es lo opuesto: un
valor crudo, de un solo uso, igual de "efímero" que `SET_RPM` --
mismo patrón exacto, solo que para presión en vez de RPM.

**Diferencias con la cascada real de `MODO=1`**:
- **Sin rampa de llenado** (`TASA_LLENADO_PSI_S` no aplica) -- el
  operador es responsable directo del valor que manda, mismo criterio
  que `SET_RPM` (tampoco tiene rampa).
- **SÍ hereda la caída a ralentí si el sensor de presión está en
  falla** (`PresionV_SensorValido()==false`) -- es una protección de
  HARDWARE, no depende de qué valor eligió el operador, así que se
  mantiene igual que en `MODO=1`.
- **SÍ hereda el supervisor de "no alcanza objetivo"** (ver sección
  8), comparando contra `SET_PRESION` en vez de `PRESION_OBJETIVO_LOCAL`
  mientras esté activo.

**Mutuamente excluyente con `SET_RPM`**: gana el último que llega --
mandar `SET_RPM` limpia `SET_PRESION` a `0` y viceversa. `SET_PRESION`
también se limpia a `0` automáticamente al ENTRAR a `MODO=3` (no en
cada downlink `MODO=3` redundante mientras ya se está en `MODO=3`),
para que no quede activando la cascada con un valor de una sesión
anterior sin que el operador lo haya vuelto a pedir.

⚠️ **Bug real encontrado y corregido de paso, al implementar esto**:
el bloque de `main.c` que asigna `setpointRpmCrudo` para
`MODO=3`/`MODO=4` corría DESPUÉS del bloque de la cascada de presión,
y no tenía ninguna condición sobre qué `CALIB` estaba
activo -- le pisaba SIEMPRE el valor ya calculado por
`PresionPid_CalcularSetpointRpm()` con el `SET_RPM` crudo (típicamente
`0`, sin comandar). Esto significa que **`CALIB=10` nunca
llegó a mover RPM de verdad desde que se ensanchó
`presionLocalActivoAhora` el 2026-09-24** para incluir
`MODO=CALIBRACIÓN+CALIB=10` -- el cálculo se hacía, pero se pisaba antes
de llegar al servo. Corregido agregando `&& !presionLocalActivoAhora`
a la condición de ese bloque (mismo arreglo necesario para que
`SET_PRESION` funcione, ya que comparte el mismo camino de código).

#### 4.3.2 `RPM_MAX` vs `RPM_MAX_CARGA` (ID 4 y 32) — dos techos de RPM distintos (agregado 2026-09-28)

Las motobombas de este proyecto tienen un **embrague físico** que
conecta/desconecta la bomba del motor -- esto separa dos riesgos
completamente distintos que antes se confundían bajo un solo concepto
de "techo de RPM":

- **`RPM_MAX`** (ID 4, ya existía): el techo **MECÁNICO** del
  motor/motobomba -- fijo por modelo, no depende de la instalación ni
  de si la bomba está embragada o no. Protege contra sobre-acelerar el
  motor en sí (relevante incluso con la bomba desembragada, girando en
  vacío).
- **`RPM_MAX_CARGA`** (ID 32, nuevo): el techo **CON CARGA**
  (hidráulico) -- específico de cada instalación (ej. "no reventar la
  tubería de esta bomba en particular"), SIEMPRE `≤ RPM_MAX` (rechazado
  si se intenta configurar más alto). Solo tiene sentido cuando la
  bomba está embragada y hay presión hidráulica real de por medio.

**Cuál se usa en cada momento** (`main.c`, variable `cargaConectada`,
recalculada en cada vuelta del loop antes de aplicar el clamp final al
setpoint que llega al servo):

```
CON CARGA (RPM_MAX_CARGA):
  MODO=1 (presión local, real)
  MODO=2 (remoto, real)
  MODO=3 (manual/banco, CUALQUIER uso -- SET_RPM o SET_PRESION -- se
          asume embragada por default, más conservador)
  MODO=4 (calibración) con CALIB=9, =10 u =12 (todos
          corren con presión/aspersor real de por medio)

SIN CARGA (RPM_MAX):
  MODO=4 (calibración) con cualquier OTRO CALIB --
          0/4/5/6/7/8 (zona muerta, curva de ganancia interna,
          ralentí, sintonización/validación del PID interno) -- son
          las rutinas de banco tempranas de la secuencia de
          comisionamiento (sección 12), pensadas para correrse con la
          bomba DESEMBRAGADA.
```

Este mismo criterio también decide qué techo usa el mapeo de ganancia
interno: `CALIB=5` (sin carga) usa `RPM_MAX` directo, vía
su propio chequeo. `CALIB=9` (presión, ver sección
4.4/9) usa `RPM_MAX_CARGA` como techo de su propia rampa, y además
queda acotado por el mismo clamp final universal (`cargaConectada` lo
incluye en la lista "con carga") como segunda protección. Los
supervisores de presión local/remota (sección 8) también comparan
contra `RPM_MAX_CARGA`, no `RPM_MAX` (siempre corren en escenarios con
carga real).

⚠️ **Este cambio agregó un campo nuevo a la estructura de flash
persistida** (`rpmMaxCarga`) -- el "magic" de flash subió de `"CALL"`
a `"CALM"` (ver `calibracion_flash.c`), así que **la próxima reflash
de un TID ya en campo va a resetear TODA la calibración a los valores
por defecto** (mismo efecto ya documentado en incidentes previos de
este tipo) -- hay que reasertar `SET_RATIO`/`RPM_MIN`/`RPM_MAX`/PID/etc.
a mano después de esa reflash. `RPM_MAX_CARGA` arranca en el mismo
valor que `RPM_MAX` (sin restricción adicional) hasta que se configure
explícitamente el límite real de cada instalación.

### 4.4 Modo calibración del servo (`CALIB`, implementada)

Independiente de 4.1-4.3 — no es un modo de operación del motor, es el
mecanismo para ajustar en banco los topes mecánicos del acelerador
(`SERVO_PULSO_MIN/MAX`), sintonizar el PID, y auto-calibrar el
ralentí y el propio `SERVO_PULSO_MIN`, sin reflashear. Tres grupos,
por seguridad (ver también la tabla de categorías en la sección 2.3):

```
APAGADO   (CALIB=1 o =2): mueven el servo directo, sin
          ninguna realimentación de RPM -- exigen el motor detenido
          para ENTRAR (rechazo REJECTED_ENGINE_RUNNING si no), y se
          fuerza la salida a 0 si el motor arranca mientras se está en
          cualquiera de los dos.
ENCENDIDO (CALIB=0, =6, =7, =8, =9, =10, =11 o =12): no
          cambian cómo se maneja el servo respecto a la operación
          normal -- no exigen detener el motor para entrar; de hecho
          6/7/8/9/10/11/12 solo tienen sentido con el motor ya operando.
CALIBRACIÓN-SERVO-MIN (CALIB=4): también mueve el servo
          directo, como 1/2 -- pero a diferencia de esos dos, SÍ mira
          la RPM en cada paso (ver sección 11), así que tampoco exige
          el motor operando para entrar, igual que 4.
```

`CALIB=0` en realidad cubre dos sub-casos, resueltos en
`main.c` cada vuelta del loop según el motor **y** si se comandó
control (`SET_RPM > RPM_MIN`, ver 2.4):

```
CALIBRACIÓN-BARRIDO (CALIB=2): servo en barrido continuo
                                     MIN <-> MAX. Requiere motor
                                     detenido para entrar y para
                                     permanecer -- se sale solo si el
                                     motor arranca.
CALIBRACIÓN-MANUAL  (CALIB=1): servo quieto en su
                                     posición actual; cada downlink de
                                     SERVO_PULSO_MIN o SERVO_PULSO_MAX
                                     lo mueve directo a ese valor.
                                     Requiere motor detenido para
                                     entrar y para permanecer, igual
                                     que el barrido.
⚠️ SINTONIZACIÓN-PID (CALIB=10 en la numeración anterior a
                                     2026-09-29 -- no confundir con el
                                     CALIB=10 actual, auto-escalón de
                                     presión local) ELIMINADO 2026-09-29 --
                                     decisión explícita del usuario: las
                                     rutinas CALIB existen para que el
                                     operador no tenga que saltar de una
                                     a otra, y exigir entrar a `=10`
                                     solo para poder tocar ganancias iba
                                     contra esa idea. `PID_RPM_KP/KI` (y
                                     `PID_PSI_KP/KI`/
                                     `PID_ASP_KP/KI`) ahora
                                     se desbloquean con `MODO=CALIBRACIÓN`
                                     + `CALIB=0` (ningún CALIB
                                     activo) -- apenas un CALIB automático
                                     termina y vuelve solo a `0`, ya se
                                     puede tocar la ganancia directo,
                                     sin ese salto de más. El log
                                     `PID_TEST` (y sus equivalentes de
                                     presión) sigue activo bajo los CALIB
                                     automáticos que ya lo generaban
                                     (`=6`/`=7` para RPM, `=10`/`=11`
                                     para presión local, `=12` para
                                     remoto) -- si se quiere ver el log
                                     en una sesión totalmente manual sin
                                     ningún CALIB automático, ya no hay un
                                     modo dedicado para eso.
                                     El viejo `CALIB=10` (numeración
                                     anterior a 2026-09-29, sintonización
                                     manual) ahora se
                                     RECHAZA explícitamente (no queda
                                     solo "sin uso").
CALIBRACIÓN-SERVO-MIN (CALIB=4): barre el pulso hacia
                                     arriba en pasos chicos desde
                                     SERVO_PULSO_MIN, buscando dónde el
                                     motor empieza a acelerar de verdad
                                     (ver sección 11). Tampoco exige
                                     motor detenido para entrar -- si
                                     se activa sin el motor operando,
                                     espera a que arranque antes de
                                     medir la línea base y barrer.
                                     Solo se puede pedir viniendo de
                                     SEGURO/PID ACTIVO
                                     (CALIB=0), no directo
                                     desde 1/2/8. Renumerado
                                     2026-09-23 (antes era el valor
                                     `5`) -- se corrió a un número MÁS
                                     BAJO, antes del ralentí, porque
                                     `SERVO_PULSO_MIN` es la línea base
                                     de la propia fórmula de salida de
                                     `pid.c` (`rango = maximo -
                                     minimo`): sintonizar el PID
                                     interno (el viejo CALIB=10 de
                                     aquel momento, numeración anterior
                                     a 2026-09-29 -- hoy CALIB=0, más
                                     abajo) con una
                                     zona muerta todavía no definitiva
                                     invalidaría esa sintonización en
                                     cuanto se corrigiera después.
AUTO-ESCALÓN-INTERNO (CALIB=6, agregado 2026-09-23 en la
                                     segunda renumeración): automatiza
                                     un escalón del lazo INTERNO de RPM
                                     (PID#1) -- mismo espíritu que
                                     AUTO-ESCALÓN-LOCAL/REMOTO de abajo
                                     (mismo patrón que `REMOTO_CAL`),
                                     pero para este lazo. Requiere
                                     `MODO=4` (CALIBRACIÓN, antes era
                                     `MANUAL_BANCO`=3 hasta el
                                     2026-09-24) y el motor
                                     operando; usa `RPM_MIN` como línea
                                     base (no `CalibFlash_GetSetRpm()`,
                                     que leería `0`, porque cualquier
                                     entrada a `CALIB` con
                                     valor `!=0` ya limpia `SET_RPM`,
                                     ver más abajo) y comanda
                                     `SET_RPM = RPM_MIN + 200` RPM fijo
                                     (`INTERNO_CAL_ESCALON_RPM`). Al
                                     ENTRAR, guarda el `PID_RPM_KP/KI` que
                                     hubiera en flash y los pisa con
                                     `1.0`/`0.0` fijo (agregado
                                     2026-09-25 -- antes era una
                                     convención PROCEDIMENTAL, el
                                     operador los bajaba a mano por
                                     downlink antes de mandar `=6`);
                                     restaura los valores guardados al
                                     SALIR, sin importar el motivo
                                     (completo, abortado, o incluso
                                     cancelado mientras todavía esperaba
                                     el motor) -- la `Kp`/`Ki` ya
                                     sintonizada que hubiera cargada
                                     nunca se pierde, solo queda en
                                     pausa mientras dura la prueba.
                                     Dejándolo correr una ventana FIJA
                                     de 60 segundos
                                     (`INTERNO_CAL_DURACION_MS` --
                                     margen generoso: el lazo interno
                                     asienta en segundos, a diferencia
                                     de los lazos de presión que
                                     necesitan 10-60 minutos). Loguea
                                     con el mismo formato `PID_TEST`
                                     que ya usa la sesión manual (ver
                                     sección 9), para que
                                     `pid_tuning.py identify`/`auto
                                     --loop interno` funcionen sin
                                     cambios sobre su salida. Se aborta
                                     y resetea `SET_RPM` a `0.0` si el
                                     motor se detiene o `MODO` deja de
                                     ser `4` (CALIBRACIÓN) a mitad de la
                                     prueba;
                                     auto-revierte `CALIB`
                                     a `0` al completar o abortar. Log:
                                     `INTERNO_CAL,INICIO,...` /
                                     `INTERNO_CAL,ESCALON,...` /
                                     `INTERNO_CAL,COMPLETO,duracion_s=...`
                                     / `INTERNO_CAL,ABORTADO,motivo=...`.
                                     Igual que `10`/`12` (más abajo), NO
                                     aplica ninguna ganancia sola --
                                     solo automatiza capturar el log
                                     del escalón; el operador sigue
                                     corriendo `pid_tuning.py auto
                                     --loop interno --ganancia-log <log
                                     de CALIB=5>` y
                                     cargando/confirmando `Kp`/`Ki` a
                                     mano (con `MODO=CALIBRACIÓN` +
                                     `CALIB=0`, ver sección
                                     4.4)
                                     antes de confiar en el resultado
                                     (ver sección 9/12). Solo se puede
                                     pedir viniendo de SEGURO/PID
                                     ACTIVO (CALIB=0), no
                                     directo desde 1/2/4/8/10/12.
AUTO-VALIDACIÓN-INTERNO (CALIB=7, agregado 2026-09-25):
                                     verifica en el motor real la Kp/Ki
                                     ya recomendada y cargada, en vez de
                                     identificar una nueva -- a
                                     diferencia de `6` (que SIEMPRE
                                     fuerza `PID_RPM_KP=1/PID_RPM_KI=0` fijo
                                     para la identificación), este modo
                                     NO toca `PID_RPM_KP/KI`: usa lo que ya
                                     esté en flash. Mismo escalón
                                     (`SET_RPM = RPM_MIN + 200`) y mismo
                                     log `PID_TEST` que `6`, pero con
                                     una ventana de 4 minutos
                                     (`VALIDACION_CAL_DURACION_MS`, más
                                     larga que los 60s de `6` porque con
                                     `Ki` real activo puede tardar más
                                     en asentar) y un watchdog de
                                     oscilación sostenida en vivo: pasada
                                     una espera inicial de 30s
                                     (`VALIDACION_CAL_ESPERA_ASENTAMIENTO_MS`,
                                     para no confundir el acercamiento
                                     normal al setpoint con oscilación),
                                     cuenta cada vez que
                                     `RPM_filtrada - SET_RPM` cruza de
                                     signo tras superar un piso de 15 RPM
                                     (`VALIDACION_CAL_OSC_UMBRAL_RPM`,
                                     anti-ruido) -- si se acumulan 4
                                     cruces (`VALIDACION_CAL_OSC_MAX_CRUCES`)
                                     dentro de una ventana móvil de 20s
                                     (`VALIDACION_CAL_OSC_VENTANA_MS`),
                                     aborta de inmediato
                                     (`SET_RPM=0.0`), mismo criterio que
                                     el operador aplicaba a ojo ("si veo
                                     que loquea, mando SET_RPM=0"), ahora
                                     automático. También se aborta si el
                                     motor se detiene o `MODO` deja de
                                     ser `4` a mitad de la prueba, y
                                     auto-revierte a `CALIB=0`
                                     al completar o abortar (por
                                     cualquiera de los tres motivos). Log:
                                     `VALIDACION_CAL,INICIO,...` /
                                     `VALIDACION_CAL,ESCALON,...` /
                                     `VALIDACION_CAL,COMPLETO,duracion_s=...`
                                     / `VALIDACION_CAL,ABORTADO,motivo=motor_se_detuvo|modo_cambio|oscilacion_sostenida`.
                                     El log resultante (mismo formato
                                     `PID_TEST` que `6` -- y que el viejo
                                     CALIB=10 pre-2026-09-29, eliminado)
                                     se le pasa
                                     directo a `pid_tuning.py compare`
                                     (ver sección 9) -- reemplaza el
                                     "dejalo asentar 4 minutos y mirá
                                     cómo se ve" manual. Solo se puede
                                     pedir viniendo de SEGURO/PID ACTIVO
                                     (CALIB=0).
CALIBRACIÓN-RALENTÍ (CALIB=8): servo fijo en
                                     SERVO_PULSO_MIN mientras dura la
                                     ventana de medición (ver sección
                                     10). Tampoco exige motor detenido
                                     para entrar -- si se activa sin el
                                     motor operando, simplemente espera
                                     a que arranque antes de empezar a
                                     contar la ventana. Solo se puede
                                     pedir viniendo de SEGURO/PID
                                     ACTIVO (CALIB=0), no
                                     directo desde 1/2. Renumerado
                                     2026-09-23 (antes era el valor
                                     `4`) -- se corrió a un número MÁS
                                     ALTO, después de zona
                                     muerta/curva de ganancia/PID
                                     interno, porque a diferencia de
                                     esos tres (que dan el mismo
                                     resultado con o sin carga
                                     hidráulica) el ralentí real de un
                                     motor diesel SÍ cambia según haya
                                     o no carga conectada (confirmado
                                     en campo) -- conviene medirlo una
                                     sola vez, ya con la tubería
                                     conectada, en vez de repetirlo.
AUTO-ESCALÓN-LOCAL  (CALIB=10, agregado 2026-09-23,
                                     renumerado de `7` a `8` en la
                                     segunda renumeración del mismo
                                     día): automatiza un escalón del
                                     lazo EXTERNO de presión LOCAL --
                                     una vez motor+`MODO=4` confirmados,
                                     primero MIDE una presión base real
                                     (promedio de `PresionV_GetPresionPsi()`
                                     sobre `PRESION_CAL_BASE_PROMEDIO_MS`,
                                     agregado 2026-09-25 -- antes el
                                     escalón salía del `PRESION_OBJETIVO_LOCAL`
                                     PERSISTIDO, un setpoint operativo de
                                     campo que podía estar desactualizado,
                                     sin relación con la presión real del
                                     momento; ahora usa la misma
                                     medición que ve la cascada real),
                                     y recién con esa base manda
                                     `PRESION_OBJETIVO_LOCAL` = base medida +
                                     5 PSI fijo (`PRESION_CAL_ESCALON_PSI`)
                                     y loguea `PRESION_PID_TEST` durante
                                     una ventana fija de 3 minutos
                                     (`PRESION_CAL_DURACION_MS`, ver
                                     sección 9). Al ENTRAR, guarda el
                                     `PRESION_OBJETIVO_LOCAL` original (para
                                     restaurarlo al salir, sin importar
                                     el motivo) y el `PID_PSI_KI`
                                     que hubiera, pisándolo a `0` fijo
                                     durante la prueba (mismo motivo
                                     matemático que en el lazo interno --
                                     con `Ki` activo la fórmula de `K` se
                                     indefine) -- restaura ambos al
                                     salir. El `PID_PSI_KP` de prueba
                                     NO se toca (a diferencia del lazo
                                     interno, la ganancia de presión
                                     varía mucho por instalación, así que
                                     sigue siendo el operador quien lo
                                     elige -- ver `pid_tuning.py
                                     sugerir-kp-presion`, sección 9).
                                     Solo automatiza mandar el escalón y
                                     esperar -- el PID sigue siendo el
                                     que maneja el servo, igual que en
                                     `0`. Requiere `MODO=4` (CALIBRACIÓN,
                                     NO `MODO=1` desde 2026-09-24 --
                                     ver 4.3) ya puesto por el
                                     operador y el motor operando para
                                     ARRANCAR el escalón -- el lazo de
                                     presión REAL igual corre mientras
                                     tanto (`main.c` trata
                                     MODO=CALIBRACIÓN+CALIB=10 igual que
                                     MODO=1 real para todo lo que
                                     importa: init, rampa de llenado,
                                     falla de sensor, supervisor). Si
                                     se pide antes, se queda esperando
                                     (no es un error). Solo se puede pedir
                                     viniendo de SEGURO/PID ACTIVO
                                     (CALIB=0), no directo
                                     desde 1/2/4/6/8/9/12. Ver sección
                                     12 (Fase E).
AUTO-ESCALÓN-REMOTO (CALIB=12, agregado 2026-09-23,
                                     renumerado de `8` a `9` en la
                                     segunda renumeración del mismo
                                     día): mismo espíritu que el
                                     anterior pero para el lazo EXTERNO
                                     de presión REMOTA (`MODO=2`),
                                     probado en banco con `MODO=4`
                                     (CALIBRACIÓN, antes `MANUAL_BANCO`=3
                                     hasta el 2026-09-24) -- manda `SET_RPM`
                                     = `RPM_MIN` + 200 RPM fijo
                                     (`REMOTO_CAL_ESCALON_RPM`) y
                                     loguea `REMOTO_PID_TEST`. Termina
                                     al acumular 15 reportes de presión
                                     remota nuevos
                                     (`REMOTO_CAL_REPORTES_OBJETIVO`) o
                                     al vencer un timeout de seguridad
                                     de 60 minutos
                                     (`REMOTO_CAL_TIMEOUT_MS`), lo que
                                     ocurra primero -- el reporte del
                                     aspersor remoto es irregular (20s
                                     a 20-25min), así que no alcanza un
                                     tiempo fijo como en el escalón
                                     local. Requiere `MODO=4`
                                     (CALIBRACIÓN) ya puesto
                                     y el motor operando para arrancar;
                                     si se pide antes, espera. Solo se
                                     puede pedir viniendo de SEGURO/PID
                                     ACTIVO (CALIB=0), no
                                     directo desde 1/2/4/6/8/9/10. Ver
                                     sección 12 (Fase E).
AUTO-MAPEO-GANANCIA-PRESIÓN (CALIB=9, agregado
                                     2026-09-24, diseño definitivo
                                     2026-09-28 -- ver sección 9): rampa
                                     de `SET_RPM` en lazo ABIERTO (sin
                                     `presion_pid.c`, sin
                                     `PID_PSI_KP/KI` de por medio --
                                     el servo lo mueve el PID#1 de RPM
                                     ya validado), de `RPM_MIN` hacia
                                     `RPM_MAX_CARGA`, con un FRENO POR
                                     PSI REAL: mide la presión base en
                                     `RPM_MIN` (5s), calcula el ritmo
                                     máximo seguro `(PRESION_OBJETIVO_LOCAL −
                                     base) / TIEMPO_LLENADO_S` (el mismo
                                     que ya usa la rampa de llenado de
                                     `MODO=1`), y en cada ciclo solo
                                     sigue subiendo RPM si la presión
                                     real va POR DEBAJO de ese ritmo --
                                     si lo alcanza, pausa (no baja) hasta
                                     que el ritmo la vuelva a superar.
                                     Así la subida de presión nunca es
                                     más rápida de lo que el operador ya
                                     definió como seguro, sin importar
                                     la ganancia real de la instalación,
                                     y la prueba dura lo que el operador
                                     dijo que tarda el llenado. Loguea
                                     `PRESION_GANANCIA_CAL,PASO,rpm=...,
                                     psi=...,salto=...` cada vez que el
                                     PSI real avanza 1 PSI (tamaño del
                                     log acotado por el rango de
                                     presión, no por el tiempo) --
                                     compatible directo con
                                     `pid_tuning.py sugerir-kp-presion`/
                                     `--ganancia-log`. Termina por lo que
                                     ocurra primero: PSI real llega a
                                     `PRESION_OBJETIVO_LOCAL`
                                     (`objetivo_alcanzado`); RPM llega a
                                     `RPM_MAX_CARGA` sin alcanzarlo
                                     (`rpm_max_carga_alcanzado` --
                                     respaldo de seguridad, y además
                                     diagnóstico: la bomba no llega a esa
                                     presión dentro de su límite seguro);
                                     o timeout de último recurso de
                                     `1.5 × TIEMPO_LLENADO_S`. Si la
                                     presión base ya está en o sobre el
                                     objetivo, completa de inmediato
                                     (`objetivo_ya_alcanzado`). Aborta si
                                     el motor se detiene, cambia `MODO`,
                                     o el sensor de presión deja de ser
                                     válido (`sensor_invalido`). ⚠️ NO
                                     reemplaza el primer llenado/purga
                                     de aire manual (sección 12, Fase
                                     D): el sensor de presión no puede
                                     distinguir agua real de aire
                                     comprimido, así que ninguna rampa,
                                     por lenta que sea, puede verificar
                                     por software que el aire ya salió
                                     -- asume la tubería YA llena. Dos
                                     diseños previos se descartaron el
                                     mismo día: pasos discretos con
                                     espera/umbral de salto (3 bugs
                                     reales de calibración) y reusar la
                                     cascada de `MODO=1` (dependía de un
                                     `Kp` ya cargado -- con `Kp=0` de
                                     fábrica no hacía nada -- y el PID
                                     contaminaba la curva medida).
                                     Requiere
                                     `MODO=4` (CALIBRACIÓN) ya puesto y
                                     el motor operando para arrancar; si
                                     se pide antes, espera. Solo se puede
                                     pedir viniendo de SEGURO/PID ACTIVO
                                     (CALIB=0), no directo
                                     desde 1/2/4/6/8/10/12.
SEGURO      (CALIB=0, motor detenido, O motor
             operando en su ralentí natural sin SET_RPM > RPM_MIN
             comandado):             servo fijo/regresando a
                                     SERVO_PULSO_MIN (sin
                                     aceleración). Estado por defecto.
PID ACTIVO  (CALIB=0, motor operando, Y SET_RPM >
             RPM_MIN comandado explícitamente): servo controlado por
                                     pid.c/h (ver 2.4), no por una
                                     posición fija.
```

- `* -> CALIBRACIÓN-BARRIDO/MANUAL`: downlink `CALIB=1`
  o `=2`, solo se acepta con el motor detenido — con el motor operando
  se rechaza con `REJECTED_ENGINE_RUNNING` (STATUS=5).
- ⚠️ **Cualquier `CALIB` distinto de `0` exige `MODO=4`
  (CALIBRACIÓN) ya puesto — SIN EXCEPCIONES, ni siquiera `8`**
  (agregado 2026-09-24, decisión explícita del usuario, versión final
  tras varios rediseños en la misma sesión -- primero se probó exigir
  `MODO=0`, rompía `8`/`10`; después una excepción puntual para `8`
  aceptando `MODO=1`, funcionaba pero el usuario pidió eliminar
  cualquier excepción; el motivo real de fondo era que
  `MODO=3`/`MANUAL_BANCO` hacía doble función, uso operativo de campo Y
  modo de banco, sin forma de distinguir una cosa de la otra — ver
  `CALIB_MODO_CALIBRACION` en la sección 4.3). Rechazo `OUT_OF_RANGE`
  si se pide con cualquier otro `MODO`. Antes de este cambio (numeración
  de la época, pre-2026-09-29: sintonización de PID era el viejo `10`),
  `1`/`2`/`3`/`4`/`5`/`10` no tenían **ninguna** protección contra
  mandarse por error durante producción real (`MODO=1`/`2`): por
  ejemplo, `CALIBRACIÓN-SERVO-MIN` (`4`) mandado sin querer con el
  equipo bombeando de verdad tomaría control directo del servo,
  sacándolo del lazo de presión en pleno funcionamiento. `AUTO-ESCALÓN-LOCAL`
  (`10`) necesita el lazo de presión LOCAL corriendo DE VERDAD para
  poder meterle un escalón -- en vez de una excepción en este candado,
  `main.c` ensancha su propia condición interna para que
  `MODO=4`+`CALIB=10` dispare el mismo lazo real que `MODO=1` (init,
  rampa de llenado, falla de sensor, supervisor, ver sección 9), así
  que `10` también corre EXCLUSIVAMENTE bajo `MODO=4`, sin necesitar
  ninguna excepción acá. Esta regla se apoya en el forzado automático
  de `MODO=0` al detenerse el motor (ver más abajo) -- detener el
  motor (o recién haber arrancado el TID) deja el camino libre para
  pasar a `MODO=4` y arrancar cualquier sesión. `CALIB=0`
  (salir) siempre se acepta, sin importar `MODO`.
- `SEGURO/PID ACTIVO (0) -> AUTO-ESCALÓN-INTERNO (6), AUTO-ESCALÓN-LOCAL
  (10), AUTO-ESCALÓN-REMOTO (12), AUTO-BARRIDO-GANANCIA-PRESIÓN (9),
  VALIDACIÓN-INTERNO (7), VALIDACIÓN-PRESIÓN (11), CALIBRACIÓN-RALENTÍ
  (8) o CALIBRACIÓN-SERVO-MIN (4)`: downlink directo, con el motor
  operando o detenido indistinto — ninguno de estos mueve el servo de
  forma distinta a la operación normal, así que no hace falta detener
  el motor primero (en la práctica la mayoría solo arrancan el
  escalón/barrido con el motor ya operando y `MODO=4` puesto, pero se
  quedan esperando en vez de rechazar el downlink si se piden antes);
  `4` sí mueve el servo directo, pero como mira la RPM
  en cada paso tampoco exige detenerlo, solo espera si hace falta.
  Todos estos además solo se aceptan viniendo de `0` (rechazo
  `OUT_OF_RANGE` si se piden desde `1`, `2`, o uno desde el otro).
  ⚠️ `SINTONIZACIÓN-PID (10)` ELIMINADO 2026-09-29 (ver sección 4.4) --
  era el único de este grupo sin la restricción de origen desde `0`,
  ya no existe.
- `CALIBRACIÓN-BARRIDO ⇄ CALIBRACIÓN-MANUAL`: por downlink directo
  entre `1` y `2` (ambos requieren motor detenido para el cambio). Al
  entrar a `CALIBRACIÓN-MANUAL` el servo arranca quieto en la posición
  en la que estaba (no salta a `SERVO_PULSO_MIN` ni `MAX`) hasta el
  primer downlink de esos dos parámetros.
- `CALIBRACIÓN-BARRIDO/MANUAL -> SEGURO`: por downlink
  `CALIB=0`, **o** automáticamente en el firmware (sin
  downlink) si el motor arranca mientras se está en uno de esos dos
  modos — medida de seguridad, nunca se deja el servo bajo barrido o
  posición manual con el motor operando. Si además ya había un
  `SET_RPM > RPM_MIN` comandado desde antes, ese mismo arranque cae
  directo en `PID ACTIVO` en vez de `SEGURO`. **`CALIBRACIÓN-RALENTÍ`,
  `CALIBRACIÓN-SERVO-MIN`, `AUTO-ESCALÓN-INTERNO`,
  `AUTO-ESCALÓN-LOCAL` y `AUTO-ESCALÓN-REMOTO` no tienen esta salida
  automática por ARRANQUE del motor** — el motor arrancando (o ya
  estando operando) ahí es justamente lo esperado, no una condición de
  falla. `AUTO-ESCALÓN-INTERNO`/`AUTO-ESCALÓN-LOCAL`/`AUTO-ESCALÓN-REMOTO`
  sí tienen su propia salida automática, pero por el motivo contrario
  (el motor DETENIÉNDOSE, o `MODO` cambiando) — ver los bullets
  siguientes.
- ⚠️ **`CALIBRACIÓN-BARRIDO`/`MANUAL`/`AUTO-ALPHA`/`CALIBRACIÓN-SERVO-MIN`/
  `CALIBRACIÓN-CURVA-DE-GANANCIA`/`CALIBRACIÓN-RALENTÍ` (`1`/`2`/`3`/`4`/`5`/`8`)
  salen automáticamente a `SEGURO` si `MODO` deja de ser `CALIBRACIÓN`
  a mitad de sesión, sin esperar ningún downlink nuevo** (bug real
  encontrado en campo y corregido 2026-09-25: con `MODO=4` puesto,
  `CALIB=2` arrancado, y `MODO` vuelto a `0` después, el
  barrido seguía corriendo -- el candado de `MODO=CALIBRACIÓN` solo se
  chequeaba UNA VEZ, al aceptar el downlink de `CALIB`,
  nada lo volvía a chequear mientras la sesión seguía activa). Este
  grupo específico (`1`/`2`/`3`/`4`/`5`/`8`) no tenía ningún chequeo
  continuo de `MODO` propio, a diferencia de `6`/`7`/`9`/`10`/`11`/`12`
  (que ya abortan solos si `MODO` cambia, dentro de su propia máquina
  de estados). Mismo criterio que la salida
  automática por arranque del motor (bullet anterior): control físico
  real, nunca un switch remoto para algo de seguridad.
- `AUTO-ESCALÓN-INTERNO -> SEGURO`: automático, sin downlink, al
  completar la ventana fija de 60 segundos del escalón
  (`INTERNO_CAL,COMPLETO`, ver sección 9/12), reseteando `SET_RPM` a
  `0.0` — o abortado antes al mismo destino
  (`INTERNO_CAL,ABORTADO,motivo=...`) si el motor se detiene o `MODO`
  deja de ser `3` a mitad del escalón.
- `AUTO-ESCALÓN-LOCAL -> SEGURO`: automático, sin downlink, al
  completar la ventana fija de 3 minutos del escalón
  (`PRESION_CAL,COMPLETO`, ver sección 9/12), restaurando
  `PRESION_OBJETIVO_LOCAL` a su valor anterior — o abortado antes al mismo
  destino (`PRESION_CAL,ABORTADO,motivo=...`) si el motor se detiene o
  `MODO` deja de ser `1` a mitad del escalón.
- `AUTO-ESCALÓN-REMOTO -> SEGURO`: automático, sin downlink, al
  acumular 15 reportes de presión remota nuevos tras el escalón o al
  vencer el timeout de 60 minutos, lo que ocurra primero
  (`REMOTO_CAL,COMPLETO,motivo=reportes_objetivo|timeout`, ver sección
  9/12), reseteando `SET_RPM` a `0.0` — o abortado antes al mismo
  destino (`REMOTO_CAL,ABORTADO,motivo=...`) si el motor se detiene o
  `MODO` deja de ser `3` a mitad del escalón.
- `CALIBRACIÓN-RALENTÍ -> SEGURO`: automático, sin downlink, en cuanto
  se completa la ventana de medición de 2 minutos (aplica el nuevo
  `RPM_MIN` y vuelve a `CALIB=0` solo, ver sección 10). Si
  el motor se detiene a mitad de la ventana, la medición en curso se
  descarta pero se permanece en `CALIBRACIÓN-RALENTÍ`, esperando a que
  el motor vuelva a arrancar, sin necesidad de reenviar el downlink.
- `CALIBRACIÓN-SERVO-MIN -> SEGURO`: automático, sin downlink, al
  detectar el umbral de aceleración (aplica el nuevo `SERVO_PULSO_MIN`
  y vuelve a `CALIB=0` solo) o al llegar al techo de
  seguridad sin detectar nada (ver sección 11 — en ese caso no se
  aplica ningún cambio, es un error, no un resultado). Igual que en
  `CALIBRACIÓN-RALENTÍ`, si el motor se detiene a mitad del barrido, se
  descarta el progreso pero se permanece en el modo esperando a que
  vuelva a arrancar.
- `SEGURO ⇄ PID ACTIVO` (dentro de `CALIB=0`):
  automático según `Tacometro_EstaDetenido()` y si `SET_RPM` supera
  `RPM_MIN`, sin downlink de por medio para el motor (arrancar/detener
  basta) — el ralentí (Modo 0, sección 4.3) es deliberadamente sin
  control, nunca se recorta `SET_RPM` hacia arriba hasta `RPM_MIN` para
  forzar el lazo. Al entrar a `PID ACTIVO` se llama `PID_Init()` una
  sola vez (flanco de entrada), para no arrastrar estado de una
  activación anterior. El comportamiento del servo en `CALIB=0` no
  cambia según `MODO=CALIBRACIÓN` haya desbloqueado `PID_RPM_KP/KI` (ver
  sección 9) o no — la diferencia está solo en qué parámetros se
  pueden tocar y si se ve el log `PID_TEST`, no en el control en sí. En
  `CALIBRACIÓN-RALENTÍ` (`8`) el servo siempre está en
  `SERVO_PULSO_MIN`, no existe un "PID ACTIVO" propio de este modo. En
  `CALIBRACIÓN-SERVO-MIN` (`4`) el servo va en pasos automáticos por
  encima de `SERVO_PULSO_MIN` mientras dura el barrido (ver sección
  11) — es el único modo, aparte de `1`/`2`, donde el servo se mueve
  fuera de esa posición sin ser por el PID.
- `SERVO_PULSO_MIN/MAX` solo se pueden cambiar **por downlink**
  estando en `CALIBRACIÓN-BARRIDO` o `CALIBRACIÓN-MANUAL` (NO en
  `CALIBRACIÓN-RALENTÍ`, `CALIBRACIÓN-SERVO-MIN`,
  `AUTO-ESCALÓN-INTERNO`, `AUTO-ESCALÓN-LOCAL` ni `AUTO-ESCALÓN-REMOTO`);
  fuera de esos dos modos se rechazan con `APPLY_ERROR` (STATUS=4)
  aunque el motor esté apagado. En `CALIBRACIÓN-BARRIDO` el cambio
  tiene efecto inmediato sobre el barrido en curso; en
  `CALIBRACIÓN-MANUAL` mueve el servo directo al valor recién
  configurado. `CALIBRACIÓN-SERVO-MIN` (`4`) es la única forma de que
  `SERVO_PULSO_MIN` cambie sin un downlink explícito de ese parámetro
  — lo hace `main.c` directo, al completar el barrido (ver sección 11).
- `PID_RPM_KP/PID_RPM_KI` (`PID_KD` eliminado 2026-09-23, ver sección 9) solo se pueden cambiar
  con `MODO=CALIBRACIÓN` Y `CALIB=0` (ningún `CALIB` activo
  -- gate actualizado 2026-09-29, antes exigía `CALIB=10`,
  ver sección 4.4); en cualquier otro caso (incluida la operación
  normal, `MODO≠4`, y mientras cualquier `CALIB` esté corriendo) se
  rechazan con `APPLY_ERROR`, motor operando o no — evita que las
  ganancias del PID cambien fuera de una sesión deliberada de
  calibración, o a mitad de una prueba automática que depende de que
  no cambien.

---

## 5. Sensor de presión (motobomba, 4-20mA) — **implementado 2026-09-09, no probado en campo**

Sensor: **PCM300-1M-G-B1-C2-J4** (0-1MPa, salida 4-20mA). Circuito de
acondicionamiento (LM358 actual + rediseño planeado con LM324/LM2902,
alimentación, hallazgos de ruido de campo) → **ver hardware, sección 5**.

`presion.c/h`: dispara una conversión de `ADC1_IN2` por polling en
cada vuelta del loop principal (`Presion_Update()`), sin DMA/IT —
de sobra para una señal tan lenta como un lazo 4-20mA. Convierte
cuentas ADC → voltaje → corriente del lazo (usando la ganancia del
op-amp y el `R_shunt` del circuito, ver hardware) → PSI, con un filtro EMA
(`PRESION_ALPHA_FILTRO=0.2`, mismo criterio que `tacometro.c`) para
suavizar el ruido del ADC. `Presion_SensorValido()` detecta lazo
abierto (<3.5mA, cable cortado/sensor desconectado) o
corto/sobre-rango (>20.5mA) — mientras esté en falla, la presión
filtrada se congela en el último valor válido en vez de contaminarse
con una lectura fuera de rango.

**Calibración mA→PSI confirmada por el fabricante** (tabla de 17
puntos, perfectamente lineal): `4mA = 0 PSI`, `20mA = 145.04 PSI`
(equivale a 0-1MPa, consistente con el rango del sensor). Constantes
en `presion.h` (`PRESION_PSI_EN_CORRIENTE_MIN/MAX`).

Conectado a la telemetría: alimenta `Presion_GetPresionPsi()` en el
uplink LIVE (antes placeholder `0.0f`) y determina el estado `ACTIVO`
(ver sección 4.2).

**Autocalibración de VDDA (2026-09-14)**: en vez de asumir `VREF=3.3V`
fijo, `presion.c` lee el canal interno `VREFINT` cada 5s
(`PRESION_VREFINT_INTERVALO_MS`), calcula el VDDA real con la macro
oficial de ST (`__HAL_ADC_CALC_VREFANALOG_VOLTAGE`) usando la
calibración de fábrica del chip, y lo usa en las conversiones en vez
de un valor fijo — corrige sola una de las dos causas del offset de
~1mA encontrado en pruebas de banco (la otra es tolerancia de
resistencias). Requiere `Vrefint Channel` habilitado en `ADC1` (ya
hecho en el `.ioc`). `Presion_GetVdda()` expone el valor actual para
diagnóstico, visible en el log serial.

### ⚠️ TEMPORAL (2026-09-14): canal de voltaje en paralelo mientras se resuelve el circuito de corriente

El rediseño del circuito de acondicionamiento (sección hardware 5.3)
sigue en debugging — se encontraron y corrigieron dos bugs reales
(polaridad `+`/`-` del diferencial invertida, y `R15` en el nodo
equivocado), y el diferencial (3 op-amps) ya funciona bien, pero el
filtro activo Sallen-Key (`U6D`) todavía falla (colapsa a ~0.06V,
sospecha de oscilación — sin resolver, ver hardware sección 5.3).
Mientras tanto, para no bloquear las pruebas de `MODO`/PID en cascada,
se agregó un **segundo canal de presión, independiente y temporal**:
un sensor de salida en voltaje (GPT203, 0.5-2.5V↔0-150PSI, alimentado
a 3.3V) conectado directo a `PA4`/`ADC2_IN17` — sin necesitar circuito
de acondicionamiento, ver hardware sección 5.6.

- **`presion_voltaje.c/h`** (`PresionV_...`): mismo patrón que
  `presion.c` (filtro EMA, detección de falla) pero para este sensor,
  usando `hadc2` — completamente independiente, no comparte estado con
  el canal de corriente.
- `Presion_Init/Update` (canal de corriente, `hadc1`/`PA1`) **sigue
  corriendo en paralelo sin quitarse**, para no perder el circuito
  cuando se retome.
- **`PresionV_...` es el que alimenta `ESTADO_ACTIVO` y el uplink
  ahora mismo** — 3 puntos marcados `TEMPORAL (2026-09-14)` en
  `main.c` para revertir fácil una vez que el circuito de corriente
  quede terminado.
- El log serial muestra ambos canales lado a lado (`Presion: ... |
  PresionV: ...`) para comparar en vivo.

**Retraso conocido de `PresionV` (hallado 2026-09-29, sin corregir
todavía)**: con el sensor ya bien conectado (GND corregido, lee 0 PSI al
aire, VDDA 3.31 V), la lectura en PSI tarda bastante en alcanzar al
voltaje cuando la presión cambia rápido. Causa, confirmada en
`COM3_2026_09_29.18.57.30.564.txt`: en `ProcesarTanda()` el limitador de
velocidad (`PRESIONV_TASA_MAX_CAMBIO_PSI_S = 5.0`) recorta la muestra
**antes** del EMA (`PRESIONV_ALPHA_FILTRO = 0.08`), y el EMA la vuelve a
suavizar, así que la velocidad máxima real de cambio es
`5.0 × 0.08 = 0.4 PSI/s`, no 5 PSI/s. En el log un salto de voltaje a
~2.4 V (más de 100 PSI reales) se ve como una rampa perfectamente
lineal de +2.0 PSI cada 5 s (0.74 → 2.74 → 4.74 → 6.74 …), y al bajar
igual (−2.0 PSI cada 5 s).

- Cambios lentos (dinámica hidráulica real, décimas de PSI por segundo)
  **no se ven afectados** — el limitador solo actúa en saltos grandes.
- Efecto en operación: un cambio brusco real de presión (apertura de
  válvula, arranque/paro, purga) tarda minutos en reflejarse; ej. 100 PSI
  ≈ 250 s. Como `PresionV_GetPresionPsi()` alimenta el lazo en cascada
  (`MODO=1`), el uplink y `ESTADO_ACTIVO`, todos ven ese retraso.
- Aparte hay un retraso fijo menor: una lectura nueva solo sale cada
  tanda de `PRESIONV_MUESTRAS_POR_PROMEDIO = 21` conversiones, y luego
  el EMA α=0.08 (~12 tandas para llegar al 63 % de un escalón).
- Opciones si se decide corregir (ninguna aplicada): (a) aplicar el
  limitador a la salida ya filtrada en vez de a la entrada del EMA
  (así el techo real sería los 5 PSI/s); (b) subir
  `PRESIONV_TASA_MAX_CAMBIO_PSI_S` a ~`5.0 / 0.08 ≈ 60` para conservar
  el techo efectivo de 5 PSI/s; (c) subir `PRESIONV_ALPHA_FILTRO`. Ojo:
  el limitador se agregó a propósito el 2026-09-22 para rechazar ráfagas
  de ruido del RAK3172 (~1.4 PSI de un instante a otro), y el EMA a 0.08
  para rechazar ruido en el lazo externo — revisar ambos motivos antes
  de aflojarlos.

---

## 6. Integración con AWS IoT Core for LoRaWAN

- **Dispositivo**: `DSL-0001`, Device Profile `RIO-DSL-AU915` (AU915 sub-banda 2, Clase
  C, LoRaWAN 1.0.3; migrado desde US915 el 2026-09-29 — el wireless device
  se recreó en AWS, su WirelessDeviceId cambió, DevEUI/AppEUI/AppKey no), Service Profile `RIO` (compartido con el nodo
  aspersor del equipo).
- **Regla de IoT**: `RIO_DSL_Lambda`, tópico de entrada `RIO/DSL`.
- **Lambda de uplinks**: `RIO-DSL-DecodeUplink` (Python 3.12,
  `lambda_function.py` + `decoder.py`).
- **Lambda de downlinks**: `RIO-DSL-SendDownlink`
  (`send_downlink.py` + `encoder.py`), dispara
  `iotwireless:SendDataToWirelessDevice` a partir de un mensaje MQTT
  con el nombre del parámetro (ej. `{"SET_RPM": 450}`), sin exponer
  el ID numérico al usuario.

### Migración de banda US915 → AU915 (2026-09-29)

Gateway RAK2287 (Basic Station) en AU915 sub-banda 2 y AWS ya migrados.
Los wireless devices se borraron y recrearon con las mismas llaves
(DevEUI/AppEUI/AppKey), por lo que su `WirelessDeviceId` cambió (AWS no
permite cambiar la región de un device). Perfiles nuevos:
`RIO-DSL-AU915` y `RIO-ASP-AU915` (MAC 1.0.3, RP002-1.0.1).

**Sensor (RAK3172, RUI3).** La región, la clase y el DR **no están en el
firmware del STM32**: viven en la memoria del módulo y se configuran una
sola vez por consola serial (reflashear el STM32 no las cambia):

```
AT+NJM=1
AT+BAND=6        (AU915)
AT+MASK=0002     (sub-banda 2: canales 8-15 + 65, 916.8-918.2 MHz)
AT+CLASS=C       (DSL-0001; el aspersor ASP-0001 usa AT+CLASS=A)
ATZ
```

Antes de cambiar, guardar el estado con `AT+BAND=?`, `AT+MASK=?`,
`AT+CLASS=?`, `AT+DEVEUI=?`, `AT+APPEUI=?`, `AT+APPKEY=?`. No forzar
RX2 ni DR (los defaults de AU915 son correctos; la numeración de DR
cambia respecto a US915). El firmware sigue mandando `AT+MASK=0002` en el
arranque y antes de cada reintento de join — misma sintaxis en AU915. El
uplink de 28 bytes cabe desde DR0 (51 bytes máx. en DR0-2; en US915 DR0
solo admitía 11).

**Lambdas.** Revisadas, sin cambios necesarios:
- `RIO-DSL-DecodeUplink` solo lee `Fport` y `PayloadData`; no depende de
  banda, frecuencia, DR ni ID del device.
- `RIO-DSL-SendDownlink` no tiene UUIDs fijos: resuelve el device por
  `Name` (`DSL-0001`) con `list_wireless_devices`, así que toma el ID
  nuevo solo — **siempre que el device recreado se llame exactamente
  `DSL-0001`**. Publicar en `RIO/DSL-0001/downlink`, no con el UUID viejo.
- Verificar en AWS (fuera del código): que la política IAM del rol de
  SendDownlink no limite `iotwireless:SendDataToWirelessDevice` al ARN del
  device viejo, y que la regla/destino sigan apuntando a la Lambda correcta.

**Verificación.** `+EVT:JOINED` y uplinks sin error en el log serial;
tráfico entre 916.8 y 918.2 MHz en el gateway; eventos en CloudWatch con
el ID nuevo; downlink de prueba a DSL-0001 (Clase C) con ACK en FPort 3.
**Estado: DSL-0001 ya conectó en AU915** (2026-09-29).

### Forma real del evento que recibe la Lambda de uplinks (confirmada con tráfico real)

```json
{ "PayloadData": "<base64>", "Fport": 1 }
```

⚠️ **No** viene anidado bajo `WirelessMetadata.LoRaWAN`, y **no**
incluye `DevEui` en absoluto (solo `FCnt` y `FPort`) — confirmado
capturando un mensaje real en el cliente de prueba MQTT, no asumido de
la documentación.

### Consulta SQL de la regla (uplinks)

```sql
SELECT VALUE
  aws_lambda(
    "arn:aws:lambda:us-east-1:271551172366:function:RIO-DSL-DecodeUplink",
    { "PayloadData": PayloadData, "Fport": WirelessMetadata.LoRaWAN.FPort }
  )
FROM 'RIO/DSL'
```

`SELECT VALUE` (en vez de listar campos con `as`) hace que el
resultado de la Lambda salga **plano** en el tópico de salida, sin
envolverlo en una propiedad extra.

Acción: **Republish** a `RIO/DSL/DECODED`, rol IAM `GIO_role`
(compartido con la regla del nodo aspersor).

### Layout de bytes del uplink LIVE (FPort 1, 28 bytes)

Debe coincidir byte a byte entre `RAK3172_EnviarUplinkLive()`
(firmware) y `decoder.py::_decodificar_live()` (Lambda):

| Offset | Bytes | Campo | Formato |
|---|---|---|---|
| 0–1 | 2 | `motorIdNumeric` | uint16 BE (ej. 1 → "DSL-0001") |
| 2–3 | 2 | `rpm` | uint16 BE, x10 |
| 4–5 | 2 | `presion` (salida del motor) | uint16 BE, x10 — placeholder 0 hasta `presion.c` |
| 6 | 1 | `estado` | uint8 — 1=ACTIVO, 2=APAGADO, 3=ENCENDIDO |
| 7–10 | 4 | `fecha_hora` (unix, ya con -6h aplicado en el STM32) | uint32 BE |
| 11–14 | 4 | `inicio_operacion` (unix, ya local) | uint32 BE |
| 15–18 | 4 | `segundos_transcurridos` | uint32 BE |
| 19–22 | 4 | `latitud` | int32 BE, x10,000,000 |
| 23–26 | 4 | `longitud` | int32 BE, x10,000,000 |
| 27 | 1 | `codigoAlerta` | uint8 — ver tabla de alertas abajo |

`latitud`/`longitud` vienen del GPS en vivo cuando hay fix, con
respaldo a la última posición conocida persistida en flash, y a las
coordenadas fijas `14.27387764641955, -91.09260397779028` como último
recurso si el GPS nunca ha conseguido fix — ver prioridad completa en
2.6.

⚠️ **Subido de 27 a 28 bytes el 2026-09-23** al agregar `codigoAlerta`.
Antes de esto, las alertas del sistema (`ALERTA,PRESION_OBJETIVO_NO_ALCANZADA`,
`ALERTA,MODO_REMOTO_OBJETIVO_NO_ALCANZADO`, sensor de presión en falla,
watchdog de señal en `MODO=2`) solo se logueaban por monitor serial —
en campo nadie se enteraba salvo que alguien estuviera conectado
físicamente al equipo en ese momento.

**Tabla de alertas** (`CodigoAlerta_t` en `main.c` / `NOMBRES_ALERTA`
en `decoder.py`):

| Código | Nombre | Origen |
|---|---|---|
| 0 | `SIN_ALERTA` | — |
| 1 | `PRESION_OBJETIVO_NO_ALCANZADA` | Supervisor `MODO=1` (sección 8) |
| 2 | `MODO_REMOTO_OBJETIVO_NO_ALCANZADO` | Supervisor `MODO=2`, escenario de fuga (sección 4.3) |
| 3 | `SENSOR_PRESION_LOCAL_FALLA` | `MODO=1`, `PresionV_SensorValido()` inválido |
| 4 | `MODO_REMOTO_SIN_PRESION_VALIDA` | Watchdog `TIMEOUT_SIN_COMANDO_S` en `MODO=2` |

`codigoAlerta` es un **snapshot del estado actual** (viaja en cada
uplink LIVE, periódico o forzado, mientras la alerta siga activa), no
un evento puntual. Para saber **cuándo** ocurrió la transición (no solo
que está activa), cada punto que cambia la alerta llama a
`CalibFlash_ForzarReporte()` — eso dispara un uplink LIVE inmediato (en
vez de esperar al próximo intervalo periódico), y el `fecha_hora` de
**ese** uplink puntual es el timestamp real del evento. Se limpia
(`SIN_ALERTA`) tanto en la recuperación natural de cada condición como
al reingresar a `MODO=1`/`2` (arranque fresco — importante para la
alerta de fuga en particular, que fuerza `MODO=RALENTI` de verdad: la
alerta queda visible hasta que el operador reenvía `MODO=2`, momento
en el que se limpia sola).

### Formato de salida de la Lambda

```python
# LIVE (FPort 1) -- registro plano estilo GIOSoftware:
{
  "fecha_hora": "2026-08-14 14:45:41",
  "codigo_maquinaria": "DSL-0001",
  "tipo": "Motor Diesel",
  "area": "Riegos",
  "presion": 88.5,
  "nombre_operacion": "ACTIVO",
  "codigo_operacion": 1,
  "clasificacion_operacion": "Productivo",
  "inicio_operacion": "2026-08-14T14:44:31",
  "tiempo_transcurrido": "0 dias, 00:01:10",
  "segundos_transcurridos": 70,
  "alerta": "SIN_ALERTA",  # agregado 2026-09-23, ver NOMBRES_ALERTA
  "longitud": -91.092375,
  "latitud": 14.2740023
}

# ACK (FPort 3):
{"parameter": "SET_RATIO", "status": "OK", "valorAplicado": 17.5}
```

⚠️ **SUPUESTOS pendientes de confirmar contra lo que espera
GIOSoftware** (marcados en `decoder.py`): `tipo`/`area` fijos como
`"Motor Diesel"`/`"Riegos"`; `codigo_operacion` reutiliza 1=ACTIVO y
2=APAGADO (mismos números que el nodo aspersor, para consistencia
entre tipos de máquina) y asigna 5 a `ENCENDIDO` (estado nuevo, sin
equivalente en el aspersor).

El `valorAplicado` del ACK ya viene **convertido a valor real** (no
crudo) — la Lambda aplica la escala correcta según el parámetro
(`ESCALA_PARAMETRO` en `decoder.py`, debe coincidir exactamente con la
escala del firmware).

---

## 7. Cómo probar un downlink de configuración manualmente

1. Calcular el payload (Python):
   ```python
   import base64
   id_byte = 1  # SET_RATIO
   valor_escalado = round(17.5 * 100)  # 1750
   payload = bytes([id_byte, (valor_escalado >> 8) & 0xFF, valor_escalado & 0xFF])
   print(base64.b64encode(payload).decode())  # AQbW
   ```
2. AWS IoT Core → Conectividad inalámbrica → Dispositivos → `DSL-0001`
   → pestaña "Cola" → encolar con `FPort=2`, payload en base64.
   (O, con `RIO-DSL-SendDownlink` ya desplegado, simplemente publicar
   `{"SET_RATIO": 17.5}` en el tópico de downlink correspondiente.)
3. Confirmar en el monitor serie del G431 (`Downlink ID=... STATUS=...`)
   y en el cliente MQTT (`RIO/DSL/DECODED`).

**Sin red LoRa disponible** (ej. en banco, o en campo antes de tener
cobertura): usar el mando manual por serial en su lugar, escribiendo
directo en el mismo monitor de depuración — ver `comando_serial.c/h`,
sección 2.7. Mismo nombre y mismo valor humano que en el paso 2, ej.
escribir `SET_RATIO 17.5` y Enter.

---

## 8. Pendientes generales

- [ ] **Segundo timeout, más largo, para `MODO=2` tras señal caída por
      mucho tiempo — esperando confirmación del cliente, no
      implementado (2026-09-23).** Hoy, si se cae la señal remota, el
      motor se COMPORTA como ralentí (`setpointRpmCrudo=0`, ver watchdog
      de `TIMEOUT_SIN_COMANDO_S` sección 4.3) pero `MODO` nunca cambia
      de verdad -- en cuanto vuelve un downlink válido de `PRESION_REMOTO`,
      retoma control automático de inmediato, sin que nadie tenga que
      reconfirmar nada. Preocupación real planteada por el usuario: si
      la caída dura mucho (ejemplo dado: ~1 hora), el motor detenido
      todo ese tiempo puede dejar que la tubería se vacíe de agua (aire
      adentro) -- retomar control automático apenas vuelve la señal,
      sin que un operador haya confirmado que la tubería sigue en
      condiciones, es el mismo riesgo que ya se resolvió para el
      arranque del equipo (`MODO` nunca arranca solo en `1`/`2`, sección
      4.3) y para el intento final del supervisor de fuga de arriba.
      **Diseño propuesto, aún no construido**: un segundo timeout, bien
      más largo que `TIMEOUT_SIN_COMANDO_S` (360s), que si se supera con
      la señal caída, fuerce `MODO` a `RALENTI` **de verdad** (mismo
      mecanismo que el intento 4 del supervisor de fuga) -- así, al
      volver la señal, el motor se queda en ralentí real esperando que
      un operador mande `MODO 2` explícito de nuevo, en vez de
      reanudar solo. **Falta confirmar con el cliente cuál es el umbral
      real razonable** (el usuario tiró "una hora" como ejemplo, no
      confirmado) antes de implementarlo.
- [ ] **Terminar el circuito de acondicionamiento del lazo 4-20mA
      (pausado 2026-09-14).** El diferencial (3 op-amps) ya funciona
      bien (validado contra el amperímetro de la fuente de banco); el
      filtro activo Sallen-Key (`U6D`) sigue fallando (colapsa a
      ~0.06V, sospecha de oscilación por los capacitores de 22nF sin
      carga en esa topología con LM324 — sin confirmar con el mismo
      método nodo-por-nodo que resolvió los otros dos bugs). Mientras
      tanto, `main.c` usa el canal de voltaje (`PresionV_...`, ver
      sección 5) como fuente temporal — revertir los 3 puntos
      marcados `TEMPORAL (2026-09-14)` una vez que esto quede resuelto,
      o decidir si conviene quedarse con ambos canales de forma
      permanente (uno como respaldo del otro).
- [x] **Calibración de campo del sensor de presión -- DESCARTADA
      (planteada 2026-09-10, bosquejo de diseño 2026-09-18, decisión
      final 2026-09-18: NO implementar).** Se llegó a diseñar (mismo
      patrón que `SET_RATIO_AUTO`: un parámetro downlink que recibe una
      referencia real y el firmware calcula/persiste un offset en
      flash) y hasta se empezó a construir (IDs 30/31 en
      `calibracion_flash.h`), pero se revirtió antes de tocar el `.c`
      al analizar la magnitud real del error: el GPT203 especifica
      **Accuracy 0.5% FS** (±0.75 PSI sobre el rango de 150 PSI) --
      un margen de fábrica pequeño que no justifica el esfuerzo de
      calibrar (ni por downlink, ni con una constante medida a mano).
      El offset de 2.41 PSI medido el 2026-09-14 (mayor que ese 0.75
      PSI de spec) resultó ser, con alta probabilidad, ruido de esa
      medición puntual (fue antes de agregar el promedio recortado a
      `presion_voltaje.c`, sección 5), no un desfase real de la unidad
      -- por eso también se revirtió `PRESIONV_OFFSET_V` a `0.0f`
      (ver comentario en `presion_voltaje.h`). Mismo criterio que ya
      usa el compañero en su repo `Sensor-de-Presion`
      (`VOLTAGE_OFFSET_V = 0.0F` en su `config.h`, sin corrección,
      exacta misma razón). Si en el futuro se sospecha un offset real
      (ej. al reemplazar el sensor por una unidad claramente fuera de
      spec), volver a medir con el sensor al aire y con el filtro de
      promedio recortado ya activo antes de concluir que el offset es
      real -- y en ese caso, corregirlo con una constante medida a
      mano alcanza, no hace falta reabrir el diseño de downlink/flash.
- [ ] **Horómetro del motor — no diseñado, solo anotado como pendiente
      (2026-09-10).**
- [ ] **LED físico de diagnóstico de alertas — no diseñado, solo anotado
      como pendiente (2026-09-23).** Idea del usuario: usar la misma
      variable `codigoAlerta`/`s_codigoAlertaActual` que ya alimenta el
      byte 27 del uplink LIVE (ver "Layout de bytes del uplink LIVE" más
      abajo) para manejar un LED local -- apagado si `SIN_ALERTA` (`0`),
      encendido/parpadeando si hay alguna alerta activa, posiblemente con
      patrones de parpadeo distintos por código para diagnosticar sin
      necesidad de leer el log serial ni esperar a LoRa. Pendiente
      definir: pin/hardware a usar, si parpadea distinto por código o
      solo enciende fijo, y si necesita algún driver (la salida directa
      de un GPIO alcanza para un LED simple, sin transistor, si la
      corriente está dentro de lo que tolera el pin del G431).
- [x] **Supervisor de "no se llega a `PRESION_OBJETIVO_LOCAL`" — implementado
      2026-09-14 para `MODO=1` (planteado originalmente 2026-09-09 para
      `MODO=2`, ver historial abajo).** No es una protección nueva
      contra sobre-acelerar el motor — esa ya existe sola, ver
      siguiente párrafo — es solo una capa de DETECCIÓN/ALERTA. Diseño
      final, resolviendo las 5 preguntas abiertas que había dejado la
      versión original de este pendiente:
      1. Se monta ENCIMA del lazo externo (`presion_pid.c`), no lo
         reemplaza — el lazo sigue calculando su setpoint normalmente
         (recortado a `[0,RPM_MAX]` por sí solo); el supervisor solo
         observa si ese setpoint queda pegado en `RPM_MAX` sin que la
         presión real se acerque al objetivo.
      2. **No desacelera/reacelera activamente** (se simplificó frente
         a la idea original) — el lazo PID en cascada ya reacciona solo
         a cualquier cambio de presión, no hace falta forzar un ciclo
         manual. Un "intento fallido" es una ventana de
         `PRESION_SUPERVISOR_VENTANA_MS` (**30s**) sostenida con el
         lazo saturado en `RPM_MAX` y la presión fuera de tolerancia.
      3. Tolerancia: **±5%** de `PRESION_OBJETIVO_LOCAL`
         (`PRESION_SUPERVISOR_TOLERANCIA_FRACCION`).
      4. "3 veces" = 3 ventanas de 30s seguidas en esa condición
         (`PRESION_SUPERVISOR_INTENTOS_MAX`) — si la presión entra en
         tolerancia en cualquier momento, el contador se reinicia a 0.
      5. ~~Por ahora, solo monitor serial~~ **RESUELTO 2026-09-23**: el
         uplink LIVE subió a 28 bytes (`codigoAlerta`, byte 27) y esta
         alerta (código `1`, `PRESION_OBJETIVO_NO_ALCANZADA`) ya se
         manda por LoRa en cada uplink mientras siga activa — ver
         sección "Layout de bytes del uplink LIVE" más abajo. No queda
         "trabado" — sigue evaluando cada ventana y avisa
         (`ALERTA,PRESION_OBJETIVO_RECUPERADA`, código `0`) en cuanto la
         presión vuelve a tolerancia, sin esperar ningún downlink de
         reset.
      Implementado en `main.c` (bloque junto a la rama `MODO=1`, ver
      sección 4.3/12). Se ignora mientras el sensor de presión está en
      falla (esa condición ya tiene su propia alerta separada, ver
      arriba). **Pendiente real**: solo compile-verificado, falta
      probarlo con el sistema hidráulico real (mismo pendiente que el
      resto de la cascada); y decidir si este mismo supervisor también
      debería aplicarse a `MODO=2` (la idea original era para ese modo)
      una vez que `MODO=1` esté validado en campo.
- [x] Lazo PID (`pid.c/h`) implementado y activo en `main.c` (motor
      operando, sin calibración) — ver secciones 2.4 y 4.4.
- [ ] Ganancias reales `PID_RPM_KP/KI` — siguen en default (`1.0/0`),
      pendientes de sintonizar en el motor real. Ver sección 9 para el
      procedimiento (Ziegler-Nichols en lazo cerrado, sin MATLAB) — solo
      se pueden tocar con `MODO=CALIBRACIÓN` + `CALIB=0`
      (ver sección 4.4, gate actualizado 2026-09-29).
- [x] **AWS**: `send_downlink.py`'s `PARAMETER_TABLE["CALIB"]["max"]`
      subido de `2` a `3` para aceptar el nuevo modo de sintonización de
      PID (mismo ajuste que ya se había hecho cuando se agregó el valor
      `2`).
- [ ] **AWS, pendientes acumulados (ir agregando acá cada vez que el
      firmware agregue/cambie algo del lado de `send_downlink.py`, para
      no perder el hilo entre sesiones)**:
      - `PARAMETER_TABLE["CALIB"]["max"]` subir a `10`
        (quedó en `3` la última vez que se confirmó hecho — cubre de
        una vez los modos `3` a `10` agregados/renumerados después, ver
        secciones 4.4/9/10/11/12; incluye el modo `6` -- auto-escalón
        del lazo interno -- agregado en la segunda renumeración del
        2026-09-23, que no existía la última vez que se tocó este
        archivo).
      - `PARAMETER_TABLE` necesita una entrada nueva para
        `SET_RATIO_AUTO` (ID `25`, escala `x10`, sin la cual ese
        parámetro solo funciona por serial, no por downlink LoRa —
        ver sección 12).
      - `ALPHA_AUTO` NO necesita entrada propia — quedó como
        `CALIB=3`, cubierto por el bump de arriba.
      - **Nuevo 2026-09-11**: IDs `26-29` (antes `MECANISMO_*`) quedaron
        libres al quitarse el mecanismo biela-manivela — si ya se
        habían agregado a `PARAMETER_TABLE`, quitarlos de ahí también.
        `PRESION_GANANCIA_RPM`/`PRESION_OFFSET_RPM` se corrieron de
        `30`/`31` a `26`/`27` para ocupar ese hueco — si ya estaban en
        `PARAMETER_TABLE` con `30`/`31`, actualizar los IDs, no solo
        agregar entradas nuevas.
      - `PARAMETER_TABLE` necesita 2 entradas para
        `PRESION_GANANCIA_RPM` (ID `26`, escala x100, con
        signo) y `PRESION_OFFSET_RPM` (ID `27`, escala x10) — sin ellas
        `MODO=2` no se puede calibrar por downlink LoRa, solo por
        serial. Ver sección 4.3.
      - **Nuevo 2026-09-14**: IDs `28-29` (que habían quedado libres el
        2026-09-11) se reasignaron a `PID_PSI_KP` (ID `28`, escala
        x100, con signo) y `PID_PSI_KI` (ID `29`, escala x1000, con
        signo) — las ganancias del lazo EXTERNO de presión local de
        `MODO=1` (sección 4.3, `presion_pid.c`). `PARAMETER_TABLE`
        necesita estas 2 entradas nuevas, sin ellas `MODO=1` no se
        puede calibrar por downlink LoRa, solo por serial.
      - **Nuevo 2026-09-14**: `MODO` se renumeró (ver sección 4.3) —
        el valor `1` pasó de "SET_RPM directo" a "presión local", y
        "SET_RPM directo" se corrió al valor `3`. Si `send_downlink.py`
        o cualquier Lambda tiene alguna validación/documentación propia
        de los valores válidos de `MODO` (más allá de solo aceptar
        0-3), revisar que no siga asumiendo el significado viejo de `1`.
      - [x] **`decoder.py` (lado de las ACKs) — corregido y entregado
        2026-09-17.** No tenía los IDs `25-29` en `NOMBRES_PARAMETRO`/
        `ESCALA_PARAMETRO`/`PARAMETROS_CON_SIGNO` — sus ACKs salían como
        `"DESCONOCIDO(N)"` con el valor crudo sin escalar. Ojo con `25`
        (`SET_RATIO_AUTO`): el downlink que se manda es RPM×10, pero el
        ACK que se devuelve es el ratio resultante ×100 (mismo patrón
        que `SET_RATIO`) — la escala agregada es para decodificar el
        ACK, no la entrada.
      - [x] **`send_downlink.py`/`PARAMETER_TABLE` (dirección downlink)
        — corregido y entregado 2026-09-17.** Tenía 3 problemas reales
        (bloqueaban funcionalidad, no solo imprecisión de rango):
        `CALIB["max"]` en `3` (subido a `7`), `MODO["max"]`
        en `2` (subido a `3`), y faltaban por completo las entradas para
        `25-29` (`ValueError: nombre de parametro desconocido` al
        intentar mandarlos). De paso se ajustaron los rangos min/max de
        varios parámetros existentes para que coincidan exacto con la
        validación real del firmware (antes más permisivos — no rompían
        nada, pero el Lambda tardaba un downlink completo en avisar del
        error en vez de fallar de una vez). `RPM_MAX/MIN` y
        `SERVO_PULSO_MIN/MAX` quedan con una nota: el firmware valida
        esos dos pares de forma RELACIONAL (uno contra el otro, contra
        el valor ya guardado en el nodo) — una tabla estática no puede
        replicar eso, sigue siendo el firmware el validador real ahí.
      - ⚠️ **Ninguno de estos dos archivos vive en este repositorio** —
        se entregaron como archivos sueltos en la conversación (no hay
        acceso al repo Lambda-RIO desde esta sesión); hay que copiarlos
        manualmente al repo/consola de AWS.
      - **Aviso importante, no es un cambio de tabla**: `MODO` (ID 16)
        ahora sí tiene efecto real en el firmware (antes era un no-op
        aceptado y ACKeado sin hacer nada) — si alguna Lambda/servicio
        ya manda `MODO=2` + `PRESION_REMOTO` esperando que el motor reaccione,
        confirmar primero que `PRESION_GANANCIA_RPM`/`PRESION_OFFSET_RPM`
        ya se calibraron en ese nodo (ver sección 4.3) y que el nodo no
        se quedó en `MODO=0` tras la actualización (ver aviso de
        migración en esa misma sección) — si no, `MODO=2` no va a mover
        el motor aunque el downlink de `PRESION_REMOTO` llegue y se ACKee OK.
      - **Pendiente de diseño (no de AWS), agregado 2026-09-14**: switch
        físico selector de `MODO` en el gabinete, prioridad de control
        local (serial) sobre remoto (downlink, estilo variador ABB por
        Modbus), y reglas de transición entre valores de `MODO` (hoy
        `CalibFlash_SetModo()` acepta cualquier salto directo, sin
        restricción de secuencia) — ninguno tiene código todavía, ver
        sección 4.3 para el detalle de cada uno.
- [ ] Posible parámetro 25, `INTEGRAL_MAX` (anti-windup configurable
      del PID) — hoy el anti-windup es fijo en `pid.c` (integración
      condicional, sin límite configurable por downlink).
- [x] **Puente aspersor → motor (`MODO`/`PRESION_REMOTO`/`TIMEOUT_SIN_COMANDO_S`)
      — implementado 2026-09-07** (encontrado al revisar qué pasaría si
      otra Lambda empezara a mandar `PRESION_REMOTO` como downlink automático
      desde los aspersores; ver sección 4.3 para el detalle completo):
      `MODO` ahora rama el ciclo de control real (antes era un no-op);
      `MODO=2` deriva el setpoint de `PRESION_REMOTO` vía una fórmula lineal
      calibrable en campo (`PRESION_GANANCIA_RPM`/`PRESION_OFFSET_RPM`,
      IDs 30/31, mismo gate `CALIB=7` que `PID_RPM_KP/KI/KD`);
      el enum `CalibFlash_Modo_t` se renombró a
      `CALIB_MODO_RALENTI/LOCAL/REMOTO` para coincidir con el README
      (⚠️ desactualizado — renombrado de nuevo 2026-09-14 a
      `CALIB_MODO_RALENTI/PRESION_LOCAL/REMOTO/MANUAL_BANCO`, ver
      sección 4.3); y
      el watchdog de `TIMEOUT_SIN_COMANDO_S` fuerza ralentí si `MODO=2`
      deja de recibir `PRESION_REMOTO` fresca. **Pendientes reales que quedan**
      (no de diseño, de calibración/AWS): `PRESION_GANANCIA_RPM`/
      `PRESION_OFFSET_RPM` siguen en `0` (sin calibrar) hasta que se
      corra una sesión real con el motor y el sistema hidráulico
      completo; `PARAMETER_TABLE` de `send_downlink.py` necesita las
      entradas para IDs 30/31 (ver checklist de AWS arriba); y el día
      que exista `presion.c` (sección 5), decidir si `MODO=1` debería
      pasar a usar esa lectura propia en vez de `SET_RPM` (ver aviso en
      sección 4.3).
- [x] Construcción física del circuito de presión (LM358) y `presion.c/h`
      — implementado 2026-09-09, compilado limpio (0 errores/0
      warnings), ver sección 5. **Pendiente real que queda**: prueba en
      campo con el sensor real conectado al lazo 4-20mA (hoy solo
      compile-verificado), y ajustar `PRESION_UMBRAL_ACTIVO_PSI`
      (`main.c`, hoy `5.0` PSI sin dato de campo) una vez que se tenga
      una sesión real con la motobomba cargada — ver sección 4.2.
- [ ] El uplink LIVE ahora manda la presión real (PSI) en vez del
      placeholder `0.0f` fijo — confirmar que `decoder.py` (AWS, fuera
      de este repo) no tenga ninguna lógica que asumiera ese `0.0`
      constante (ej. algún cálculo o validación que nunca se probó con
      un valor distinto de cero).
- [ ] `RESET_REMOTO` (ID 22) — recibido y validado, pero
      deliberadamente **no conectado** a ninguna acción real hasta
      definir qué hace el servo durante un reinicio (mantener última
      posición vs. quedar sin control unos segundos).
- [ ] Valores reales (no placeholder) de `RPM_MAX/MIN`,
      `TIMEOUT_SIN_COMANDO_S`, `HISTERESIS_MODO_S` — pendientes de
      definir con datos del motor/cliente real.
- [ ] Confirmar con el equipo si los IDs de parámetro son un espacio
      compartido entre tipos de nodo, o independientes por tipo.
- [ ] **Idea planteada 2026-09-03, no diseñada aún**: un mecanismo para
      que el backend pueda *pedir* (no solo recibir el ACK del momento
      en que se aplicó) los valores vigentes de los parámetros de
      calibración de un nodo — útil para auditar/resincronizar la base
      de datos si se sospecha que un ACK se perdió. Hoy el único camino
      es el Application ACK de cada downlink individual (FPort 3, en el
      momento en que se manda ese parámetro) — el uplink estándar
      (FPort 1, sección 6) NO incluye `SET_RATIO`/`ALPHA`/etc., solo
      telemetría operativa. No confundir con `FORZAR_REPORTE`
      (dispara ese mismo uplink estándar, no un volcado de
      configuración).
- [ ] Verificar si `AT+TIMEREQ=1` requiere mandarse después del primer
      uplink exitoso (no solo después del join) — reporte de la
      comunidad de RAK sugiere que puede fallar si se manda demasiado
      pronto.
- [ ] `MOTOR_ID_NUMERIC` está hardcodeado a `1` en `main.c` — cuando
      exista más de un nodo motor, resolverlo desde
      `CalibFlash_GetNodeId()` (ID 18, `NODE_ID`) en vez de una
      constante.
- [ ] GPS (sección 2.6) — funcionando en campo, con fix real
      confirmado. Pendiente: confirmar si `AT+CGPS=1` devolviendo
      `ERROR` ocasionalmente (ej. si el GPS ya estaba encendido de un
      arranque anterior) necesita manejarse distinto, o si es
      inofensivo como parece hasta ahora.
- [ ] **Reinicios espontáneos del RAK3172 en campo (2026-08-29)** — ver
      sección 2.2. El módulo se reinicia solo (banner de arranque
      reaparece en medio de la sesión), a veces en ciclo continuo.
      Sospecha actual: conexión de antena o alimentación (picos de
      corriente de TX de LoRa), no software — el firmware ya detecta y
      reintenta el join correctamente cuando pasa, pero la causa raíz
      del reinicio en sí sigue sin confirmarse. Revisar cable/conector
      de antena y estabilidad del rail de alimentación del RAK3172
      durante una transmisión (osciloscopio o multímetro en modo
      min/max), y considerar más capacitancia de desacople local.
- [ ] Decidir si `AutoJoin` en `RAK3172_Join()` (`AT+JOIN=1:0:10:8`)
      vuelve a `1` para producción, o se queda en `0` (prueba temporal
      actual, ver 2.2) manejando todos los reintentos desde el host.
- [ ] Quitar los prints de `DIAGNOSTICO TEMPORAL` en `rak3172.c`
      (`RAK3172_ProcesarLinea()` imprimiendo toda línea cruda, y
      `RAK3172_ErrorCallback()`) y `gps.c` (`GPS_ErrorCallback()`) una
      vez confirmada en campo la causa raíz de los reinicios del
      RAK3172 — agregados el 2026-08-29 solo para diagnóstico, ver 2.2.

---

## 9. Sintonización del PID en el motor real (sin MATLAB)

No se usa MATLAB para este proyecto — la sintonización se hace
empírica, directo en el motor real, con los mismos mandos que ya
existen (downlink MQTT o el mando manual por serial de la sección
2.7). No hace falta ningún software externo para lograrlo, aunque un
script de Python puede ayudar a analizar los datos (ver más abajo).

### Por qué hace falta sintonizar (y no solo probar `Kp=1` a ojo)

El PID no "sabe" nada del motor — solo reacciona al error entre
`SET_RPM` y la RPM medida. Las ganancias determinan qué tan agresiva
es esa reacción:

- **`Kp` muy alto**: el servo sobre-corrige, la RPM se pasa del
  setpoint, vuelve a corregir de más en la otra dirección → oscilación
  (el motor "bombea" RPM arriba y abajo). Además de ser inútil para
  operación real, es estrés mecánico repetido sobre la varilla del
  acelerador.
- **`Kp` muy bajo**: reacciona lento y se queda con bastante error de
  estado estable (lo que ya viste con el default `Kp=1.0` en modo
  puramente proporcional).
- **`Ki`**: elimina el error de estado estable que deja un `Kp` por sí
  solo, pero mal puesto agrega sobre-impulso (overshoot) y puede
  tardar en "soltar" la corrección (aunque ya hay anti-windup por
  integración condicional en `pid.c`, ver sección 2.4).
- **`Kd`**: amortigua el sobre-impulso que puede dejar `Ki`, pero
  reacciona a qué tan rápido *cambia* el error — con una medición de
  RPM que ya de por sí "caza" en ralentí (ver la conversación sobre la
  aguja/ralentí inestable, sección 2.1), un `Kd` puesto a la ligera
  amplifica ese ruido en vez de suavizar el control, y el servo se
  pone nervioso. Si se usa, hacerlo con `Kd` pequeño y considerar bajar
  `ALPHA` primero para llegar con una RPM ya más filtrada.

El objetivo de "simular" (en MATLAB, Python, o cualquier otra
herramienta) es siempre el mismo: **probar combinaciones de ganancias
sin gastar tiempo de motor real ni arriesgar una oscilación fuerte en
la varilla física** — se prueba en una computadora primero, y solo se
valida en el motor la combinación que ya se ve razonable en papel. Acá
se salta ese paso intermedio y se sintoniza directo en el motor, con
más cuidado y de forma incremental.

### Restricción importante del firmware: no hay "lazo abierto" con el motor operando

`CALIB=1`/`=2` (manual/barrido, sección 2.4) — que sí
permitirían mandar un pulso fijo al servo sin que el PID reaccione —
**requieren el motor detenido para entrar y para permanecer**, y si el
motor arranca estando en cualquiera de esos dos modos, el firmware los
apaga solo de inmediato (medida de seguridad, ver 4.4). El viejo modo
dedicado de sintonización de PID (numeración anterior a 2026-09-29,
ELIMINADO, ver sección 4.4) no tenía esa restricción — de hecho solo
tenía sentido con el motor ya operando — pero tampoco daba un lazo
abierto: el servo lo seguía manejando el PID igual que en `=0`. Hoy
`PID_RPM_KP/KI` se tocan directo en `CALIB=0` (con `MODO=CALIBRACIÓN`), sin
ese modo dedicado, pero la conclusión es la misma. Esto es
intencional y no se debe evadir: significa que **no se puede hacer una
prueba de "curva de reacción" en lazo abierto** (mandar un escalón de
pulso fijo y medir cómo responde la RPM) con el motor corriendo —
cualquier prueba en el motor real va a ser necesariamente **en lazo
cerrado**, con el PID ya corriendo. Por suerte, hay un método de
sintonización estándar pensado exactamente para esto: **Ziegler-Nichols
en lazo cerrado (ganancia última)**.

### Procedimiento (Ziegler-Nichols en lazo cerrado)

1. ⚠️ Paso desactualizado (`CALIB=10` ELIMINADO
   2026-09-29, ver sección 4.4): `PID_RPM_KP/KI` ahora se aceptan con
   `MODO=CALIBRACIÓN` + `CALIB=0` (ningún `CALIB` activo), no
   con un modo dedicado de sintonización — por medida de seguridad,
   que no se puedan tocar las ganancias por accidente fuera de una
   sesión deliberada de calibración. Con el motor ya estabilizado en
   ralentí caliente, seguir al paso 2.
2. Confirmar `PID_RPM_KI=0` y `PID_KD=0` (default de fábrica) — se
   sintoniza `Kp` solo primero.
3. Mandar un `SET_RPM` por encima de `RPM_MIN` (un escalón moderado,
   no extremo — ej. 200-300 RPM sobre el ralentí, no el máximo del
   motor) y observar la respuesta en el log de debug (`RPM filtrada`,
   cada 1s) o mejor, capturando el log completo a un archivo (log de
   PuTTY/Tera Term, o un script de Python leyendo el puerto COM y
   guardando CSV con marca de tiempo) para poder medirlo con precisión
   después en vez de a ojo.
4. Ir subiendo `Kp` en pasos pequeños (ej. `PID_RPM_KP 1.5`, `PID_RPM_KP 2.0`,
   ...) repitiendo el mismo escalón de `SET_RPM` cada vez, hasta que la
   RPM empiece a **oscilar de forma sostenida** (ni crece sin control
   ni se apaga, se mantiene oscilando con amplitud pareja alrededor del
   setpoint). Ese valor de `Kp` es la **ganancia última (`Ku`)**; medí
   el período de esa oscilación (tiempo entre dos picos consecutivos)
   — es el **período último (`Tu`)**.
   ⚠️ Al acercarte a `Ku` la RPM va a oscilar de verdad — tené a mano
   `SET_RPM 0` (por serial o MQTT) para cortar el lazo de inmediato si
   se ve demasiado agresivo, y no dejes esta prueba desatendida.
5. Con `Ku` y `Tu` medidos, aplicar las fórmulas clásicas de
   Ziegler-Nichols (elegir según qué tan agresivo se quiera el control
   final):
   ```
   Solo P:    Kp = 0.50 * Ku
   PI:        Kp = 0.45 * Ku          Ki = Kp / (Tu / 1.2)
   PID:       Kp = 0.60 * Ku          Ki = Kp / (Tu / 2)      Kd = Kp * (Tu / 8)
   ```
   Para un acelerador diésel, conviene arrancar por la fila **PI** (sin
   `Kd`) dado el ruido de medición ya conocido en el ralentí — agregar
   `Kd` solo si de verdad hace falta amortiguar más el sobre-impulso, y
   con cautela.
6. Cargar esas ganancias (`PID_RPM_KP`/`PID_RPM_KI`/`PID_KD` por downlink o
   serial) y repetir el escalón de `SET_RPM` del paso 3 — buscar una
   respuesta que llegue al setpoint sin oscilar más de una vez y sin
   error de estado estable notorio. Ajustar a mano desde ahí si hace
   falta (bajar un poco `Kp`/`Ki` si todavía se pasa de largo).

### ⚠️ Confirmado en campo (2026-09-01): Ziegler-Nichols oscila cerca del setpoint en este motor — usar SIMC en su lugar

Ziegler-Nichols está diseñado para dar la respuesta **más rápida
posible**, sin ponderar mucho el retraso de la planta — y en un motor
con tiempo muerto significativo respecto a su constante de tiempo
(como resultó ser este), eso se traduce en ganancias que **oscilan
justo al acercarse al setpoint**, sin importar qué tan chico se ponga
`Ki` (se confirmó con `Ki=0.05`, `0.02` y hasta `0.01` — todos terminan
oscilando, solo cambia cuánto tardan). No es un problema de encontrar
el valor correcto dentro de este método — es una limitación del método
en sí para este tipo de planta.

**Alternativa: [SIMC](https://folk.ntnu.no/skoge/publications/2003/tuningPID/) (Skogestad, 2003)**
— mismas entradas (`K`, `tau`, `L` del modelo identificado con
`identify`), pero deja elegir a propósito qué tan conservador ser en
vez de perseguir siempre la respuesta más rápida:
```
Kc = (1/K) × τ / (τc + L)          Ti = min(τ, 4×(τc + L))     Ki = Kc/Ti
```
`τc` (constante de tiempo deseada del lazo cerrado) se elige a mano —
más grande = más lento pero más robusto. Implementado en
`tools/pid_tuning/pid_tuning.py simc`. Confirmado en simulación:
con una planta de `K=0.5, tau=3.0s, L=1.5s`, las ganancias PI de
Ziegler-Nichols (`Kp=5.69, Ki=1.22`) oscilan sin asentar ni en 54s
simulados, mientras que SIMC con `τc=2L` (`Kp=1.33, Ki=0.44`) converge
suave y sin sobre-impulso en ~40s — mismo modelo de planta, mismo
punto de partida, solo cambia la fórmula de tuneo.

**Procedimiento revisado**: en vez del paso 4 de arriba (subir `Kp` en
el motor real hasta oscilación), alcanza con:
1. Un solo escalón de `SET_RPM` con `Kp=1, Ki=0, Kd=0` (ya lo tenés).
2. `python pid_tuning.py identify --log captura.log --kp 1.0` para
   sacar `K`/`tau`/`L`.
3. `python pid_tuning.py simc --K <K> --tau <tau> --L <L>` — da tres
   filas (agresivo/medio/conservador). Empezar por "Medio".
4. `python pid_tuning.py simulate --K ... --kp ... --ki ...` para
   confirmar que no oscila en la simulación antes de cargarlo.
5. Cargar esas ganancias y validar en el motor real — recién ahí, con
   mucho menos riesgo de toparse con la oscilación que costó tanto
   tiempo de motor real diagnosticar.

### ✅ Ganancias confirmadas en el motor real (2026-09-02): `Kp=0.3, Ki=0.02, Kd=0`

Validadas sobre el rango real de operación del motor (ralentí hasta
1200-1500 RPM, el máximo habitual de trabajo). Sostenidas más de 100s
en `SET_RPM=1200` sin oscilar y sin error de estado estable notorio
(RPM se mantiene en banda ~1195-1205, solo ruido normal de medición).

**Por qué no se llegó a esto por el camino de `identify`/`simc` de
arriba**: el motor resultó tener una ganancia de planta fuertemente no
lineal según el punto de operación — un escalón chico cerca de ralentí
(870→885 RPM) identificó `K=0.014`, mientras que un escalón hasta 1500
RPM identificó `K=0.224`, dieciséis veces más grande. Un modelo FOPDT
ajustado en un punto de operación no se puede extrapolar de forma
confiable a otro punto lejano — de hecho, al simular las ganancias que
SIMC calculó a partir del modelo de `K=0.224` contra un objetivo de
1500 RPM, el servo se saturaba al máximo sin llegar nunca al setpoint,
señal de que el modelo no era válido tan lejos de donde se identificó.

También se confirmó que el problema de oscilación cerca del setpoint
(sección anterior) no era solo cuestión de `Ki`: `Kp=0.6` y `Kp=1.0`,
que parecían limpios con P puro cerca de ralentí, resultaron oscilar
sosteniblemente (sin amortiguarse) al mandar un escalón directo a
`SET_RPM=1300` — el `Kp` en sí ya era demasiado agresivo para la zona
de mayor ganancia de la planta, lejos de ralentí.

**Lo que sí funcionó**: bajar a `Kp=0.3` (confirmado estable con P puro
en 1300 y 1500) y agregar un `Ki=0.02` chico de forma incremental,
igual que el resto de esta sección — validado directamente en el
motor real en vez de confiar en la extrapolación del modelo lineal.

**Sostenido largo en 1500 también confirmado limpio y asentado del
todo (2026-09-02)**: se repitió el escalón ralentí→1500, esta vez
sostenido ~170s completos — la RPM termina asentada en una banda
angustada justo alrededor de 1500 (~1485-1509, solo el ruido normal de
medición, ±10-15 RPM), sin un solo ciclo de oscilación en toda la
subida ni en el asentamiento final. `Kp=0.3, Ki=0.02, Kd=0` queda
**confirmado por completo** en todo el rango real de operación
(ralentí hasta 1500), en ambos setpoints probados (1200 y 1500).

**Bajada de setpoint confirmada limpia (2026-09-02)**: escalón directo
1400→1200 (mismas ganancias) desciende suave y monótono, sin ninguna
oscilación sostenida, convergiendo en ~85s — mismo comportamiento
simétrico que la subida.

**`Kd` probado y descartado (2026-09-02): la respuesta final es
`Kd=0`.** Se probó `Kd=0.005` sobre el `Kp=0.3, Ki=0.02` ya confirmado
(escalón ralentí→1200): no trajo ningún problema (`pulso_us` se
mantuvo igual de suave, sin el temblor por ruido que se temía) pero
tampoco ninguna mejora medible (tiempo de asentamiento prácticamente
igual, ~90-100s). Como un `Kd` que no demuestra beneficio es solo una
variable más para mantener, se optó por dejar `Kd=0`.

**`Ki=0.03` probado y descartado (2026-09-02) — confirma que `Ki=0.02`
queda como definitivo.** Se probó `Ki=0.03` buscando un asentamiento
más rápido. La primera prueba salió confundida (`Kd` seguía en
`0.005`, no en `0`) y mostró ráfagas cortas de saltos grandes entre
muestras (±30-90 RPM) durante la subida — parecía ruido amplificado
por la derivada. Se repitió con `Kd=0` confirmado explícitamente, y
**la misma ráfaga volvió a aparecer** — prueba que es `Ki=0.03` por sí
solo el causante, no `Kd`. La ráfaga es transitoria (el motor se
recupera solo y asienta limpio después, ~1195-1200 con ruido normal),
pero nunca apareció en ninguna prueba con `Ki=0.02` — señal de que
`0.03` ya está más cerca del límite de estabilidad de este motor. El
tiempo de asentamiento tampoco mejoró de forma medible (~90-130s en
ambos casos). Conclusión: sin beneficio de velocidad y con un riesgo
nuevo, se vuelve a `Ki=0.02`. Y dado que la ráfaga no es el patrón
clásico de sobre-impulso que `Kd` corrige, agregar `Kd` acá tampoco
tenía sentido — se descartó por la misma razón que antes.

**GANANCIAS FINALES DEL PID RIO-DSL: `Kp=0.3, Ki=0.02, Kd=0`** —
confirmadas limpias (sin oscilación, sin windup, sin ráfagas
transitorias, error de estado estable prácticamente nulo) en todo el
rango real de operación, en ambos sentidos (ralentí↔1200, ralentí↔1500,
bajada 1500→1200), y confirmadas como el mejor compromiso después de
probar y descartar tanto `Ki=0.03` como `Kd=0.005`.

### Re-sintonización para el montaje directo del servo (branch `servo-directo`, 2026-09-08)

Con el servo montado directo sobre el eje de la palanca del acelerador
(ver sección 12 y la memoria de rediseño mecánico) la planta cambió
por completo — no se puede reusar `Kp=0.3, Ki=0.02` de arriba, tunadas
contra la biela-manivela. Re-sintonización completa, con
`SERVO_PULSO_MIN=1124` / `SERVO_PULSO_MAX=1650` ya calibrados.

**`identify` dio `L=0.000s`** — tiempo muerto no detectable, confirma
que el montaje directo eliminó el falso tiempo muerto que tenía el
punto muerto de la biela-manivela (`K=0.246`, `tau=0.430s`, a partir de
un escalón cerrado con `Kp=0.3`). Pero ese `K` salió de una señal
débil y ruidosa (el escalón nunca llegó cerca del setpoint con
`Kp=0.3` solo), así que no es confiable por sí solo — el `K` real se
tomó del mapeo de curva de ganancia en lazo abierto
(`CALIB=5`, sección 12), que varía entre **1.9 y 6.3
RPM/µs** según la zona del rango (~3.4x de variación, menos que las
~8.8x de la biela-manivela, pero todavía significativo — sigue siendo
una planta con ganancia no uniforme, ver "gain-scheduling" en
sección 8).

**Ziegler-Nichols tampoco aplicó esta vez, pero por una razón
distinta**: con `L≈0`, `pid_tuning.py tune` no encuentra ninguna
ganancia última (`Ku`) ni con `Kp=1000` en la simulación — un sistema
de primer orden puro (sin tiempo muerto) bajo control proporcional es
incondicionalmente estable en la teoría, así que el método de
ganancia última de Z-N literalmente no tiene nada que buscar. La
oscilación real que sí se observó en el motor viene de retardos más
chicos que el modelo FOPDT no captura (muestreo de 200ms, filtro
`ALPHA`, límite de velocidad del servo).

**Bug encontrado en `pid_tuning.py simc`**: con `L=0` exacto, la
opción "Agresivo" (`tau_c=L`) divide entre cero
(`ZeroDivisionError` en `simc_pi_gains`). Se recalculó a mano con un
piso conservador `L=0.15s` (del orden del período de muestreo) en vez
de confiar en el `L=0` literal — pendiente de agregar ese piso mínimo
directamente al script.

**Validación en el motor real, iterativa**:
1. Con `K=3.0` (zona de ganancia media del mapeo) SIMC dio
   `Kp=0.25, Ki=0.57` — funcionó limpio en `SET_RPM=1050`, pero
   **osciló sostenido** en `SET_RPM=1300` (zona de mayor ganancia del
   mapeo) — confirma en campo el problema de ganancia no uniforme.
2. Recalculado con el `K=6.3` peor caso del mapeo → `Kp=0.12, Ki=0.27`
   — mucho mejor en 1300, pero quedó una oscilación lenta residual
   (~5-10s de período, ~90-130 RPM de amplitud).
3. Más conservador todavía (`tau_c=3×tau`) → `Kp=0.047, Ki=0.11` — la
   oscilación residual **casi no cambió** en tamaño ni período pese a
   bajar la ganancia proporcional/integral más de 2.5x, señal de que
   ya no era el lazo (más ganancia lo habría agravado, no una
   oscilación que se mantiene igual). Confirmado sin carga real (motor
   sin tubería conectada) — no es un efecto de purga de aire.
4. Dejado correr más tiempo en `SET_RPM=1300`: **asienta limpio
   después de un rato largo**, en banda ~1290-1310 (ruido normal),
   sin volver a oscilar — confirma que era un transitorio, no una
   inestabilidad sostenida.

**GANANCIAS FINALES (montaje directo): `Kp=0.047, Ki=0.11, Kd=0`** —
sintonizadas con SIMC (no Ziegler-Nichols, que no aplica a esta planta
por falta de tiempo muerto detectable) sobre el `K` peor caso medido
en el mapeo de ganancia, con margen extra de conservadurismo
(`tau_c=3×tau`) para tolerar la variación de ganancia entre zonas sin
gain-scheduling real todavía implementado.

⚠️ `PID_KD` se ELIMINÓ del protocolo 2026-09-23, decisión explícita del
usuario tras confirmar que este `Kd=0` seguía siendo válido y sin uso
real -- el ID 8 se reasignó a `TASA_LLENADO_PSI_S` (ver sección
4.3/tabla de parámetros); `pid.c` ya no calcula término derivativo,
solo P+I.

Asentamiento lento
**a propósito** — aceptado para esta aplicación (motobomba, necesita
tiempo igual para llenar/purgar la tubería). Pendiente: validar con
carga real de agua, y considerar gain-scheduling real si se necesita
una respuesta más rápida en el futuro.

**✅ Confirmado en banco (2026-09-08): estas ganancias funcionan bien
en un barrido de setpoints** por todo el rango real de operación
(ralentí hasta 1400+ RPM), sin oscilación sostenida en ningún punto,
con el motor sin carga (sin tubería conectada). Validación completa
con agua real todavía pendiente.

**⚠️ Incidente de campo (2026-09-08): un cambio de estructura en
`calibracion_flash.c` (fuera de esta sesión de tuneo, para agregar
`PRESION_GANANCIA_RPM`/`PRESION_OFFSET_RPM`) subió el "magic" de
validación de la flash (`"CALF"`→`"CALG"`).** En el siguiente reflash,
esto borró TODA la calibración guardada (`SET_RATIO`, `SERVO_PULSO_MIN/MAX`,
`PID_RPM_KP/KI/KD`, todo) y la volvió a los valores de fábrica del código,
sin ningún aviso visible más que el pulso quedandose fijo en el
`SERVO_PULSO_MIN` de fábrica (`1000`, no el `1124` calibrado) sin
reaccionar a ningún `SET_RPM`. Hay que estar atento a esto cada vez
que se toque la estructura `CalibFlash_Datos_t` — el próximo reflash
después de ese cambio borra silenciosamente todo lo calibrado en
campo. Ver también el nuevo log `PID_GANANCIAS` más abajo, agregado
justamente para que esto sea más fácil de detectar la próxima vez.

**Efecto colateral del mismo incidente: `MODO` (ID 16) recuperó su
comportamiento por defecto (`RALENTÍ`), que ahora sí es funcional**
(antes de que existiera `MODO_REMOTO`/presión, `MODO` no se leía en
ningún lado — ver el bloque "MODO: fuente del setpoint de RPM" en
`main.c`). Con `MODO=0` (RALENTÍ), el firmware ignora `SET_RPM` por
completo, sin importar qué tan bien esté sintonizado el PID —
síntoma: el servo se queda fijo en `SERVO_PULSO_MIN` pase lo que pase.
Hace falta `MODO=1` (LOCAL) para que `SET_RPM` vuelva a comandar el
lazo, como se venía usando durante toda esta sesión. **`MODO` se
reclasificó de `CONFIGURACION` a `CALIBRACION`** (2026-09-08) para
poder cambiarlo sin detener el motor — antes de que tuviera un efecto
real, no importaba en qué categoría cayera.

**Log `PID_GANANCIAS` (agregado 2026-09-08)**: mientras `PID_RPM_KP/KI`
están habilitados para tocarse (`MODO=CALIBRACIÓN` + `CALIB=0`,
gate actualizado 2026-09-29 -- antes era `CALIB=10`, ver
sección 4.4), el firmware imprime `PID_GANANCIAS,Kp=...,Ki=...` apenas
se cumple esa condición, y de nuevo cada vez que `PID_RPM_KP`/`PID_RPM_KI`
cambian (por serial o por downlink LoRa, los dos pasan por el mismo
dispatcher) — para que nunca más quede la duda de qué ganancias están
realmente vigentes en una sesión de calibración, después del incidente
de arriba. ⚠️ Formato corregido 2026-09-23: ya no imprime `Kd=...`
(`PID_KD` se eliminó del protocolo, ver más abajo).

**Investigación de discrepancia RPM vs. tacómetro analógico
(2026-09-08) — resuelta, no era el TID.** En campo se armó una tabla
comparando `SET_RPM` contra la lectura del tacómetro analógico de la
máquina, mostrando un desvío creciente con la RPM (bien hasta
~1150 RPM, hasta +100 RPM de diferencia en 1300-1400). Antes de tocar
nada del PID o del circuito de lectura, se comparó la frecuencia cruda
medida por el TID (monitor serial) contra la frecuencia real medida
con un osciloscopio directo en la señal W del alternador, en todo el
rango — **coincidieron casi exactas en cada punto** (ej. 383 Hz vs
383 Hz en 1300 RPM, 409 Hz vs 409 Hz en 1400 RPM). Esto descarta por
completo el circuito de lectura y la conversión frecuencia→RPM del
firmware (una sola fórmula lineal en `tacometro.c`, sin ninguna
compensación oculta) como causa. La razón frecuencia/RPM que implica
la lectura del tacómetro analógico, en cambio, **no es constante**
(~0.29 a baja RPM, ~0.273 a alta RPM, ~6% de no linealidad) — la
desviación está en el tacómetro analógico (su propio circuito interno
de conversión, o una calibración de fábrica para otro alternador), no
en el TID. **Conclusión: la lectura de RPM del TID es la correcta**,
no hace falta ni conviene replicar la no linealidad del instrumento
analógico en el firmware.

### Dónde sí ayuda un script de Python

No hace falta para ejecutar el procedimiento de arriba (es 100% de
campo, con los mandos que ya existen), pero sí ayuda para analizarlo
mejor que a ojo sobre el log crudo:

- Parsear el log capturado (serial o CSV) a una serie de tiempo real,
  y graficarla (`matplotlib`) — ver la oscilación sostenida del paso 4
  y medir `Tu` con precisión (distancia entre picos) es mucho más
  confiable en un gráfico que contando segundos en la terminal.
- Automatizar el cálculo de las fórmulas de Ziegler-Nichols del paso 5
  a partir de `Ku`/`Tu` medidos, para no hacerlo a mano.
- Si en algún momento se quiere ir más allá de Ziegler-Nichols (ajustar
  fino con un modelo real del sistema), se puede ajustar una curva a
  la respuesta capturada (ej. `scipy.optimize.curve_fit` contra un
  modelo de segundo orden) y usar ese modelo para probar más
  combinaciones de ganancias en la computadora antes de volver a tocar
  el motor — es exactamente el rol que iba a cumplir MATLAB, solo que
  con datos reales del motor en vez de un modelo teórico.

**Ya existe una implementación de esto** en [`tools/pid_tuning/`](../tools/pid_tuning/)
(`pid_tuning.py`): parsea el log `PID_TEST` capturado, identifica el
modelo de planta FOPDT (`K`/`tau`/`L`) de un escalón real en lazo
cerrado, busca `Ku`/`Tu` con Ziegler-Nichols **sobre el modelo
simulado** (sin oscilar el motor real), y sugiere ganancias P/PI/PID —
ver el `README.md` de esa carpeta para el flujo completo con ejemplos
de comandos. Replica a mano la lógica exacta de `PID_CalcularSalidaUs()`
(`pid.c`) para que la simulación sea fiel al firmware real; si `pid.c`
cambia, esa réplica en Python hay que actualizarla también.

### Alternativa más segura: identificar el modelo sin llevar el motor a oscilar

El paso 4 de arriba (subir `Kp` hasta oscilación sostenida) funciona,
pero implica hacer oscilar el motor real a propósito. Se puede evitar
por completo si lo que se quiere es simular en Python antes de tocar
ganancias agresivas en el motor: alcanza con **una sola prueba segura**
con las ganancias de fábrica (`Kp=1, Ki=0, Kd=0` — ya sabemos que esto
no oscila, solo deja error de estado estable).

1. Motor estable, mandar un escalón moderado de `SET_RPM` (ej.
   +200-300 RPM) y capturar el log `PID_TEST` (ver más abajo) hasta que
   se estabilice.
2. De la curva capturada, medir:
   - `ΔSET_RPM`: tamaño del escalón mandado.
   - `ΔRPM_ss`: cuánto subió la RPM en estado estable (menor que
     `ΔSET_RPM`, por el error de P puro).
   - `L`: retraso (s) entre el escalón y que la RPM empiece a moverse
     (incluye el slew-rate del servo y el retraso mecánico del motor).
   - `τ_cl`: tiempo desde que empieza a moverse hasta llegar al 63.2%
     del cambio total.
3. Como el lazo es proporcional puro con `Kp` conocido, se puede
   despejar la planta en lazo abierto (realimentación unitaria sobre
   una planta de primer orden):
   ```
   G_cl = ΔRPM_ss / ΔSET_RPM

   K = G_cl / (Kp * (1 - G_cl))     -- ganancia de la planta (RPM/µs)
   τ = τ_cl / (1 - G_cl)            -- constante de tiempo de la planta
   L ≈ L_cl                          -- el retraso no cambia con la realimentación
   ```
4. Con `K`/`τ`/`L` ya hay un modelo FOPDT (first-order-plus-dead-time)
   simulable en Python. Replicar ahí **la lógica exacta de `pid.c`**
   (mismo clamp `[0, rango]`, mismo anti-windup, mismo reset de
   integral con `Ki=0`) sobre ese modelo, y recién ahí sí buscar `Ku`/
   `Tu` subiendo `Kp` **en la simulación**, sin riesgo real. Validar la
   combinación ganadora en el motor real repitiendo el escalón del
   paso 1.
5. Este modelo es una aproximación lineal válida cerca del punto de
   operación donde se hizo la prueba — repetir con 1-2 escalones de
   tamaño distinto para confirmar que `K`/`τ`/`L` salen parecidos antes
   de confiar en él.

### Log `PID_TEST` para capturar los escalones (agregado en `main.c`)

Línea de log dedicada, en formato CSV, para no tener que parsear el
resto del texto intercalado en el monitor serie (RAK3172, GPS, eco del
mando manual, etc.):

```
PID_TEST,<ms_desde_arranque>,<SET_RPM>,<RPM_filtrada>,<pulso_servo_us>
```

- Se imprime cada `200ms` (5Hz) — mucho más seguido que el log general
  de 1s, para tener suficientes puntos por constante de tiempo al
  ajustar la curva.
- ⚠️ Actualizado 2026-09-29: **se activa en `CALIB=6`**
  (auto-escalón interno, ver más abajo) **o `=7`** (validación con
  ganancias reales, ver sección 4.4) — ya NO existe el viejo `CALIB=10`
  (numeración anterior a 2026-09-29) como modo dedicado de sintonización
  (eliminado, ver sección 4.4);
  `PID_RPM_KP/KI` ahora se desbloquean con `MODO=CALIBRACIÓN` +
  `CALIB=0`, condición separada de este log. Fuera de `=6`/`=7`
  el monitor se ve exactamente como antes de que existiera este log
  (solo la línea de 1s). Se apaga solo en cuanto ese `CALIB` termina o se
  aborta. `pid_tuning.py identify`/`auto --loop interno` funcionan
  igual sobre la salida de cualquiera de los dos modos.
- Usa `HAL_GetTick()` (ms desde el arranque), **no** la hora del RTC —
  el RTC se resincroniza cada ~30s por GPS/LoRaWAN (sección 2.2) y esas
  correcciones contaminarían el tiempo relativo de un escalón en
  curso; `HAL_GetTick()` es monótono y no depende de si el reloj ya
  sincronizó.
- Incluye el **pulso real aplicado al servo** (`Servo_GetPulsoActualUs()`),
  no solo el setpoint — importante porque `SERVO_VELOCIDAD_MAX_US_S`
  (sección 2.4) limita qué tan rápido puede moverse el pulso real, así
  que el valor efectivamente aplicado puede ir por detrás de lo que el
  PID "pidió" ese ciclo.
- Para capturarlo: log del terminal (PuTTY/Tera Term) o un script
  leyendo el puerto COM, filtrando las líneas que empiecen con
  `PID_TEST,` antes de armar el CSV para Python.

### Log `PRESION_PID_TEST` para el lazo EXTERNO de presión (`MODO=1`, agregado 2026-09-14)

Mismo formato/propósito que `PID_TEST` de arriba, pero para el lazo de
presión en cascada (sección 4.3, `presion_pid.c`). `pid_tuning.py` tiene
un subcomando dedicado, `identify-presion` (agregado 2026-09-21), que
parsea este formato de 5 campos y corre el mismo núcleo de
identificación FOPDT que `identify` (compartido internamente) pero en
las unidades de este lazo (RPM de corrección → PSI, en vez de µs de
servo → RPM).

```
PRESION_PID_TEST,<ms_desde_arranque>,<PRESION_OBJETIVO_LOCAL>,<PresionV_medida>,<RPM_generado_por_el_lazo_externo>,<RPM_real>
```

- Se imprime cada `200ms` (5Hz), igual cadencia que `PID_TEST`.
- ⚠️ Actualizado 2026-09-29: **se activa con `CALIB=10`**
  (auto-escalón local, antes numerado `=8`, ver más abajo) **o `=11`**
  (validación con ganancias reales, antes numerado `=13`, ver sección
  4.4) **Y la cascada real corriendo**
  (`presionLocalActivoAhora`) — ya no existe el viejo `CALIB=10` de
  sintonización manual (numeración anterior a 2026-09-29 -- no
  confundir con el `CALIB=10` actual, arriba, que es otra cosa).
  Ambos modos escriben el mismo log `PRESION_PID_TEST`, solo cambia el
  propósito de cada uno (identificación con `Ki=0` forzado en `=10`,
  validación con las ganancias reales sin forzar nada en `=11`).
- Incluye las dos variables de la cascada en la misma línea: la
  presión (lo que este lazo controla) y el RPM resultante (su salida,
  que a su vez es el setpoint que recibe el lazo interno ya validado)
  — útil para ver ambos lazos a la vez sin tener que cruzar dos logs.

**Flujo de sintonización recomendado (Fase E de la sección 12), usando
`pid_tuning.py`:**
1. **Caracterización en lazo abierto** (paso 9 de la Fase E): en
   `MODO=4` (calibración), mandar distintos `SET_RPM` a mano y anotar
   la presión resultante del log general de 1s (`Presion: .../PresionV: ...`)
   — no hace falta `PRESION_PID_TEST` para esto todavía, ya que
   `PID_PSI_KP/KI` en `0` significa que el lazo externo no corre.
2. **Identificación del modelo**: `pid_tuning.py identify-presion --log
   <captura> --kp <PID_PSI_KP usada>` sobre un log de
   `PRESION_PID_TEST` en lazo cerrado, para obtener `K`/`τ`/`L` de la
   relación presión↔RPM de este sistema hidráulico específico.
   **Requiere `PID_PSI_KI=0` durante ESA captura** (mismo motivo
   que el lazo interno, sección 9: con el integral activo el error de
   estado estable se borra en cada escalón y la fórmula de `K` se
   indefine — confirmado en campo 2026-09-21, un log con
   `PID_PSI_KI=0.3` ya activo no sirve para este paso, aunque sí
   sirve para validar el paso 4). El `K` del paso 1 (lazo abierto) es
   preferible de todas formas por la misma razón que se explica en la
   sección 9 para el lazo de RPM — usar `identify-presion` sobre todo
   para sacar `τ`/`L`, que el paso 1 no da.
3. **Cálculo de ganancias**: `pid_tuning.py simc` (u otra opción del
   script) sobre ese modelo — mismo cuidado con el bug conocido de
   `L=0` exacto (ver sección 9), y preferir el `K` medido en lazo
   abierto (paso 1) sobre el que salga de un escalón cerrado chico,
   por la misma razón que se explica en la sección 9 para el lazo de
   RPM.
4. **Validación real**: cargar `PID_PSI_KP/KI`, entrar a `MODO=1`,
   y repetir el escalón en el motor real, ajustando de a poco --
   idéntica metodología a la sección 9, solo que la variable
   controlada es presión en vez de RPM.

### Automatizando el escalón: `CALIB=6`/`=7`/`=9`/`=10`/`=12` y `pid_tuning.py auto`/`compare`/`sugerir-kp-presion` (agregado 2026-09-23, extendido 2026-09-24 y 2026-09-25)

Todo el procedimiento de arriba (los tres lazos: interno de RPM, local
de presión y remoto) sigue siendo **100% manual y de campo** en su
esencia -- lo único que se automatizó es la parte más mecánica y
propensa a error humano de repetirlo: mandar el escalón, medir el
tiempo/cantidad de reportes, y no olvidarse de deshacerlo. Siete
piezas (la tercera, `=6`, agregada en la segunda renumeración del mismo
día para darle al lazo interno la misma comodidad que ya tenían los
dos lazos de presión; la cuarta, `=9`, agregada un día después para
darle al PID#2 el mismo tipo de barrido en LAZO ABIERTO que ya tenía el
PID#1 con `=5`; la quinta, `compare`, agregada otro día después para
verificar en campo lo que `auto` solo predice; la sexta, `=7`, agregada
el mismo día que `compare` para automatizar justo el log que `compare`
necesita; la séptima, `sugerir-kp-presion`, agregada el mismo día para
que el operador ya no tenga que adivinar qué `Kp` de prueba usar en
`=10`), pensadas para usarse juntas:

- **`CALIB=6`** (ver sección 4.4/12) automatiza el
  escalón del lazo INTERNO de RPM (PID#1) -- requiere `MODO=4`
  (CALIBRACIÓN) y el motor operando, comanda `SET_RPM = RPM_MIN + 200`
  RPM fijo (`INTERNO_CAL_ESCALON_RPM`) y loguea `PID_TEST` (el mismo
  log que usaba la vieja sesión de sintonización manual, `CALIB=10`
  numeración anterior a 2026-09-29, eliminada) durante una ventana FIJA de 60
  segundos (`INTERNO_CAL_DURACION_MS`) -- de sobra frente a lo rápido
  que asienta este lazo, a diferencia de los lazos de presión que
  necesitan 10-60 minutos. Se aborta y resetea `SET_RPM` a `0.0` si el
  motor se detiene o `MODO` deja de ser `4` a mitad de la prueba, y
  auto-revierte a `CALIB=0` al completar o abortar. Log:
  `INTERNO_CAL,INICIO,...` / `INTERNO_CAL,ESCALON,...` /
  `INTERNO_CAL,COMPLETO,duracion_s=...` /
  `INTERNO_CAL,ABORTADO,motivo=...`.
- **`CALIB=10`/`=12`** (ver sección 4.4/12) automatizan
  SOLO el envío del escalón y la espera para los lazos de presión --
  `10` para el lazo local (`MODO=1`, mide una presión base real antes de
  arrancar y manda `PRESION_OBJETIVO_LOCAL` = base + 5 PSI fijo, agregado
  2026-09-25, ver sección 4.4; ventana fija de 3min (bajada de 10min
  2026-09-28 tras identificar el modelo real con el log de campo, ver
  sección 9), loguea
  `PRESION_PID_TEST`) y `12` para el remoto (`MODO=4`, escalón fijo de
  `SET_RPM`, termina por reportes o timeout, loguea el log
  `REMOTO_PID_TEST`). `10` también guarda y fuerza `PID_PSI_KI=0`
  durante la prueba (restaurado al salir, mismo motivo que el lazo
  interno) -- el `Kp` de prueba sigue siendo libre, ver
  `sugerir-kp-presion` más abajo. El servo lo sigue manejando el PID
  normal en los tres casos (`6`/`10`/`12`) -- no es un modo de lazo
  abierto, y ninguno calcula ninguna ganancia por sí solo. Los tres se
  auto-abortan y deshacen el escalón si el motor se detiene o `MODO`
  cambia a mitad de la prueba, así que no hace falta estar pendiente
  para "limpiar" manualmente si algo interrumpe la sesión.
- **`CALIB=9`** (agregado 2026-09-24, diseño definitivo
  2026-09-28, ver sección 4.4/12) automatiza el mapeo de ganancia en LAZO
  ABIERTO del PID#2 (presión local) -- equivalente de `=5` (que hace lo
  mismo para el PID#1) pero con `SET_RPM` en `MODO=4` en vez del pulso
  del servo: `presion_pid.c` no corre y no hay `PID_PSI_KP/KI` de por
  medio (así la curva no queda contaminada por un controlador, y no
  depende de que ya haya un `Kp` cargado), pero el PID#1 de RPM ya
  validado sí sigue moviendo el servo. A diferencia de `5`/`6`/`10`/`12`,
  este SÍ tiene un riesgo físico real (golpe de ariete, hay presión
  hidráulica de por medio), así que la subida está frenada por la
  presión REAL medida:
  1. Posa el motor en `RPM_MIN` y promedia la presión real durante 5s
     (`PRESION_CAL_BASE_PROMEDIO_MS`, misma ventana que `=10`) → presión
     base.
  2. Calcula el ritmo máximo seguro de subida
     `(PRESION_OBJETIVO_LOCAL − base) / TIEMPO_LLENADO_S` en PSI/s -- el mismo
     que ya usa la rampa de llenado real de `MODO=1` (sección 4.3.1) --
     y un ritmo nominal de avance de RPM
     `(RPM_MAX_CARGA − RPM_MIN) / TIEMPO_LLENADO_S`.
  3. En cada ciclo compara la presión real contra la "línea ideal"
     `base + ritmo_seguro × tiempo_transcurrido`: si va POR DEBAJO, sube
     RPM al ritmo nominal; si ya la alcanzó o superó, **pausa** el avance
     (RPM se mantiene, no baja) hasta que la línea la vuelva a superar.
     No es un PID (no hay ganancia ni integral que calibrar) -- es un
     semáforo avanza/pausa. Resultado: la presión nunca sube más rápido
     de lo que el operador ya definió como seguro, SIN IMPORTAR la
     ganancia real de la instalación, y la prueba dura lo que el operador
     dijo que tarda el llenado (ejemplo real que motivó este diseño: si
     la bomba llega a 60 PSI con solo 900 de 1200 RPM, una rampa pautada
     solo por RPM hubiera subido 50 PSI en ~10 min en vez de los 30 min
     que el operador validó -- el freno por PSI lo evita).
  4. Loguea `PRESION_GANANCIA_CAL,PASO,rpm=...,psi=...,salto=...` cada
     vez que la presión real avanza 1 PSI
     (`PRESION_GANANCIA_CAL_PASO_LOG_PSI`) -- el tamaño del log queda
     acotado por el rango de presión recorrido (p.ej. ~50 líneas de 10 a
     60 PSI), no por cuánto tarde la prueba. Si la presión brinca más de
     1 PSI en un solo ciclo, sale UNA línea con el `salto` real (p.ej.
     `salto=3.00`) en vez de tres -- no se pierde información de
     ganancia (el análisis usa la pendiente entre líneas consecutivas),
     y con una rampa tan lenta un brinco así sería en sí mismo anómalo
     (probable ruido de sensor), visible en el log.

  Termina por lo que ocurra primero: presión real llega a
  `PRESION_OBJETIVO_LOCAL` (`objetivo_alcanzado`, el caso normal); RPM llega a
  `RPM_MAX_CARGA` sin alcanzarlo (`rpm_max_carga_alcanzado` -- respaldo
  de seguridad que en una instalación normal no debería tocarse nunca,
  y que si se toca es en sí mismo un diagnóstico: la bomba no llega a
  esa presión dentro de su límite seguro de RPM); o un timeout de
  último recurso de `1.5 × TIEMPO_LLENADO_S`
  (`PRESION_GANANCIA_CAL_TIMEOUT_MARGEN`, solo por si el sensor se traba
  sin marcarse inválido). Guardas explícitas: si la presión base ya está
  en o sobre el objetivo completa de inmediato
  (`objetivo_ya_alcanzado`), y si `TIEMPO_LLENADO_S=0` o
  `RPM_MAX_CARGA ≤ RPM_MIN` sale con `ERROR,motivo=config_invalida`. Se
  aborta y resetea `SET_RPM` a `0.0` si el motor se detiene, `MODO` deja
  de ser `4`, o el sensor de presión deja de ser válido
  (`sensor_invalido`); auto-revierte a `CALIB=0` al
  terminar por cualquier motivo. Asume la tubería YA llena/presurizada
  -- no reemplaza un primer llenado manual (ver sección 12, Fase D). Dos
  diseños previos se descartaron el mismo día: pasos discretos de
  `SET_RPM` con espera de asentamiento y umbral de salto anormal (sus
  constantes propias causaron 3 bugs reales de calibración) y reusar la
  cascada real de `MODO=1` (dependía de un `Kp` ya cargado -- con `Kp=0`
  de fábrica no hacía nada -- y el PID contaminaba la curva).
  Log: `PRESION_GANANCIA_CAL,INICIO,...` /
  `PRESION_GANANCIA_CAL,BASE,psi_base=...` /
  `PRESION_GANANCIA_CAL,RAMPA,objetivo=...,tasa_psi_s=...,tasa_rpm_s=...,rpm_max_carga=...,timeout_s=...` /
  `PRESION_GANANCIA_CAL,PASO,rpm=...,psi=...,salto=...` /
  `PRESION_GANANCIA_CAL,COMPLETO,motivo=objetivo_alcanzado|objetivo_ya_alcanzado|rpm_max_carga_alcanzado|timeout,...` /
  `PRESION_GANANCIA_CAL,ABORTADO,motivo=motor_se_detuvo|modo_cambio|sensor_invalido` /
  `PRESION_GANANCIA_CAL,ERROR,motivo=config_invalida,...`.
- **`CALIB=7`** (agregado 2026-09-25, ver sección 4.4)
  automatiza el log de VALIDACIÓN del lazo interno (PID#1) -- clon
  mecánico de `=6` (mismo escalón, mismo log `PID_TEST`, requiere
  `MODO=4` + motor operando), pero **sin forzar `PID_RPM_KP=1/PID_RPM_KI=0`**
  (usa la ganancia YA cargada en flash, la recomendada por `auto` y
  bajada por downlink) y con una ventana de 4 minutos en vez de 1 --
  reemplaza el "dejalo asentar y mirá cómo se ve" manual. Suma un
  watchdog de oscilación sostenida que no tienen `6`/`9`/`10`/`12`: pasada
  una espera de 30s, cuenta cruces de signo de `RPM - SET_RPM` (con piso
  anti-ruido de 15 RPM) en una ventana móvil de 20s -- 4 cruces abortan
  de inmediato (`SET_RPM=0.0`), mismo criterio que antes aplicaba el
  operador a ojo. El log resultante se le pasa directo a `compare` (ver
  más abajo). Log: `VALIDACION_CAL,INICIO,...` /
  `VALIDACION_CAL,ESCALON,...` / `VALIDACION_CAL,COMPLETO,...` /
  `VALIDACION_CAL,ABORTADO,motivo=motor_se_detuvo|modo_cambio|oscilacion_sostenida`.
- **`pid_tuning.py auto --loop {interno,presion,remoto}`** encadena en
  un solo comando lo que antes eran pasos sueltos (`identify`/
  `identify-presion`/`identify-remoto` → `simc` → `simulate`) y sugiere
  ganancias directo sobre el log capturado (con `CALIB=6`
  o `=7` para el lazo interno, o con el log que resulte de `=10`/`=12`/`=11`
  para los lazos de presión). `--loop interno` y `--loop presion`
  aceptan además `--ganancia-log <log del barrido correspondiente>`
  (`CALIB=5` para interno, `=9` para presión) para
  reemplazar la `K` identificada en lazo cerrado (poco confiable, ver
  sección 9 más arriba) por la `K` peor caso del mapeo de ganancia en
  lazo abierto -- para `--loop interno` es la misma metodología real
  que se usó a mano para llegar a `Kp=0.047, Ki=0.11` (ver más arriba,
  "Re-sintonización para el montaje directo del servo"), ahora
  reproducible con un solo comando en vez de recalcular a mano; para
  `--loop presion` es la contraparte nueva, sin validar todavía en
  campo (bloqueado por falta de motobomba al momento de escribir esto).
  El lazo remoto además tiene `identify-remoto`, para la identificación
  en LAZO ABIERTO específica de ese caso (ver el comentario de `main.c`
  sobre el log `REMOTO_PID_TEST`, evento-driven y gateado a `MODO=4`,
  no `MODO=2`) -- por eso `--ganancia-log` no aplica a `--loop remoto`,
  ese lazo ya se identifica en lazo abierto directo, sin el problema de
  `K` poco confiable que las otras dos opciones corrigen.
- **`pid_tuning.py compare --loop {interno,presion,remoto}`** (agregado
  2026-09-25) cierra el ciclo: `auto`/`simulate` dan una `Kp`/`Ki`
  sugerida sobre un MODELO simulado, pero eso no reemplaza confirmar en
  el motor real -- `compare` toma un log normal (`PID_TEST`/
  `PRESION_PID_TEST`/`REMOTO_PID_TEST`, con esas ganancias YA cargadas y
  corriendo, a diferencia de `identify`/`auto` que exigen `Ki=0` durante
  la captura) y lo compara contra la simulación del mismo modelo
  (`--K --tau --L`) con esas mismas ganancias (`--kp --ki`). Dos
  salidas: un **% de parecido** (100% menos el error cuadrático medio
  normalizado al tamaño del escalón, no correlación cruzada -- penaliza
  mejor una oscilación real que la correlación, que puede seguir dando
  alto aunque el real oscile encima de la forma general) y una
  **detección de oscilación** aparte, contando cambios de signo del
  residuo (real menos simulado) en la ventana de asentamiento -- 3 o más
  cruces dispara la advertencia, específicamente para el caso de "el %
  de parecido se ve bien pero el real igual empezó a oscilar" que un
  solo número podría esconder. También compara sobre-impulso, tiempo de
  asentamiento y error de estado estacionario real vs. simulado, y
  `--plot-out` da el gráfico con el residuo en un segundo eje. Sirve
  para los tres lazos (mismo motor de simulación que `simulate`/`auto`,
  solo cambian los parámetros que se le pasan). Para el lazo interno, el
  log del `CALIB=7` de arriba ya viene en el formato que
  este comando espera, sin captura manual.
- **`pid_tuning.py sugerir-kp-presion --ganancia-log <log de CALIB=9>`**
  (agregado 2026-09-25) resuelve la pregunta de "¿qué `Kp` de prueba
  mando antes de correr `CALIB=10`?" sin que el operador tenga que adivinar
  ni mandar varios logs a revisar -- calcula un `Kp` sugerido a partir
  de la ganancia (`K`) YA medida por el barrido en lazo abierto de
  `CALIB=9`, usando `G_cl = K·Kp/(1+K·Kp)` despejada para un `G_cl`
  objetivo configurable (`--g-cl-objetivo`, default `0.3`). Usa el `K`
  MÍNIMO del barrido (la zona de menor ganancia) para garantizar señal
  suficiente en toda la zona operativa, y avisa si esa elección deja el
  `G_cl` demasiado alto en la zona de mayor ganancia (riesgo de
  inestabilidad numérica en la identificación posterior, no de golpe de
  ariete -- ver la derivación completa más arriba). El siguiente paso
  que imprime es literal: bajar `PID_PSI_KP`/`PID_PSI_KI=0` por
  downlink con ese valor, y recién ahí mandar `CALIB=10`
  (que además ya fuerza `PID_PSI_KI=0` solo, ver sección 4.4).

⚠️ **Ninguna de las siete piezas aplica ganancias sola.** `auto` termina
en una recomendación (`Kp`/`Ki` sugeridos), no en un downlink -- cargar
esas ganancias (`PID_RPM_KP/KI`, `PID_PSI_KP/KI` o
`PID_ASP_KP/KI`, según el lazo) y confirmar que la respuesta
real no oscila sigue siendo un paso manual, con el mismo criterio de
"validar en el motor real antes de confiar en el modelo" que el resto
de esta sección. Ver `tools/pid_tuning/` para el detalle de los
subcomandos.

## 10. Auto-calibración de ralentí (`CALIB=8`)

⚠️ **Renumerado 2026-09-23 (antes era el valor `4`), y de nuevo
2026-09-29 (era `7`, ver sección 4.4)** — se corrió a un
número más alto, después de la zona muerta/curva de ganancia/PID
interno (`4`/`5`/`6`), porque el ralentí real de un motor diesel SÍ
cambia según haya o no carga hidráulica conectada (confirmado en
campo — a diferencia de esos otros tres, que dan el mismo resultado
con o sin carga), así que conviene medirlo una sola vez, ya con la
tubería conectada, en vez de repetirlo. Ver sección 12 para el orden
completo de puesta en marcha.

Primera de las rutinas de auto-calibración planteadas para sacar el
ojo humano de la calibración inicial de cada unidad (la segunda,
detectar automáticamente dónde el servo empieza a acelerar el motor
para fijar `SERVO_PULSO_MIN`, queda pendiente de diseñar — a
diferencia de esta, sí necesita mover el servo directo con el motor
operando, así que le hace falta su propia cerca de seguridad antes de
construirse, no es un ajuste chico).

**Qué hace**: mide el ralentí real del motor (promediando
`RPM_filtrada`) y actualiza `RPM_MIN` automáticamente, en vez de que
alguien tenga que mirar el log y copiar un número a mano.

**Cómo usarla**: estando en operación normal (`CALIB=0`),
mandar
```
CALIB 8
```
(por downlink o por el mando manual de serial, sección 7). A
diferencia de los modos `1`/`2` (calibración de servo), **no hace
falta detener el motor** para entrar — de hecho la rutina solo tiene
sentido con el motor operando. Se puede mandar con el motor ya
corriendo, o antes de arrancarlo (en ese caso, la rutina simplemente
espera a que arranque antes de empezar a contar la ventana de 2
minutos). Ver sección 4.4 para el resto de la máquina de estados de
`CALIB`.

**Condiciones para que se acepte el downlink** (si no se cumplen, se
rechaza de una vez, sin quedar pendiente):
- Modo actual `CALIB=0` — no se puede pedir `8` directo
  desde `1`, `2`, ni desde `4`/`5`/`6`/`10`/`12` (rechazo
  `OUT_OF_RANGE`, ver sección 4.4).

Una vez dentro del modo `8`, el servo queda fijo en `SERVO_PULSO_MIN`
todo el tiempo (igual que "sin control activo" en operación normal),
sin importar en qué punto esté la ventana de medición.

**Por qué dura 2 minutos en vez de unos segundos**: un motor frío
puede arrancar bien por debajo de su ralentí real y tardar en subir
(observado en campo: arranca ~500 y sube a ~800 ya caliente). Un
promedio corto agarraría esa rampa de calentamiento y calcularía un
`RPM_MIN` demasiado bajo. La rutina corre **2 minutos en total** desde
que el motor está operando, pero solo promedia los **últimos 30
segundos** de esa ventana — los primeros ~90s sirven de tiempo de
calentamiento/estabilización y se descartan.

**Si el motor se detiene a mitad de la ventana**: la medición en curso
se descarta (no tiene sentido promediar con el motor parado), pero se
permanece en modo `8` esperando a que vuelva a arrancar — no hace
falta reenviar el downlink, la ventana de 2 minutos arranca de nuevo
sola apenas el motor esté operando otra vez.

**Resultado**: `RPM_MIN` = promedio de los últimos 30s + margen fijo
de 30 RPM (para que el ruido normal de medición, ±10-15 RPM, no cruce
el umbral por accidente y dispare el PID solo por ruido). Al aplicar
el resultado, el firmware vuelve solo a `CALIB=0` — no
hace falta un downlink extra para salir del modo `8`.

**"ACK" por LoRa al terminar**: además del log serial, al completar la
medición el firmware fuerza un uplink inmediato (`CalibFlash_ForzarReporte()`,
misma bandera interna que dispara `FORZAR_REPORTE` por downlink) en vez
de esperar al próximo intervalo periódico de 30s — así se sabe por
LoRaWAN, sin mirar el log serial, que la calibración terminó. Es el
mismo formato de uplink estándar (RPM/estado/posición, sección 6), no
lleva el valor nuevo de `RPM_MIN` como campo propio — si hiciera falta
que el uplink cargue ese valor explícito habría que extender el
formato de 27 bytes (fuera de este repo, en el decoder de AWS).

**Log de progreso** (formato CSV, igual criterio que `PID_TEST`):
```
RALENTI_CAL,INICIO,esperando_motor=<0/1>
RALENTI_CAL,MIDIENDO,duracion_total_s=120,ventana_promedio_s=30
RALENTI_CAL,PROGRESO,transcurrido_s=<s>,rpm_actual=<rpm>            -- cada 5s
RALENTI_CAL,COMPLETO,promedio=<rpm>,RPM_MIN_nuevo=<rpm>,muestras=<n>,ok=<0/1>
RALENTI_CAL,ABORTADO,motivo=motor_se_detuvo                         -- vuelve a esperar, sigue en modo 8
```

**Pendiente**: `send_downlink.py` (Lambda AWS, fuera de este repo)
necesita `PARAMETER_TABLE["CALIB"]["max"]` bumped hasta
`12` (cubre este modo y el de la sección 11, más los agregados
después) — mismo patrón que cuando se agregaron
`CALIB=2` y los modos posteriores.

## 11. Auto-calibración de `SERVO_PULSO_MIN` (`CALIB=4`)

⚠️ **Renumerado 2026-09-23 (antes era el valor `5`)** — se corrió a un
número más bajo, ANTES del ralentí (sección 10) y del PID interno,
porque `SERVO_PULSO_MIN` es la línea base de la propia fórmula de
salida del PID (`pid.c`: `rango = maximo - minimo`) — sintonizar el
lazo interno con una zona muerta todavía no definitiva invalidaría esa
sintonización en cuanto se corrigiera después. Ver sección 12 para el
orden completo de puesta en marcha.

Segunda rutina de auto-calibración (la primera es la sección 10).
Encuentra el pulso exacto donde el acelerador empieza a mover la RPM
de verdad — motivado por una observación de campo: moviendo el brazo
del servo a mano, unos grados de recorrido no hacen nada (una **zona
muerta** mecánica del acelerador) antes de que el motor empiece a
reaccionar. Si `SERVO_PULSO_MIN` queda apenas *dentro* de esa zona
muerta, las correcciones chicas del PID cerca de ralentí caen a veces
adentro (sin efecto) y a veces la cruzan (respuesta brusca) — un
posible motivo de la oscilación intermitente observada a RPM bajas.

A diferencia de `CALIB=1`/`=2` (que también mueven el
servo directo, pero a ciegas, sin mirar la RPM — por eso exigen el
motor detenido), este modo sí mira la RPM en cada paso, así que puede
correr con el motor operando de forma seguro y acotada.

**Cómo usarla**: estando en operación normal (`CALIB=0`),
mandar
```
CALIB 4
```
Igual que el modo `8` (ralentí): no hace falta detener el motor primero
(si se manda sin el motor operando, la rutina espera a que arranque).
Solo se acepta viniendo de modo `0` (rechazo `OUT_OF_RANGE` si se pide
desde `1`/`2`, ni desde `5`/`6`/`8`/`10`/`12`).

**Algoritmo:**
1. **Línea base fresca**: al arrancar (con el motor ya operando),
   promedia `RPM_filtrada` durante 12s con el servo quieto en el
   `SERVO_PULSO_MIN` actual — no confía en `RPM_MIN` guardado, mide de
   nuevo en el momento.
2. **Pasos chicos hacia arriba**: incrementa el pulso de a 8µs por
   vez, empezando en `SERVO_PULSO_MIN`.
3. **Espera de asentamiento**: 2.5s en cada paso antes de leer la RPM.
4. **Detección**: si `RPM_filtrada` sube 30 RPM o más sobre la línea
   base (bien por encima del ruido normal de ±10-15 RPM) durante 2
   lecturas seguidas **en el mismo pulso** (no dispara con un pico de
   una sola muestra) → ahí "empieza a acelerar".
5. **Margen de seguridad**: el nuevo `SERVO_PULSO_MIN` = el pulso
   donde se confirmó la detección, menos 6 pasos (48µs, subido de 3
   pasos/24µs el 2026-09-03 — ver nota de campo más abajo) de colchón —
   para no quedar pegado justo al borde de la zona muerta.
6. **Techo duro**: si se suben 180µs sobre el `SERVO_PULSO_MIN` de
   entrada sin detectar nada, se aborta con error — no sigue subiendo
   a ciegas si algo no encaja.
7. **Aborto inmediato** si: el motor se detiene a mitad del barrido
   (se descarta el progreso, pero se sigue en el modo esperando a que
   vuelva a arrancar — no hace falta reenviar el downlink), o si un
   solo paso sube la RPM 120 o más de golpe (un salto mucho más grande
   de lo esperado — señal de que algo salió distinto a lo previsto, se
   frena ya en vez de aplicar cualquier resultado automático).
8. **Al completar** (éxito o error): el servo vuelve a
   `SERVO_PULSO_MIN` (el nuevo si se encontró, el de entrada si no), y
   `CALIB` vuelve solo a `0`. Si se aplicó un valor nuevo,
   dispara el mismo "ACK" por LoRa que la calibración de ralentí
   (`CalibFlash_ForzarReporte()`, sección 10).

**Análisis de riesgo** (por qué estos números son de bajo riesgo):
- El techo (180µs) es una fracción chica del recorrido total del
  servo (~1075µs típico, `925→2000`) — menos del 20%. El pulso nunca
  puede terminar más allá de ese techo, sin importar qué pase.
- El movimiento es autolimitado por diseño: en cuanto se confirma la
  detección, deja de subir. No sigue empujando "para confirmar más" —
  el peor caso es quedarse justo en el borde de la zona muerta, nunca
  mucho más allá.
- Duración acotada: en el peor caso (llega al techo sin detectar
  nada), son `180/8 ≈ 23` pasos × 2.5s ≈ 1 minuto — motor cerca del
  ralentí ese rato, nada distinto a dejarlo en marcha mínima.
- El riesgo real no es mecánico, es de precisión: si el tiempo de
  asentamiento (2.5s) fuera corto para el motor real, un paso podría
  "pasarse de largo" unos µs antes de que se note la subida de RPM.
  Pero el margen de seguridad (retroceder 6 pasos al aplicar el
  resultado) ya absorbe justo ese tipo de error — no hace falta que la
  detección sea perfecta al µs.

**Log de progreso** (formato CSV):
```
SERVOMIN_CAL,INICIO,esperando_motor=<0/1>
SERVOMIN_CAL,BASELINE,pulso_base=<us>,duracion_s=12
SERVOMIN_CAL,BARRIENDO,baseline_rpm=<rpm>,pulso_inicial=<us>
SERVOMIN_CAL,PASO,pulso=<us>,rpm=<rpm>,subida=<rpm>,confirmaciones=<0-2>
SERVOMIN_CAL,COMPLETO,pulso_umbral=<us>,SERVO_PULSO_MIN_nuevo=<us>,ok=<0/1>
SERVOMIN_CAL,ABORTADO,motivo=motor_se_detuvo        -- vuelve a esperar, sigue en modo 4
SERVOMIN_CAL,ABORTADO,motivo=salto_rpm_anormal      -- sale del modo, no aplica nada
SERVOMIN_CAL,ERROR,motivo=techo_alcanzado_sin_deteccion  -- sale del modo, no aplica nada
```

**✅ Confirmado en campo (2026-09-03)**: en el motor de pruebas se
encontró una zona muerta mecánica real de ~136µs (`925→1061`, mucho
más grande que el techo de 180µs presupuestado — quedó justo). Se
aplicó `SERVO_PULSO_MIN=1037` (1061 − 3 pasos de 8µs, margen original).

**⚠️ Corrección de campo el mismo día**: al volver a correr esta
rutina más tarde (motor ya con `SET_RATIO` recalibrado), el umbral
detectado salió en `1093`, no `1061` — y la curva paso a paso mostró
que la transición **no siempre es un escalón limpio**: hubo subida
sostenida y real (`14.8`/`7.3`/`26.8`/`26.1`/`27.0` RPM) durante ~6
pasos antes de confirmar la detección. Con el margen original de 3
pasos, el `SERVO_PULSO_MIN` resultante (`1069`) quedó a mitad de esa
rampa, no en zona plana — el motor en reposo terminó rondando
`845-865` RPM en vez del relentí real (~826). Diagnosticado comparando
la curva `PASO` completa contra el barrido anterior (más abrupto) —
confirma que la zona muerta de este acelerador tiene un "codo" de
transición gradual, no un salto limpio. **Margen subido de 3 a 6
pasos (24µs→48µs)** en base a estos dos barridos reales (cubre la
rampa observada sin perjudicar el caso abrupto). Corregido en el
sitio manualmente ese mismo día (`SERVO_PULSO_MIN=950`, confirmado sin
acelerar el relentí) mientras se aplicaba el fix. Sigue pendiente
hacer el bump de AWS Lambda mencionado arriba. También sigue pendiente
(no construida, apenas discutida) una tercera medida de seguridad: si
el PID entra en oscilación sostenida por más de ~30s, desactivarlo
automáticamente (bajar a `SET_RPM=0`, no de a poco — ya se confirmó en
campo que bajar gradual no corta el *windup*) y reintentar el setpoint
original después de una pausa corta.

## 12. Secuencia de puesta en marcha en una máquina nueva

Orden recomendado al instalar el nodo en un motor por primera vez.
Cada paso depende de que el/los anteriores ya estén bien, así que no
conviene saltarlos ni cambiar el orden.

**Fase A — motor apagado (mecánico):**
1. `CALIB=1` — verificación visual manual de mínimos y
   máximos (sección 2.4/4.4). Sirve también como chequeo de seguridad:
   si a simple vista el servo ya se ve "abierto" más de la cuenta en
   la posición de reposo, es señal de que el `SERVO_PULSO_MIN` heredado
   (de otra máquina o calibración anterior) no aplica a este montaje
   mecánico nuevo, y hay que ajustarlo a mano antes de automatizar nada
   con el modo `4`.
2. `CALIB=2` — barrido automático sin restricción, para
   confirmar que el servo se mueve libre en todo el rango sin
   resistencia mecánica anómala.

**Fase B — motor encendido (sensores):**
3. `SET_RATIO` (o `SET_RATIO_AUTO`, ver abajo) — calibrar el tacómetro
   contra una referencia externa (tacómetro de mano). No basta una sola
   lectura en relentí: conviene tomar 2-3 lecturas repartidas en el
   rango de operación (relentí, un punto medio, uno alto si es seguro)
   y comparar el ratio implícito por cada una
   (`ratio = frecuenciaHz × 60 / RPM_referencia`). Si salen dentro de
   ~1-2% entre sí es solo ruido de la comparación — quedarse con
   cualquiera de ellas (o la última que se mandó, que es la que queda
   aplicada). Si hay una tendencia clara y consistente entre RPM bajas
   y altas, hay algo mecánico real (holgura/deslizamiento en la fuente
   de la señal) que un solo número fijo no puede corregir del todo; en
   ese caso priorizar el punto donde el PID va a operar la mayor parte
   del tiempo. La medición de `tacometro.c` usa Input Capture (período
   real en µs, no conteo de pulsos en ventana fija), así que no debería
   haber no-linealidad propia de la medición del firmware entre
   relentí y RPM altas.

   **`SET_RATIO_AUTO` (parámetro 25, agregado 2026-09-03)** hace el
   cálculo por vos: en vez de mandar el ratio ya despejado a mano, se
   manda el RPM que marca el tacómetro de referencia en ese instante
   (`x10`, mismo formato que `SET_RPM`) y el firmware calcula y aplica
   `SET_RATIO` directo, usando su propia `frecuenciaHz` del momento
   (`tacometro.c:109`, la misma fórmula pero despejada). El ACK
   devuelve el ratio resultante (no el RPM que mandaste), así que se ve
   directo si quedó razonable. Repetir el comando en 2-3 puntos de RPM
   durante la puesta en marcha sigue siendo la forma de detectar si hay
   un problema mecánico real: si el ratio resultante cambia bastante
   entre un punto y otro, no es la medición del firmware (ver arriba)
   — es la fuente de la señal físicamente. `OUT_OF_RANGE` si se manda
   `0` o si el motor no tiene lectura válida en ese momento. Comando
   serial equivalente: `SET_RATIO_AUTO <rpm>`. **Pendiente**: falta el
   alta en `send_downlink.py`'s `PARAMETER_TABLE` (ID 25, escala x10)
   para poder mandarlo por downlink LoRa, no solo por serial.
4. `ALPHA` (o `ALPHA_AUTO`, ver abajo) — fijar el filtro EMA de RPM.
   Debe quedar fijo **antes** de los modos `4`/`5`/`6`/`10`: cambiarlo
   después de sintonizar el PID le cambia la dinámica que el PID ya
   aprendió a manejar (más o menos lag en la señal), obligando a
   re-sintonizar. En la práctica esto pesa poco: el filtro se actualiza
   en cada pulso del tacómetro (cada pocos ms a las RPM típicas de este
   motor), no cada 200ms como el log `PID_TEST` — con `ALPHA=0.35` el
   retraso que agrega el filtro es del orden de milisegundos,
   insignificante frente a la dinámica real del lazo (cientos de ms).
   `ALPHA` no tiene un valor "correcto" único como `SET_RATIO` — es un
   compromiso de diseño entre ruido y velocidad de respuesta, así que
   no hace falta recalcularlo en cada instalación si el default ya
   funciona bien.

   **`CALIB=3` (agregado 2026-09-03) — calibración
   automática de `ALPHA`.** A diferencia de `SET_RATIO_AUTO`, acá no
   hay ninguna referencia externa que darle — el firmware mide su
   propio ruido, así que no necesita cargar ningún valor: se manda
   igual que los modos `4`/`7` (calibración de zona muerta y de
   ralentí, respectivamente, renumerados 2026-09-23, antes eran los
   valores `5`/`4`, ver secciones 10/11) (`CALIB 3`, solo aceptado
   viniendo de modo `0`, con o sin el motor operando — si no está
   operando, la rutina espera). Con el motor ya operando, toma ~8s de
   muestras de `RPM_instantánea` sin filtrar (cada 200ms), calcula su
   desviación estándar (algoritmo de Welford, evita el error numérico
   de restar cuadrados de RPM que ya son grandes), y despeja el `ALPHA`
   que dejaría la señal filtrada dentro de un objetivo fijo de `±10
   RPM` (`ALPHA_CAL_OBJETIVO_DESVIACION_RPM` en `main.c`), usando la
   atenuación de ruido de un filtro EMA: `alpha = 2r²/(1+r²)` con
   `r = objetivo/desviación_cruda`. Se aplica y persiste directo (mismo
   `CalibFlash_SetAlphaFiltro()` que usa `ALPHA`), sin que el operador
   tenga que copiar ningún número, y al terminar vuelve sola a
   `CALIB=0` (igual que `4`/`7`), disparando el mismo
   "ACK" por LoRa (`CalibFlash_ForzarReporte()`). **Techo de seguridad
   (`ALPHA_CAL_TECHO_ALPHA=0.5`, agregado 2026-09-03 tras un caso real
   en campo)**: si el ruido crudo medido en la ventana de 8s ya sale en
   o por debajo del objetivo, la fórmula tiende a `alpha≈1.0`
   (prácticamente sin filtro) — pasó en la primera prueba real
   (`desviación_cruda=10.08`, casi igual al objetivo de `10`, dio
   `alpha=0.99` antes de este fix). El riesgo es que una ventana de 8s
   puede simplemente no capturar alguna de las ráfagas de perturbación
   periódicas (~15-25s) ya conocidas en este motor — "se vio limpio en
   esos 8 segundos" no es garantía de que no haga falta nada de
   suavizado. El techo evita aplicar un `alpha` más agresivo que `0.5`
   sin importar qué tan bajo salga medido el ruido. Progreso en el log
   `ALPHA_CAL,...` (`INICIO`/`MIDIENDO`/`COMPLETO`/`ABORTADO` si el
   motor se detiene a mitad de la ventana — se descarta el progreso
   pero se sigue en modo `3` esperando a que arranque de nuevo, mismo
   criterio que `4`/`7` — /`ERROR` si no hubo muestras suficientes). No
   reemplaza a `ALPHA` directo — sigue haciendo falta para
   cargar/restaurar un valor ya conocido desde la base de datos sin
   tener que correr la medición de nuevo (mismo argumento que
   `SET_RATIO` vs `SET_RATIO_AUTO`). **Pendiente**: falta el bump de
   `send_downlink.py`'s `PARAMETER_TABLE["CALIB"]["max"]`
   hasta `10` (mismo patrón que las subidas anteriores) y una prueba
   real en campo (por ahora solo compila limpio).

**⚠️ Reordenado 2026-09-23 (segunda renumeración del día)**: las Fases
C/D/E de abajo cambiaron de orden respecto a versiones anteriores de
este README. El criterio (confirmado por el usuario): zona muerta
(`SERVO_PULSO_MIN`), curva de ganancia y la sintonización del PID
INTERNO (lazo de RPM) dan el **mismo resultado con o sin la
tubería/carga hidráulica conectada** — son propiedades mecánicas del
servo/varilla y de la respuesta de RPM del motor, no del sistema
hidráulico. El ralentí (`RPM_MIN`), en cambio, **sí cambia según haya o
no carga conectada** (un motor diesel gobierna distinto bajo carga) —
confirmado en campo, hubo que repetir esa calibración después de
conectar la tubería. Y los PID de presión (local/remoto) directamente
no tienen sentido sin presión real que medir. Por eso conviene hacer
zona muerta + curva de ganancia + PID interno TODO antes de conectar
la tubería (minimiza cuántas veces hay que conectar/desconectar), y
recién ahí conectar la tubería UNA sola vez, dejando ralentí y los dos
PID de presión para después, ya con carga real.

**Fase C — motor encendido, SIN carga hidráulica (calibración
automática del servo y del lazo interno):**
5. `CALIB=4` — calibración automática de zona muerta del
   servo (`SERVO_PULSO_MIN`, sección 11; renumerado 2026-09-23, antes
   era el valor `5`).
6. **`CALIB=5` (opcional, diagnóstico) — mapeo de curva de
   ganancia.** Renumerado 2026-09-23, antes era el valor `6`. Agregado
   2026-09-03 a raíz de una discusión sobre la geometría del mecanismo
   brazo-varilla que mueve la cremallera de la bomba: el brazo del
   servo gira, pero la cremallera se mueve linealmente (o casi) — esa
   conversión ángulo→posición no es constante en todo el recorrido
   (parecido a un mecanismo biela-manivela), así que la ganancia real
   del sistema (cuánta RPM sube por cada µs de pulso) varía según en
   qué parte del recorrido esté el servo, no solo por la zona muerta ya
   conocida. Esta rutina barre el pulso hacia arriba en pasos de `8µs`
   (mismo tamaño que la zona muerta, a propósito — no uno más grande,
   para no arriesgarse a pasarse del techo de RPM de un salto grande en
   la zona de más ganancia) desde `SERVO_PULSO_MIN`, en **lazo abierto**
   (sin PID), con una espera de asentamiento de `2.5s` en cada paso,
   registrando el par `(pulso, RPM real)` — hasta llegar a `RPM_MAX`
   (el techo real ya configurado para esa instalación; hasta
   2026-09-28 usaba un techo fijo propio,
   `GANANCIA_CAL_RPM_TECHO=1500.0f`, pensado específicamente para este
   escenario SIN CARGA — eliminado a pedido del usuario para no pedirle
   configurar dos techos de RPM distintos, ver sección 4.4) o hasta
   `SERVO_PULSO_MAX`. Aborta si el motor se detiene a mitad del
   barrido, o si un solo paso sube la RPM más de
   `GANANCIA_CAL_SALTO_ANORMAL_RPM` de golpe (protege contra pasarse
   del techo de un salto grande, además de detectar lecturas
   anómalas). **Por ahora esta rutina SOLO mide y loguea la curva — no
   aplica ninguna corrección al PID directamente**, pero su log
   alimenta `pid_tuning.py auto --loop interno --ganancia-log` (paso 8)
   para sacar una `K` peor caso robusta, la misma metodología real que
   se usó a mano para llegar a `Kp=0.047, Ki=0.11` (ver sección 9,
   "Re-sintonización para el montaje directo del servo"). Log:
   `GANANCIA_CAL,INICIO`
   / `BARRIENDO` / `PASO,pulso=<us>,rpm=<rpm>,salto=<rpm>` /
   `COMPLETO,motivo=techo_rpm_alcanzado,...` /
   `ABORTADO,motivo=motor_se_detuvo` /
   `ABORTADO,motivo=salto_rpm_anormal` /
   `ERROR,motivo=servo_pulso_max_alcanzado_sin_llegar_al_techo`.
7. **`CALIB=6` (opcional, agregado 2026-09-23 en la
   segunda renumeración) — auto-escalón del lazo INTERNO de RPM (PID#1)
   — O el escalón manual equivalente con `MODO=3` (MANUAL_BANCO,
   mandando `SET_RPM` a mano) — ver paso 9** (⚠️ actualizado
   2026-09-29: ya no existe `CALIB=10`, ver sección 4.4):
   cualquiera de los dos deja un log `PID_TEST` para el paso siguiente.
   ⚠️ **Este paso es
   INDEPENDIENTE del paso 6 (`CALIB=5`, curva de
   ganancia)** -- `=6` NO corre el barrido de ganancia de nuevo ni lo
   reemplaza, es una rutina completamente aparte (solo el escalón de
   `SET_RPM`). Hacen falta LOS DOS logs por separado (el de este paso Y
   el del paso 6) para el paso 8 de abajo. `CALIB=6`
   automatiza mandar el escalón: requiere `MODO=4` y el motor operando,
   guarda `PID_RPM_KP/KI` y los pisa a `1.0`/`0.0` fijo (agregado
   2026-09-25 -- restaura los valores guardados al salir, por cualquier
   motivo, sin que el operador tenga que bajarlos a mano antes ni
   después), comanda `SET_RPM = RPM_MIN + 200` RPM fijo
   (`INTERNO_CAL_ESCALON_RPM`) y lo deja correr una ventana FIJA de 60s
   (`INTERNO_CAL_DURACION_MS`), logueando `PID_TEST` con la misma
   cadencia de 5Hz que la sesión manual. Se aborta y resetea `SET_RPM`
   a `0.0` si el motor se detiene o `MODO` deja de ser `4`, y vuelve
   solo a `CALIB=0` al terminar o abortar. Log:
   `INTERNO_CAL,INICIO,...` / `INTERNO_CAL,ESCALON,...` /
   `INTERNO_CAL,COMPLETO,duracion_s=...` /
   `INTERNO_CAL,ABORTADO,motivo=...`. Ver sección 4.4/9.
8. ⚠️ **Este paso se corre en tu computadora, no en el TID** -- el
   firmware ya hizo su parte (pasos 6 y 7), solo generó los dos logs
   por serial. Acá los combinás vos: correr `pid_tuning.py auto --loop
   interno --log <PID_TEST del paso 7> --ganancia-log <GANANCIA_CAL del
   paso 6>` para obtener una recomendación de `Kp`/`Ki` — ver sección 9
   ("Automatizando el escalón") para el detalle de por qué conviene
   preferir la `K` del mapeo de ganancia (paso 6) sobre la que sale de
   identificar el escalón en lazo cerrado. **`--kp` ya no hace falta
   pasarlo** (agregado 2026-09-25): default `1.0` para `--loop interno`,
   porque `CALIB=6` siempre se hace con `PID_RPM_KP=1`/`PID_RPM_KI=0` fijo -- el
   propio firmware guarda lo que hubiera en `PID_RPM_KP/KI` al entrar a
   `=6`, los pisa a `1.0`/`0.0` durante la prueba, y restaura los
   valores guardados al salir (agregado 2026-09-25, ver paso 7 y
   sección 4.4 — antes esto era una convención procedimental, ahora es
   automático) -- solo especificalo a mano si alguna vez usás un valor
   distinto de `1.0` para el escalón.
9. ⚠️ **Actualizado 2026-09-29** (antes decía `MODO=4`+`CALIB=10`
   -- eso nunca funcionó del todo bien, porque `SET_RPM` exige
   `MODO=3` exacto para aceptarse, no `MODO=4`, ver sección 4.3):
   cargar `PID_RPM_KP`/`PID_RPM_KI` (sección 9) con `MODO=CALIBRACIÓN` +
   `CALIB=0` (ningún `CALIB` activo, ver sección 4.4), y
   LUEGO cambiar a `MODO=3` (MANUAL_BANCO) para mandar `SET_RPM` a
   mano y observar la respuesta real (con o sin haber usado el paso
   7/8 antes, este paso sigue siendo obligatorio: la recomendación de
   `auto` se confirma en el motor real antes de confiar en ella). Si
   hace falta ajustar la ganancia de nuevo tras observar, volver a
   `MODO=4`+`CALIB=0` para cambiarla y otra vez a `MODO=3` para
   reprobarla -- ya no se puede ajustar en caliente sin salir de
   `MODO=3` (a diferencia de como funcionaba con el extinto
   `CALIB=10`). Con esto el lazo interno queda validado,
   todavía SIN carga hidráulica conectada -- sin importar si el
   equipo va a operar después por presión local o remota, es el mismo
   lazo en los dos casos, no cambia.

**— Conectar la tubería/aspersor físicamente —**
10. Recién acá se conecta la carga hidráulica real, UNA sola vez. Todo
    lo de arriba (Fase C) dio el mismo resultado con o sin ella; todo lo
    de abajo (Fase D) la necesita para tener sentido.

**Fase D — motor encendido, CON carga hidráulica (ralentí real y
sintonización de los lazos de presión, `MODO=1`/`MODO=2`, agregado
2026-09-14):**
11. `CALIB=8` — calibración automática de ralentí
    (`RPM_MIN`, sección 10; renumerado 2026-09-23, antes era el valor
    `4`, y de nuevo 2026-09-29, antes era el valor `7`). Se hace recién ACÁ, y no antes junto con la zona muerta,
    porque el ralentí real de un motor diesel depende de si hay carga
    conectada (confirmado en campo) -- a diferencia de la zona muerta y
    la curva de ganancia (Fase C), que no dependen de eso.

12. ⚠️ **Primer llenado manual — SOLO la primera vez que se presuriza
    ESA tubería en ESA instalación** (agregado 2026-09-24, ver también
    la nota de riesgo del paso 14): con `MODO=3` (MANUAL_BANCO --
    corregido 2026-09-29, `SET_RPM` exige `MODO=3` exacto, no `MODO=4`,
    ver sección 4.3), subir `SET_RPM` a
    mano en incrementos chicos, esperando y observando/escuchando la
    tubería entre cada uno, hasta confirmar la línea llena de agua
    (sin aire atrapado) y una lectura de presión estable en `RPM_MIN`.
    Con la línea vacía o con aire, CUALQUIER cambio de RPM es más
    peligroso que el mismo cambio ya con la línea llena y presurizada
    (riesgo de golpe de ariete), sin importar qué tan chico sea el
    paso -- por eso este paso es manual y no se automatiza: el
    firmware no tiene forma de detectar "hay aire en la línea" por sí
    solo. En una recalibración posterior de una máquina que ya operó
    (tubería ya llena de antes), este paso se puede saltar.
13. Definir y mandar `PRESION_OBJETIVO_LOCAL` (ID 23) — el PSI que se quiere
    sostener en operación — y `TIEMPO_LLENADO_S` (ID 33) — cuántos
    segundos tarda un llenado seguro de esta tubería, según ya lo sabe
    el operador. **Ambos son obligatorios ANTES del paso 14** (movido
    2026-09-28): `CALIB=9` ya no caracteriza a ciegas
    hasta un techo fijo -- usa estos dos valores para calcular su
    propio ritmo máximo seguro de subida de presión, así que necesita
    conocerlos de antemano.
14. **`CALIB=9` (opcional, agregado 2026-09-24, diseño
    definitivo 2026-09-28) — mapeo de ganancia en LAZO ABIERTO del
    PID#2 (presión local).** Reemplaza a la caracterización manual que
    tenía este paso en versiones anteriores del README -- equivalente
    de `CALIB=5` (curva de ganancia del PID#1, paso 6)
    pero para la relación RPM→presión: en `MODO=4` (sin
    `presion_pid.c`/`Kp`/`Ki` corriendo), mide la presión base real en
    `RPM_MIN`, y sube `SET_RPM` en una rampa continua PAUSADA por la
    presión real (nunca más rápido que el ritmo seguro que ya definen
    `PRESION_OBJETIVO_LOCAL`/`TIEMPO_LLENADO_S` del paso 13 -- ver sección 9
    para el detalle del freno), logueando `PRESION_GANANCIA_CAL,PASO,
    rpm=...,psi=...,salto=...` cada vez que la presión real avanza
    1 PSI. **Asume la tubería ya llena/presurizada (paso 12 ya
    hecho)** -- no es una rutina de primer llenado. Termina al llegar
    a `PRESION_OBJETIVO_LOCAL`, al llegar a `RPM_MAX_CARGA` sin alcanzarlo
    (respaldo de seguridad), o por timeout de último recurso; se
    aborta y resetea `SET_RPM` a `0.0` si el motor se detiene, `MODO`
    cambia, o el sensor de presión deja de ser válido, y vuelve sola a
    `CALIB=0` al terminar (`CALIB=9`). La duración real coincide con
    lo que el operador ya dijo que tarda el llenado (`TIEMPO_LLENADO_S`),
    o termina antes si la presión objetivo se alcanza con menos RPM.
    Log completo: ver sección 9. Solo mide y loguea -- el cálculo de
    ganancias sigue siendo con `pid_tuning.py auto --loop presion
    --ganancia-log <este log>` (ver sección 9), y alimenta el paso 15
    de abajo.
15. ⚠️ Actualizado 2026-09-29: con `MODO=CALIBRACIÓN` + `CALIB=0`
    (sección 4.4), mandar `PID_PSI_KP`/`PID_PSI_KI` (IDs 28/29)
    con valores conservadores (arrancan en `0`, sin efecto -- o la
    recomendación del paso 14 si ya se corrió `pid_tuning.py auto
    --loop presion` con ese log), y LUEGO pasar a `MODO=1` para
    observar si el lazo externo persigue `PRESION_OBJETIVO_LOCAL` sin
    oscilar, subiendo las ganancias de a poco si se arrancó en `0` (para
    subir de nuevo, volver a `MODO=4`+`CALIB=0`, ajustar, y otra vez a
    `MODO=1` para reprobar). Mismo criterio de sintonización que el
    paso 9, ver sección 4.3/9.
16. **`CALIB=10` (opcional, agregado 2026-09-23, renumerado
    en la segunda renumeración -- antes era el valor `7`, y de nuevo
    2026-09-29, antes era el valor `8`) —
    auto-escalón del lazo de presión LOCAL.** Alternativa/complemento a
    repetir el paso 15 a mano. ⚠️ **A diferencia del paso 15 (que corre
    con `MODO=1` real), `CALIB=10` exige `MODO=4` (CALIBRACIÓN) para
    aceptarse -- agregado 2026-09-24, decisión explícita del usuario:
    ningún `CALIB` corre con ningún otro `MODO`, sin
    excepciones.** Después del paso 15, pasar `MODO` de `1` a `4`
    antes de mandar `CALIB=10` -- el lazo de presión REAL sigue corriendo igual mientras tanto
    (`main.c` trata `MODO=4`+`CALIB=10` idéntico a `MODO=1` real para todo
    lo que importa: init del PID, rampa de llenado, falla de sensor,
    supervisor). Con `MODO=4` y el motor operando, automatiza mandar un
    escalón fijo de `PRESION_OBJETIVO_LOCAL` (+`PRESION_CAL_ESCALON_PSI`, 5
    PSI) y lo deja correr una ventana fija de `PRESION_CAL_DURACION_MS`
    (3 minutos, bajada de 10min el 2026-09-28 tras identificar el
    modelo real con `pid_tuning.py identify-presion` sobre un log de
    campo real: el escalón real se asienta en `L=11.4s` + ~4-5 veces
    `tau_cl=3.2s` ≈ 25-30s, no los `60-120s` que se habían asumido
    antes de tener ese dato -- 3min sigue dando ~6x de margen sobre
    ese asentamiento real), logueando `PRESION_PID_TEST` con la misma cadencia de 5Hz
    que en el paso 15. Si se pide antes de que `MODO=4` esté puesto o el
    motor esté operando, se queda esperando (no es un error) — solo se
    puede pedir viniendo de `CALIB=0`, no directo desde
    otro modo de calibración. Al terminar (o si el motor se detiene o
    `MODO` deja de ser `4` a mitad del escalón) vuelve solo a
    `CALIB=0` y restaura `PRESION_OBJETIVO_LOCAL` a su valor
    anterior. Log: `PRESION_CAL,INICIO,...` / `ESCALON,objetivo_base=
    <psi>,objetivo_nuevo=<psi>,duracion_s=<s>` / `COMPLETO,duracion_s=
    <s>` / `ABORTADO,motivo=motor_se_detuvo|modo_cambio`. **Solo
    automatiza mandar el escalón y esperar** — el cálculo de
    `PID_PSI_KP/KI` a partir del `PRESION_PID_TEST` resultante
    sigue siendo con `tools/pid_tuning/pid_tuning.py` (`auto --loop
    presion`, opcionalmente con `--ganancia-log <log del paso 14>`,
    ver sección 9) y la decisión de aplicar esas ganancias sigue siendo
    del operador, no es un lazo de auto-tuning cerrado. Siguiente paso:
    correr `pid_tuning.py auto` sobre el log capturado y, si las
    ganancias sugeridas se ven razonables, volver al paso 15 para
    cargarlas y confirmar en campo.
17. **`CALIB=11` (opcional, agregado 2026-09-25, ver sección 4.4/9) —
    validación de ganancias reales del lazo de presión LOCAL.**
    Equivalente de presión de `CALIB=7` (validación de ganancias reales
    del lazo INTERNO, sección 9) -- requiere `CALIB=9` (paso 14) y
    `CALIB=10` (paso 16) ya corridos, y `PID_PSI_KP/KI` ya cargados
    (paso 15). A diferencia de `CALIB=10`, NO fuerza `PID_PSI_KI=0`
    -- usa la ganancia YA cargada en flash, la recomendada por `auto`/
    `sugerir-kp-presion` y confirmada por el operador. Mismo escalón
    fijo de `PRESION_OBJETIVO_LOCAL` y mismo log `PRESION_PID_TEST` que
    `CALIB=10`, pero con una ventana de 4 minutos (en vez de 3) y un
    watchdog de oscilación sostenida en vivo (mismo criterio que su
    equivalente del lazo interno) -- aborta de inmediato
    (`SET_RPM=0.0`... `PRESION_OBJETIVO_LOCAL` restaurado) si detecta
    oscilación sostenida. El log resultante se le pasa directo a
    `pid_tuning.py compare --loop presion` (ver sección 9) para
    confirmar en campo el % de parecido contra el modelo simulado, en
    vez de "dejalo asentar 4 minutos y mirá cómo se ve" a mano. Al
    terminar (o abortar) vuelve solo a `CALIB=0` y restaura
    `PRESION_OBJETIVO_LOCAL` a su valor anterior, igual que `CALIB=10`.
18. Si este nodo también va a operar en `MODO=2` (gobernado por un
    aspersor remoto), definir primero `PRESION_OBJETIVO_REMOTO` (ID 30
    -- ahora es el setpoint real del PID, no opcional) y calibrar ahí
    mismo `PID_ASP_KP`/`PID_ASP_KI` (IDs 26/27)
    de la misma forma, siempre con `MODO=CALIBRACIÓN` +
    `CALIB=0` (ver sección 4.4). Mismo
    criterio de sintonización que el paso 15 (empezar conservador,
    subir de a poco observando la respuesta real).
19. **`CALIB=12` (opcional, agregado 2026-09-23, renumerado
    en la segunda renumeración -- antes era el valor `8`, y de nuevo
    2026-09-29, antes era el valor `9`) —
    auto-escalón del lazo REMOTO.** Mismo espíritu que el paso 16, pero
    para el lazo remoto: se prueba en banco con `MODO=4`
    (CALIBRACIÓN), no en `MODO=2`, porque para identificar la planta
    hace falta mandar el escalón de RPM directo, sin pasar por el PID
    de presión todavía sin calibrar. Con `MODO=4` ya puesto (o
    esperando si no) y el motor operando, automatiza mandar `SET_RPM`
    = `RPM_MIN` + `REMOTO_CAL_ESCALON_RPM` (200 RPM fijo) y loguea
    `REMOTO_PID_TEST` (evento-driven, una línea por cada reporte nuevo
    de `PRESION_REMOTO` del aspersor remoto). Termina al acumular
    `REMOTO_CAL_REPORTES_OBJETIVO` (15) reportes nuevos tras el
    escalón, o al vencer `REMOTO_CAL_TIMEOUT_MS` (60 minutos) de
    seguridad, lo que ocurra primero -- no puede usar una ventana fija
    como el paso 16 porque el aspersor reporta de forma muy irregular
    (20s a 20-25min según su propio estado, sección 4.3). Al terminar
    (o abortar por motor detenido/cambio de `MODO`) vuelve solo a
    `CALIB=0` y resetea `SET_RPM` a `0.0`. Log:
    `REMOTO_CAL,INICIO,...` / `ESCALON,set_rpm_base=<rpm>,
    set_rpm_nuevo=<rpm>,reportes_objetivo=<n>,timeout_s=<s>` /
    `REPORTE,n=<n>,presion=<psi>` / `COMPLETO,motivo=
    reportes_objetivo|timeout,reportes=<n>` /
    `ABORTADO,motivo=motor_se_detuvo|modo_cambio`. Igual que el paso
    16, solo automatiza el envío del escalón y la espera — el cálculo
    de ganancias sigue siendo con `pid_tuning.py` (`auto --loop
    remoto`, usando `identify-remoto` para la identificación en lazo
    abierto, ver sección 9) y la aplicación sigue siendo manual/con
    confirmación en campo. Siguiente paso: volver al paso 18 para
    cargar las ganancias sugeridas y confirmar.

**Fase F — entrar a operación:**
20. `CALIB=0` — cierra la sesión de sintonización,
    bloquea de nuevo todas las ganancias (`PID_RPM_KP/KI/KD`,
    `PID_PSI_KP/KI`, `PID_ASP_KP/KI`) contra
    cambios accidentales en operación normal.
21. `MODO` al valor que corresponda para este equipo en producción
    (`1` si gobierna por su sensor local, `2` si lo gobierna el
    aspersor remoto). Desde acá el sistema queda operando solo, sin
    más intervención del operador.

### Flujo de operación pura (equipo ya calibrado, sin volver a la fase 12)

Una vez hecha la puesta en marcha de arriba, este es el flujo que se
repite en cada arranque normal -- la calibración de la sección 12 no
se repite en cada operación, típicamente es anual o cuando haga falta
recalibrar algo puntual (ver sección 4.4/9). Con
`CALIB=0` siempre, **todas las ganancias quedan
bloqueadas** (`APPLY_ERROR` si se intenta cambiarlas) -- protegidas
justamente para que nadie las toque sin querer en operación normal.
El único parámetro que se toca día a día es `MODO`:

```
CALIB = 0 siempre (modo de calibración, ni se toca)

MODO=0 (ralentí)   ──┬──► MODO=1 (presión local): lazo en cascada
  estado de reposo/   │    corriendo solo -- presion_pid.c calcula
  seguro               │    setpointRpm, pid.c mueve el servo, sin
                        │    intervención del operador.
                        │
                        ├──► MODO=2 (remoto/aspersor): PID en cascada
                        │    evento-driven ya calibrado corriendo solo,
                        │    con watchdog de TIMEOUT_SIN_COMANDO_S si
                        │    deja de llegar PRESION_REMOTO fresca (sección 4.3).
                        │
                        └──► MODO=3 (manual/banco): uso operativo
                             puntual, ej. trasladar manguera/aspersor
                             -- el operador setea un RPM fijo un rato
                             y después regresa a 0, 1 o 2.
```

⚠️ **`MODO=4` (CALIBRACIÓN) queda A PROPÓSITO fuera de este diagrama** --
no es parte del día a día, es exclusivo de la sección 12 (puesta en
marcha/recalibración anual, con `CALIB != 0`). Un operador
de campo nunca debería necesitar tocarlo en operación normal.

El operador (o el switch físico selector, cuando exista -- sección 8)
solo toca `MODO`, nunca `CALIB` ni ninguna ganancia. Es
la separación de fondo entre los dos parámetros: `CALIB`
es el modo de banco/ingeniería (esta sección 12, uso esporádico);
`MODO` es el selector operativo del día a día. ⚠️ Las reglas exactas
de transición entre valores de `MODO` (ej. si hace falta pasar por `0`
antes de saltar entre 1/2/3) todavía están pendientes de definir, ver
sección 4.3/8 — hoy cualquier salto directo se acepta sin restricción.

⚠️ **`MODO` se fuerza solo a `0` (RALENTÍ) cada vez que el motor se
detiene** (agregado 2026-09-24, decisión explícita del usuario tras un
caso real de campo: el motor se queda sin diésel con `MODO=1`/`2`
puesto, tarda horas en reanudarse, y mientras tanto la tubería se
vacía/entra aire). Antes de este cambio, `MODO` persistía en RAM
mientras el TID no se reiniciara -- el control se retomaba solo al
reencender el motor, sin importar cuánto hubiera durado la parada. El
riesgo: la rampa de llenado (`TASA_LLENADO_PSI_S`) y `PresionPid_Init()`
solo se rearman con un **flanco de `MODO`** (ver `presionLocalActivoAhora`/
`Antes` en `main.c`), no con un flanco del motor -- si `MODO` nunca dejó
de ser `1` durante la parada, al reencender el PID arranca "en caliente"
contra una lectura de presión ya vieja/irreal y puede pedir RPM alta de
golpe, el mismo riesgo de golpe de ariete que `CALIB=9` evita durante
calibración, pero en operación normal y sin ninguna protección
corriendo. Con este forzado, **cualquier apagado del motor** (sin
diésel, fin de jornada, lo que sea) exige que el operador confirme
`MODO=1`/`2` a mano de nuevo al reencender -- se prioriza la seguridad
por sobre la comodidad de que el control se reanude solo. El switch
físico selector (cuando exista) tendría que tener esto en cuenta: un
selector que ya esté en la posición "1" no basta para rearmar el
control después de un apagado, haría falta que el operador lo mueva
(aunque sea a `0` y de vuelta a `1`) o un mecanismo equivalente.

**`SERVO_PULSO_MAX` queda fuera de esta secuencia automática a
propósito**: a diferencia del mínimo, no hay forma segura de detectar
"llegó al tope mecánico y generó resistencia" solo mirando RPM — haría
falta retroalimentación de corriente/torque del servo, que el hardware
actual no tiene. Se sigue calibrando a mano (`CALIB=1`).
Si en el futuro se agrega sensado de corriente al servo, ahí sí valdría
la pena automatizarlo.

### Corrección geométrica del mecanismo biela-manivela — REMOVIDA (2026-09-11)

Existió entre 2026-09-03 y 2026-09-11 una corrección de linealización
geométrica (`MECANISMO_*`, IDs 26-29, función `Mecanismo_CorregirPulso()`
en `main.c`) para compensar la relación no lineal ángulo-servo↔posición
real de la varilla del mecanismo biela-manivela original (manivela
`r=5.5cm`, varilla `L=30cm`, montada en el punto muerto — ecuación
`x(θ) = r·cos(θ) + √(L² − r²·sin²(θ))` y su inversa exacta, ley de
cosenos). Quedaba apagada por defecto (`MECANISMO_CORRECCION_ACTIVA=0`)
y nunca se activó una vez que se adoptó el montaje directo del servo
(branch `servo-directo`, ver más arriba en esta misma sección) — el
montaje directo no tiene esta no-linealidad geométrica (el servo
acopla 1:1 con la palanca), así que la corrección quedó permanentemente
inerte y se quitó del código por completo (parámetros, función, IDs
26-29 liberados — no reasignar sin confirmar que ningún nodo viejo en
campo los siga usando). El mecanismo biela-manivela + esta corrección
siguen intactos en la rama `main` como punto de retorno.