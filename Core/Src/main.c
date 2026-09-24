/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tacometro.h"
#include "calibracion_flash.h"
#include "rak3172.h"
#include "servo.h"
#include "pid.h"
#include "rtc_reloj.h"
#include "gps.h"
#include "comando_serial.h"
#include "presion.h"
#include "presion_voltaje.h"
#include "presion_pid.h"
#include "presion_pid_remoto.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INTERVALO_ENVIO_RPM_MS   30000U   // mandar RPM cada 30 segundos

/* Debounce del estado del motor (ENCENDIDO/APAGADO/ACTIVO) antes de
 * confirmarlo para telemetria/inicio_operacion -- ver comentario junto a
 * estadoCandidato en el superloop. Un motor diesel real tarda varios
 * segundos en pasar de detenido a régimen; 3s es sobrado para filtrar
 * una rafaga de ruido de <1s en el tacometro sin retrasar de forma
 * perceptible una transicion real. */
#define ESTADO_DEBOUNCE_MS   3000U

/* Auto-calibracion de ralenti (CALIBRAR_RALENTI, ver calibracion_flash.c) --
 * ventana total larga porque un motor frio puede arrancar bien por debajo
 * de su ralenti real y tardar en subir (ej. ~500 -> ~800 observado en
 * campo); de esa ventana total solo se promedian los ultimos segundos
 * (RALENTI_CAL_VENTANA_PROMEDIO_MS), descartando la rampa de calentamiento
 * de los primeros ~90s. RALENTI_CAL_MARGEN_RPM se suma al promedio antes de
 * aplicarlo como RPM_MIN, para que el ruido normal de medicion (+-10-15 RPM,
 * ver hallazgos de PID) no cruce el umbral por accidente. */
#define RALENTI_CAL_DURACION_TOTAL_MS     120000U
#define RALENTI_CAL_VENTANA_PROMEDIO_MS    30000U
#define RALENTI_CAL_MARGEN_RPM                30.0f

/* Auto-calibracion de SERVO_PULSO_MIN (CONTROL_HABILITADO=4, renumerado
 * 2026-09-23, antes era 5) -- barre el
 * pulso hacia arriba en pasos chicos desde el SERVO_PULSO_MIN actual,
 * buscando donde el motor empieza a acelerar de verdad (la "zona muerta"
 * mecanica del acelerador, observada en campo: unos grados de recorrido
 * sin ningun efecto antes de que el motor reaccione). Parametros pensados
 * para que el barrido sea autolimitado y de bajo riesgo -- ver README
 * seccion 10 para el analisis de riesgo completo. */
#define SERVOMIN_CAL_PASO_US                     4U    /* incremento por paso -- BAJADO de 8 a 4 (2026-09-07) al pasar a montaje directo del servo sobre el eje de la palanca: el recorrido util completo (relenti a fondo) paso a ser ~30 grados de servo en vez de los ~60 grados de la biela-manivela, es decir la MISMA variacion de RPM ahora ocurre en la mitad del rango de pulso -- la ganancia promedio por microsegundo se duplica como minimo, y probablemente iguala o supera el punto mas empinado que tenia la curva no lineal vieja. Revalidar con el primer barrido real: si "subida"/"salto" siguen saliendo grandes con este paso, bajarlo mas. */
#define SERVOMIN_CAL_TECHO_US                   180U    /* tope maximo sobre el SERVO_PULSO_MIN de entrada -- si se llega sin detectar nada, se aborta con error en vez de seguir subiendo a ciegas */
#define SERVOMIN_CAL_DWELL_MS                  2500U    /* espera de asentamiento antes de leer la RPM en cada paso */
#define SERVOMIN_CAL_BASELINE_MS               12000U    /* ventana para medir una linea base fresca de RPM antes de barrer */
#define SERVOMIN_CAL_UMBRAL_DETECCION_RPM        30.0f  /* cuanto debe subir la RPM sobre la linea base para contar como "empezo a acelerar" -- mayor al ruido normal de medicion (+-10-15 RPM) */
#define SERVOMIN_CAL_SALTO_ANORMAL_RPM  (SERVOMIN_CAL_UMBRAL_DETECCION_RPM * 4.0f) /* si un solo paso sube esto o mas, es un salto fuera de lo esperado -- abortar ya, no seguir */
#define SERVOMIN_CAL_CONFIRMACIONES_NECESARIAS    2U    /* lecturas seguidas por encima del umbral, en el mismo pulso, antes de dar por confirmada la deteccion */
#define SERVOMIN_CAL_MARGEN_PASOS                 6U    /* pasos de colchon hacia atras al aplicar el resultado, para no dejar SERVO_PULSO_MIN pegado justo al borde. SUBIDO de 3 a 6 (2026-09-03) tras confirmar en campo que la transicion NO siempre es un escalon limpio -- una corrida real mostro subida sostenida (14.8/7.3/26.8/26.1/27.0 RPM) durante ~6 pasos antes de confirmar deteccion, y 3 pasos de margen dejo el SERVO_PULSO_MIN resultante a mitad de esa rampa (RPM de reposo termino ~20-30 RPM por encima del relenti real). 6 pasos cubre esa rampa observada sin perjudicar el caso de transicion abrupta (el barrido original de esta misma zona muerta vino en puro ruido hasta justo antes del punto de deteccion). */

/* Auto-calibracion de ALPHA -- CONTROL_HABILITADO=3 (ver
 * calibracion_flash.c) -- mide la dispersion real de RPM_instantanea
 * (sin filtrar) durante una ventana corta con el motor ya estable, y
 * calcula el ALPHA necesario para que la RPM ya filtrada quede dentro
 * de un objetivo de ruido fijo (ALPHA_CAL_OBJETIVO_DESVIACION_RPM),
 * usando la relacion de atenuacion de un filtro EMA:
 * alpha = 2r^2/(1+r^2), con r = objetivo/desviacion_cruda. No mueve el
 * servo (mismo motivo que el modo 7 (ralenti) -- pidActivoAhora nunca es true en
 * este modo, cae solo en la rama segura de abajo). Ver README seccion 12.
 *
 * CASO BORDE encontrado en campo 2026-09-03: si el ruido crudo medido
 * ya sale por debajo/cerca del objetivo, "r" se acerca a 1 y el alpha
 * calculado se va casi a 1.0 (practicamente sin filtro) -- una ventana
 * de 8s puede simplemente no alcanzar a capturar alguna de las rafagas
 * de perturbacion periodicas (~15-25s) ya conocidas en este motor, asi
 * que "se ve limpio en la ventana" no es garantia de que no haga falta
 * nada de suavizado. ALPHA_CAL_TECHO_ALPHA pone un techo -- nunca se
 * aplica un alpha mas agresivo (mas cercano a 1) que ese, sin importar
 * que tan bajo salga el ruido medido. */
#define ALPHA_CAL_VENTANA_MS                    8000U   /* duracion total de la ventana de muestreo */
#define ALPHA_CAL_INTERVALO_MUESTREO_MS          200U   /* cada cuanto se toma una muestra dentro de la ventana -- mismo periodo que el log PID_TEST */
#define ALPHA_CAL_OBJETIVO_DESVIACION_RPM        10.0f  /* objetivo de ruido en la RPM YA FILTRADA -- ver discusion README seccion 12 */
#define ALPHA_CAL_TECHO_ALPHA                     0.5f  /* nunca aplicar mas filtrado "agresivo" que esto (ver caso borde arriba) -- algo mas permisivo que el 0.35 ya validado, pero lejos de "sin filtro" */

/* Mapeo de curva de ganancia -- CONTROL_HABILITADO=5 (renumerado
 * 2026-09-23, antes era 6, ver calibracion_flash.c). Barre el pulso
 * hacia arriba en pasos chicos desde SERVO_PULSO_MIN, en lazo ABIERTO
 * (sin PID), registrando el par (pulso, RPM real) en cada paso -- a
 * diferencia de CONTROL_HABILITADO=4 (zona muerta, antes 5)
 * (que solo busca el borde de la zona muerta), esto mapea la forma
 * completa de la curva de ganancia en el rango de operacion real, para
 * diagnostico/diseno de una futura correccion de gain-scheduling en el
 * PID. Por ahora SOLO mide y loguea -- todavia no aplica ninguna
 * correccion (ver README seccion 12 y discusion sobre la geometria del
 * mecanismo brazo-varilla).
 *
 * Mismo paso de 8us que la zona muerta (no uno mas grande) a proposito:
 * el usuario pidio explicitamente no arriesgarse a pasarse de largo del
 * techo de RPM de un salto grande en la zona de mas ganancia -- 8us
 * tarda mas (~3 min peor caso) pero acota mejor el salto por paso. */
#define GANANCIA_CAL_PASO_US                       4U    /* mismo tamano que SERVOMIN_CAL, a proposito -- ver nota arriba. BAJADO de 8 a 4 junto con SERVOMIN_CAL_PASO_US (2026-09-07) por el mismo motivo: montaje directo, recorrido util a la mitad de grados que la biela-manivela -> ganancia por microsegundo al menos el doble. */
#define GANANCIA_CAL_DWELL_MS                    2500U   /* espera de asentamiento antes de leer la RPM en cada paso */
#define GANANCIA_CAL_RPM_TECHO                  1500.0f  /* nunca pasar de esta RPM durante el barrido -- elegido por el usuario como el limite seguro de esta prueba, no el maximo del motor */
#define GANANCIA_CAL_SALTO_ANORMAL_RPM            150.0f  /* si un solo paso sube la RPM esto o mas, frenar ya -- protege contra pasarse del techo de un salto Y detecta lecturas anormales */

/* Mapeo de curva de ganancia de PRESION -- CONTROL_HABILITADO=11 (agregado
 * 2026-09-24), equivalente de GANANCIA_CAL de arriba pero para el PID#2
 * (presion local, presion_pid.c) en vez del PID#1. Barre SET_RPM hacia
 * arriba en pasos chicos en MODO=3 (sin presion_pid corriendo -- lazo
 * ABIERTO para la relacion RPM->presion), registrando el par (RPM, PSI)
 * en cada paso. Por ahora SOLO mide y loguea, igual que GANANCIA_CAL.
 *
 * A diferencia de GANANCIA_CAL, ESTE barrido SI tiene un riesgo fisico
 * real que el otro no tiene: hay una tuberia con presion de verdad de
 * por medio, y un cambio de RPM demasiado brusco puede producir un golpe
 * de ariete. Dos decisiones de diseno en respuesta a esto, discutidas
 * con el usuario:
 *
 * 1. El SET_RPM de cada paso se RAMPEA (PRESION_GANANCIA_CAL_RAMPA_RPM_S),
 *    no se salta de golpe -- el DWELL de abajo es para que la PRESION
 *    asiente, no hace lento el cambio de RPM en si, y el PID#1 interno
 *    ya es rapido (Kp=0.047/Ki=0.11, llega a un SET_RPM nuevo en pocos
 *    segundos) -- sin esta rampa, el cambio de caudal real seria casi
 *    instantaneo pese al dwell largo. TASA_MAX_CAMBIO_RPM_S/_LLENADO_S
 *    (calibracion_flash.c) NO sirven para esto -- existen como
 *    parametros pero no estan cableados a ningun lazo de control
 *    todavia, asi que esta rutina rampea su propio SET_RPM interno.
 * 2. Esta rutina ASUME que la tuberia ya esta llena y en presion
 *    estable en RPM_MIN -- no es una rutina de primer llenado. El
 *    operador debe hacer el primer llenado a mano, de a poco, la
 *    primera vez que se presuriza una tuberia nueva, ANTES de correr
 *    este modo (ver README seccion 12, Fase D). El firmware no puede
 *    detectar "hay aire en la linea" por si solo.
 *
 * Techo bajado de un +15 PSI inicial a +10 PSI (2026-09-24, pedido del
 * usuario): mismo poder de identificacion con menos presion total y
 * menos tiempo de rutina. */
#define PRESION_GANANCIA_CAL_PASO_RPM             200.0f  /* incremento de SET_RPM objetivo en cada paso */
#define PRESION_GANANCIA_CAL_RAMPA_RPM_S            1.7f  /* velocidad MAXIMA a la que este modo sube su propio SET_RPM interno -- ~50 RPM/30s, un paso de 200 RPM tarda ~2min en llegar, gradual en vez de un salto instantaneo */
#define PRESION_GANANCIA_CAL_DWELL_MS            90000U    /* asentamiento por paso TRAS terminar la rampa -- tau real del lazo de presion 60-120s medido */
#define PRESION_GANANCIA_CAL_BASE_PROMEDIO_MS     5000U    /* ventana para promediar la PSI base en ralenti antes de arrancar a barrer */
#define PRESION_GANANCIA_CAL_DELTA_TECHO_PSI       10.0f  /* techo = presion base + esto, NO un PSI absoluto -- la base varia por instalacion */
#define PRESION_GANANCIA_CAL_SALTO_ANORMAL_PSI      4.0f  /* si un solo paso sube la presion esto o mas, frenar ya -- mismo criterio que GANANCIA_CAL_SALTO_ANORMAL_RPM */
#define PRESION_GANANCIA_CAL_TIMEOUT_MS          900000U  /* 15 min, red de seguridad de ultimo recurso -- con rampa+dwell la rutina completa estima 13-18min, este timeout no deberia dispararse en un barrido normal */


/* El RTC de este nodo corre del LSI interno (~32kHz nominal, sin cristal
 * externo -- ver hallazgos de hardware), que no esta calibrado ni
 * compensado por temperatura. Deriva medida en campo: ~0.8% (equivale a
 * ~12 min/dia sin correccion). Por eso se resincroniza periodicamente
 * en vez de una sola vez al arrancar -- ver bloque de sincronizacion en
 * el superloop.
 *
 * Fuente PRIMARIA: hora UTC del propio GPS (satelital, mas precisa, y
 * no compite por el canal AT del RAK3172 ni gasta duty cycle) -- se
 * intenta cada INTERVALO_RESYNC_GPS_MS si hay fix vivo. Ese intervalo
 * se dejo igual al de envio de RPM (30s) porque intentarlo no cuesta
 * nada -- el modulo SIM7600X igual solo refresca su propio reporte
 * cada 10s (AT+CGPSINFO=10), asi que preguntar mas seguido no trae
 * dato mas fresco.
 * Fuente de RESPALDO: DeviceTimeReq por LoRaWAN -- compite libremente
 * con el GPS para el primer sync (el que llegue primero gana). Una vez
 * que YA hubo un sync exitoso (de cualquier fuente), el respaldo por
 * LoRa se limita a activarse solo si pasan INTERVALO_FALLBACK_LORA_MS
 * sin ningun resync exitoso nuevo (ej. GPS sin vista al cielo por un
 * rato largo) -- para no competir por el canal AT del RAK3172 sin
 * necesidad mientras el GPS este sincronizando bien. */
#define INTERVALO_RESYNC_GPS_MS      INTERVALO_ENVIO_RPM_MS   // intentar resync por GPS cada vez que se manda RPM (30s)
#define INTERVALO_FALLBACK_LORA_MS   (30UL * 60UL * 1000UL)   // forzar resync por LoRaWAN si no hay exito en 30 min

/* El propio RAK3172 ya reintenta el join 8 veces cada 10s dentro de UN
 * solo comando (ver AT+JOIN=1:0:10:8 en RAK3172_Join(), ~80s en total),
 * pero si esas 8 fallan (o el modulo se queda sin cobertura un rato)
 * nada volvia a pedir el join de nuevo -- el nodo quedaba sin unirse
 * para siempre. 2 min da margen sobre esos ~80s antes de reintentar. */
#define INTERVALO_REINTENTO_JOIN_MS  (120UL * 1000UL)
/* ⚠️ SUPUESTO / placeholder: por ahora hardcodeado a 1 (equivale a
 * "DSL-0001"). Cuando exista mas de un nodo motor, resolver esto desde
 * CalibFlash_GetNodeId() (ID 18, NODE_ID) en vez de una constante. */
#define MOTOR_ID_NUMERIC   1U

/* Codigos de estado -- DEBEN coincidir exactamente con ESTADO_NOMBRES
 * de decoder.py del lado AWS. */
#define ESTADO_ACTIVO     1U  /* motor operando Y con presion de salida real (ver presion.c) */
#define ESTADO_APAGADO    2U
#define ESTADO_ENCENDIDO  3U  /* motor girando, sin presion de salida suficiente aun (arranque/sin carga) */

/* Umbral de presion (PSI) que distingue ACTIVO de solo ENCENDIDO.
 * ⚠️ SUPUESTO / placeholder sin dato de campo todavia -- ajustar una
 * vez que se tenga una sesion real con el sistema hidraulico cargado
 * (ver README seccion 8). */
#define PRESION_UMBRAL_ACTIVO_PSI  5.0f

/* Supervisor de "no se puede llegar a PRESION_OBJETIVO" en MODO=1 (ver
 * el bloque MODO mas abajo, y README secciones 4.3/8/12). Cada "intento
 * fallido" es una ventana sostenida de esta duracion con el lazo
 * saturado en RPM_MAX y la presion fuera de tolerancia -- tras
 * PRESION_SUPERVISOR_INTENTOS_MAX ventanas seguidas, se imprime una
 * alerta por monitor serial Y se manda por LoRa (ALERTA_PRESION_OBJETIVO_NO_ALCANZADA,
 * ver CodigoAlerta_t mas abajo, agregado 2026-09-23; ver README
 * seccion 8). */
#define PRESION_SUPERVISOR_VENTANA_MS            30000U
#define PRESION_SUPERVISOR_INTENTOS_MAX              3U
#define PRESION_SUPERVISOR_TOLERANCIA_FRACCION     0.05f

/* Retroceso activo del supervisor de MODO=2 (agregado 2026-09-23,
 * escenario real: fuga entre el motor y el aspersor -- el aspersor
 * nunca alcanza su propia presion objetivo por mas RPM que se le pida,
 * asi que el lazo se queda pegado en RPM_MAX indefinidamente sin
 * lograr nada, gastando combustible/motor por gusto). A diferencia del
 * supervisor de MODO=1 (solo alerta, nunca actua), este SI retrocede
 * activamente entre intentos: cuando un reporte confirma que sigue
 * saturado y lejos del objetivo, se fuerza "sin comandar" (mismo
 * mecanismo que la rama de ralenti, sin cambiar MODO de verdad) durante
 * este tiempo antes de volver a intentar -- decision explicita del
 * usuario, distinta del criterio original ("no es una proteccion, solo
 * deteccion") que sigue aplicando para MODO=1. */
#define PRESION_SUPERVISOR_REMOTO_RETROCESO_MS   20000U

/* Cantidad de intentos (aceleraciones que se retroceden) antes de la
 * alerta+ralenti final en MODO=2 -- CONSTANTE APARTE de
 * PRESION_SUPERVISOR_INTENTOS_MAX (que sigue en 3, sin tocar, para
 * MODO=1) a pedido explicito del usuario 2026-09-23: quiere 3
 * aceleraciones reales retrocedidas antes de rendirse, no 2 -- con
 * intentosFallidos alcanzando este valor recien en el 4to fallo. */
#define PRESION_SUPERVISOR_REMOTO_INTENTOS_MAX       4U

/* Auto-escalon INTERNO (PID#1) -- CONTROL_HABILITADO=6 (NUEVO,
 * agregado 2026-09-23 segunda pasada, ver mas abajo junto al log
 * PID_TEST). Mismo espiritu que PRESION_CAL/REMOTO_CAL: automatiza
 * SOLO mandar el escalon de SET_RPM en MODO=3 y esperar un tiempo
 * fijo -- el calculo de Kp/Ki sigue siendo con
 * tools/pid_tuning/pid_tuning.py (auto --loop interno --ganancia-log),
 * esto no aplica nada solo. Escalon +200 RPM sobre RPM_MIN (mismo
 * tamano que REMOTO_CAL_ESCALON_RPM, por consistencia). Duracion FIJA
 * de 60s -- generosa para este lazo (tau tipico de fracciones de
 * segundo a pocos segundos, muy por debajo de los 10min/60min que
 * necesitan los lazos de presion). */
#define INTERNO_CAL_ESCALON_RPM                  200.0f
#define INTERNO_CAL_DURACION_MS                 60000U   /* 1 minuto */

/* Auto-escalon de presion local -- CONTROL_HABILITADO=8 (renumerado
 * 2026-09-23, segunda pasada -- antes 7; ver mas abajo, junto al log
 * PRESION_PID_TEST), agregado 2026-09-23. Automatiza SOLO
 * el envio del escalon y la espera (mandar PRESION_OBJETIVO+delta y
 * dejarlo un rato fijo) -- el calculo de Kp/Ki sigue siendo con
 * tools/pid_tuning/pid_tuning.py (auto --loop presion) sobre el log
 * PRESION_PID_TEST resultante, y la decision de aplicar esas ganancias
 * sigue siendo del operador. Valores confirmados por el usuario
 * 2026-09-23: mismo tamano de escalon que ya se uso en la sesion real
 * de sintonizacion (README seccion 9, "1.36 PSI de un escalon de 5
 * PSI"), duracion FIJA (no detecta "asentado" solo, mismo criterio
 * simple que ya usa ALPHA_CAL con su ventana fija de 8s) de 10min --
 * de sobra (~6-7x el tau real conocido, 60-120s, ver README). */
#define PRESION_CAL_ESCALON_PSI                    5.0f
#define PRESION_CAL_DURACION_MS                600000U   /* 10 minutos */

/* Auto-escalon remoto -- CONTROL_HABILITADO=9 (renumerado 2026-09-23,
 * segunda pasada -- antes 8; ver mas abajo, junto al log
 * REMOTO_PID_TEST), agregado 2026-09-23. Mismo espiritu que
 * PRESION_CAL_* de arriba, pero el criterio de fin NO puede ser tiempo
 * fijo -- un reporte del aspersor puede tardar hasta 20-25min, asi que
 * termina por la que ocurra primero entre "ya llegaron suficientes
 * reportes nuevos" (REMOTO_CAL_REPORTES_OBJETIVO) o un timeout de
 * seguridad generoso (REMOTO_CAL_TIMEOUT_MS) para no dejar el motor
 * corriendo indefinidamente sin supervision si el aspersor deja de
 * reportar. Valores confirmados por el usuario 2026-09-23. */
#define REMOTO_CAL_ESCALON_RPM                   200.0f
#define REMOTO_CAL_REPORTES_OBJETIVO                15U
#define REMOTO_CAL_TIMEOUT_MS                  3600000U   /* 60 minutos */

/* Codigos de alerta mandados por LoRa en el byte 27 del uplink LIVE
 * (ver RAK3172_EnviarUplinkLive(), payload de 28 bytes) -- agregado
 * 2026-09-23 porque las alertas de arriba/abajo solo se logueaban por
 * serial hasta entonces, nadie se enteraba en campo si no estaba
 * conectado fisicamente en ese momento. codigoAlertaActual es un
 * SNAPSHOT del estado actual (0 = sin alerta), se manda en CADA uplink
 * LIVE (periodico o forzado). Para saber CUANDO ocurrio una transicion
 * (no solo que esta activa), cada punto que cambia codigoAlertaActual
 * llama a CalibFlash_ForzarReporte() -- eso dispara un uplink LIVE
 * inmediato (ver bloque de envio mas arriba), y su fechaHoraLocal es
 * el timestamp del evento. Los 4 valores corresponden 1 a 1 con las
 * alertas que ya existian (ver bloques MODO=1/2 mas abajo); si se
 * agrega una alerta nueva, sumarla aca Y a NOMBRES_ALERTA en
 * decoder.py del lado AWS. */
typedef enum {
    ALERTA_SIN_ALERTA                          = 0,
    ALERTA_PRESION_OBJETIVO_NO_ALCANZADA       = 1, /* supervisor MODO=1 */
    ALERTA_MODO_REMOTO_OBJETIVO_NO_ALCANZADO   = 2, /* supervisor MODO=2 (fuga) */
    ALERTA_SENSOR_PRESION_LOCAL_FALLA          = 3, /* MODO=1, sensor invalido */
    ALERTA_MODO_REMOTO_SIN_PRESION_VALIDA      = 4  /* watchdog TIMEOUT_SIN_COMANDO_S en MODO=2 */
} CodigoAlerta_t;

/* Coordenadas fijas del sitio -- ultimo respaldo si el GPS nunca ha
 * conseguido fix (ni en este arranque ni en ninguno anterior, ver
 * CalibFlash_GetUltimaLatitudConocida()). Con el GPS ya instalado,
 * este valor solo se usaria en el primerisimo arranque sin vista al
 * cielo. */
#define LATITUD_FIJA    14.27387764641955f
#define LONGITUD_FIJA  -91.09260397779028f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* Alerta activa AHORA MISMO (ver CodigoAlerta_t arriba) -- file-scope
 * porque se lee en el bloque de envio del uplink LIVE (mas arriba en
 * el loop que donde se calcula/actualiza, ver bloques MODO=1/2 mas
 * abajo) y se escribe ahi. Un local estatico dentro del loop no
 * serviria: C exige que este declarado antes de su primer uso textual,
 * y el envio del uplink ocurre antes en el codigo fuente que el calculo
 * de las alertas. */
static CodigoAlerta_t s_codigoAlertaActual = ALERTA_SIN_ALERTA;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* Causa del reinicio -- leida ANTES de limpiar las banderas (el HAL
   * las deja puestas de un reset a otro hasta que se limpian a mano),
   * para diagnosticar reinicios espontaneos sin depurador conectado
   * (ver hallazgos de hardware sobre resets por ground-bounce/ruido
   * del riel 3.3V). BOR+PIN juntos = ciclo de energia real (arranque
   * en frio); PIN solo (sin BOR) = NRST se disparo con la alimentacion
   * ya arriba (boton de reset, o ruido/glitch en esa linea). Se limpian
   * al final para que el PROXIMO reset se vea limpio, no acumulado con
   * este. */
  printf("Causa del reset:%s%s%s%s%s%s%s\r\n",
         __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)  ? " PIN(NRST)" : "",
         __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)  ? " BOR(brownout/power-on)" : "",
         __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)  ? " SOFTWARE(NVIC_SystemReset)" : "",
         __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? " IWDG(watchdog independiente)" : "",
         __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) ? " WWDG(watchdog de ventana)" : "",
         __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) ? " LOW_POWER(salida ilegal de Stop/Standby)" : "",
         __HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST)  ? " OPTION_BYTES" : "");
  __HAL_RCC_CLEAR_RESET_FLAGS();

  CalibFlash_Init();
  Reloj_Init(&hrtc);

  /* Estimacion inicial del reloj, mientras no llegue la resincronizacion
     * real con la red -- mejor que el default de MX_RTC_Init() (2000). */
  uint32_t horaConocidaFlash = CalibFlash_GetUltimaHoraUtcConocida();
  if (horaConocidaFlash > 0U) {
	Reloj_CargarHoraAproximada(horaConocidaFlash);
  }

  Tacometro_Init(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

  Presion_Init(&hadc1);
  /* TEMPORAL (2026-09-14): canal de voltaje (GPT203, PA4/ADC2) mientras se
   * resuelve el circuito de acondicionamiento del lazo 4-20mA -- ver
   * presion_voltaje.h y el README de hardware seccion 5. Corre en paralelo,
   * sin quitar Presion_Init/Update de arriba, para no perder el avance del
   * circuito de corriente cuando se retome. */
  PresionV_Init(&hadc2);

  /* RAK3172 en USART1 (PA9/PA10). LoRa es prioridad ante todo lo demas
   * en el arranque -- se inicializa y se pide el join ANTES del GPS,
   * para que el join salga lo antes posible y no compita con los
   * HAL_Delay() bloqueantes del bring-up del GPS (~2.5s). */
  RAK3172_Init(&huart1);

  HAL_Delay(500);  // dar tiempo al RAK3172 a terminar su propio arranque

  /* AT+MASK=0002 (sub-banda 2): fix real y necesario para el bug
   * documentado de RUI3 en US915 (con AT+MASK=00FF el join falla con
   * AT_ERROR) -- NO es diagnostico, se queda. Las 8 consultas de
   * solo lectura que corrian aca (AT+NWM=?, AT+NJM=?, etc.) SI eran
   * diagnostico puro para confirmar el provisioning durante el
   * bring-up -- se quitaron (2026-08-18) porque cada una podia
   * esperar hasta 3s, sumando varios segundos al arranque sin
   * aportar nada ahora que el problema de join ya esta resuelto. */
  RAK3172_EnviarComandoAT("AT+MASK=0002");
  {
      uint32_t inicioEspera = HAL_GetTick();
      while (!RAK3172_ComandoListo() && (HAL_GetTick() - inicioEspera) < 3000U) {
          RAK3172_Update();
      }
  }
  /* Impreso aparte (con etiqueta) porque el reporte generico de
   * "resultado=" en el loop principal no distingue de que comando fue
   * -- para ese momento ya se sobreescribe con el resultado del JOIN
   * de abajo, y un AT+MASK que fallo (ERROR/TIMEOUT) quedaba invisible
   * en el log. */
  printf("RAK3172: AT+MASK=0002 -> resultado=%d (0=OK,1=ERROR,2=TIMEOUT,3=BUSY)\r\n",
         RAK3172_GetUltimoResultado());

  RAK3172_Join();

  /* GPS (SIM7600X) en USART2 (PB3/PB4) -- ver gps.h. Va DESPUES del
   * join de LoRa a proposito (ver comentario arriba). */
  GPS_Init(&huart2);

  HAL_Delay(500);  // dar tiempo al SIM7600X a terminar su propio arranque

  /* AT+CGPSCOLD se probo (2026-08-25) para forzar un cold start
   * explicito -- devolvio ERROR, descartado (no soportado en esta
   * version de firmware del modulo). Vuelto a AT+CGPS=1. Contesta
   * "OK" (arranque en frio) o "ERROR" (ya estaba encendido de una
   * sesion previa) -- inofensivo. */
  GPS_EnviarComandoAT("AT+CGPS=1");

  /* Pausa necesaria entre comandos -- sin ella, el modulo deja de
   * contestar cualquier cosa, ni siquiera el eco de AT+CGPSINFO=10
   * mandado justo despues. NO es la causa del bug de "nunca da fix
   * tras un corte real" que se investigo extensamente (2026-08-25) --
   * esa causa resulto ser el cable USB del Nucleo a la PC metiendo
   * ruido al plano de tierra mientras el GNSS intenta enganchar
   * satelites (ver README seccion 2.6) -- no timing de comandos. */
  HAL_Delay(1500);

  /* Habilita el auto-reporte periodico de posicion cada 10 segundos --
   * a partir de aca el modulo manda "+CGPSINFO: ..." por su cuenta
   * (URC), sin que el host tenga que volver a pedirla. */
  GPS_EnviarComandoAT("AT+CGPSINFO=10");
  printf("GPS: inicializado en USART2, auto-reporte cada 10s habilitado\r\n");

  Servo_Init(&htim3, TIM_CHANNEL_3);
  /* SERVO_PULSO_MIN = posición segura (sin aceleración/ralentí) --
   * confirmado en el montaje físico real, no es un valor arbitrario.
   * Se manda de una vez al arrancar (no se conoce la posición previa
   * del servo, quedó como estuviera al perder alimentación).
   *
   * Se carga ANTES de HAL_TIM_PWM_Start() a propósito (reordenado
   * 2026-09-22): el registro de comparación del timer se puede escribir
   * aunque la salida PWM todavía no esté habilitada -- así, cuando
   * arranca la salida, ya sale directo con el pulso calibrado real, sin
   * pasar ni un instante por el "Pulse=1500" genérico que trae
   * MX_TIM3_Init() (CubeMX, no se toca) como default de fábrica. Antes
   * de este cambio, ese hueco de unas pocas instrucciones alcanzaba a
   * mandarle al servo un pulso bien distinto al real (encontrado en
   * campo: el servo daba un pequeño salto visible en cada arranque,
   * aunque ya estuviera bien posicionado). */
  Servo_SetPulsoUs(CalibFlash_GetServoPulsoMinUs());
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  /* Mando manual TEMPORAL por el mismo puerto de debug (LPUART1),
   * mientras no hay red LoRa en campo para probar downlinks reales --
   * ver comando_serial.h. Quitar cuando ya no se necesite. */
  ComandoSerial_Init(&hlpuart1);

  printf("Tacometro STM32G431 - inicio, join LoRaWAN solicitado...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  Tacometro_Update();
	  Presion_Update();
	  PresionV_Update();

  	  /* MEDIDA TEMPORAL DE DIAGNOSTICO (2026-09-03): en CONTROL_HABILITADO=10
  	   * (sintonizacion de PID, renumerado 2026-09-23 -- ver nota de
  	   * cabecera del enum en calibracion_flash.h) se saltea RAK3172_Update()/GPS_Update() por
  	   * completo -- se sospecha que los reintentos de join del RAK3172
  	   * (o el modulo GPS) estan generando ruido electrico/caidas de
  	   * voltaje que contaminan la medicion de RPM durante las pruebas de
  	   * escalon, justo cuando se vio al RAK3172 reiniciandose solo
  	   * (banner de arranque completo repetido) en mitad de una prueba
  	   * con oscilacion severa. Aisla la variable para las pruebas de PID:
  	   * si la oscilacion desaparece con esto, confirma que el problema
  	   * era electrico, no del PID. QUITAR esta condicion (dejar
  	   * RAK3172_Update()/GPS_Update() corriendo siempre) en cuanto se
  	   * resuelva la causa real de los reinicios del RAK3172 -- no es una
  	   * solucion definitiva, el nodo no puede operar en campo sin LoRa. */
  	  if (CalibFlash_GetControlHabilitado() != 10U) {
  		  RAK3172_Update();
  		  GPS_Update();
  	  }
  	  ComandoSerial_Update();

  	  /* Persiste la primera posicion valida del arranque en flash --
  	   * una sola vez, no en cada actualizacion cada 10s (desgastaria
  	   * la flash sin necesidad para un nodo que practicamente no se
  	   * mueve). Sirve de respaldo si el proximo arranque no consigue
  	   * fix a tiempo (ver prioridad de posicion en el uplink LIVE). */
  	  static bool yaPersistioGpsEsteArranque = false;
  	  if (!yaPersistioGpsEsteArranque && GPS_TieneFix()) {
  		  yaPersistioGpsEsteArranque = true;
  		  CalibFlash_SetUltimaPosicionConocida(GPS_GetLatitud(), GPS_GetLongitud());
  		  printf("GPS: primera posicion del arranque persistida en flash (%.6f, %.6f)\r\n",
  		         GPS_GetLatitud(), GPS_GetLongitud());
  	  }

  	  /* Aviso cuando el join se confirme -- por FLANCO (no unido ->
  	   * unido), no una sola vez para siempre: si el modulo se
  	   * reinicia solo y pierde el join a mitad de sesion (ver
  	   * "AT_NO_NETWORK_JOINED" en rak3172.c, que apaga
  	   * RAK3172_EstaUnido()), este aviso debe poder repetirse cuando
  	   * se vuelva a unir. */
  	  static bool estabaUnidoAntes = false;
  	  bool estaUnidoAhora = RAK3172_EstaUnido();
  	  if (!estabaUnidoAntes && estaUnidoAhora) {
  		  printf("RAK3172: unido a la red LoRaWAN (+EVT:JOINED)\r\n");
  	  }
  	  estabaUnidoAntes = estaUnidoAhora;

  	  /* Reintento de join: sin esto, si los 8 intentos internos del
  	   * modulo (AT+JOIN=1:0:10:8) fallan o nunca llego un +EVT:JOINED
  	   * (ej. sin cobertura un rato en el arranque), el nodo se quedaba
  	   * sin unir para siempre -- nada volvia a pedir el join de nuevo.
  	   *
  	   * Se reafirma AT+MASK=0002 justo antes de cada reintento (no solo
  	   * la vez del arranque): si ese comando fallo silenciosamente en
  	   * el arranque (visto en campo: "resultado=1/ERROR" ahi), el join
  	   * se quedaria fallando para siempre por el mismo bug de sub-banda
  	   * ya documentado (ver hallazgos de hardware), sin que este
  	   * reintento lo pudiera arreglar. Maquina de 2 pasos no bloqueante
  	   * (MASK, despues JOIN) para no frenar el resto del loop. */
  	  static uint32_t ultimoIntentoJoinTick = 0;
  	  static bool reenviandoMaskAntesDeJoin = false;
  	  if (reenviandoMaskAntesDeJoin && RAK3172_ComandoListo()) {
  		  reenviandoMaskAntesDeJoin = false;
  		  printf("RAK3172: aun no unido a la red -- reintentando join (AT+JOIN)\r\n");
  		  RAK3172_Join();
  	  } else if (!RAK3172_EstaUnido() && !reenviandoMaskAntesDeJoin && RAK3172_ComandoListo() &&
  			  (HAL_GetTick() - ultimoIntentoJoinTick >= INTERVALO_REINTENTO_JOIN_MS)) {
  		  ultimoIntentoJoinTick = HAL_GetTick();
  		  printf("RAK3172: aun no unido a la red -- reafirmando AT+MASK=0002 antes de reintentar join\r\n");
  		  RAK3172_EnviarComandoAT("AT+MASK=0002");
  		  reenviandoMaskAntesDeJoin = true;
  	  }

  	  /* ================== Sincronizacion de hora: GPS (primaria) + LoRaWAN (respaldo) ==================
  	   * Ver definicion de INTERVALO_RESYNC_GPS_MS / INTERVALO_FALLBACK_LORA_MS
  	   * arriba para el porque de la redundancia. */
  	  static uint32_t ultimoIntentoGpsTick = 0;
  	  static uint32_t ultimoResyncExitosoTick = 0;
  	  static bool huboResyncAlgunaVez = false;
  	  bool relojFueCorregidoEsteCiclo = false;

  	  /* --- Fuente primaria: GPS. Antes del primer sync (de cualquier
  	   * fuente) se intenta en CADA vuelta del loop, no cada 10 min, para
  	   * no perder tiempo esperando si el fix ya esta disponible. */
  	  bool tocaIntentarGps = (HAL_GetTick() - ultimoIntentoGpsTick >= INTERVALO_RESYNC_GPS_MS) ||
  			  !Reloj_EstaSincronizado();

  	  if (tocaIntentarGps) {
  		  ultimoIntentoGpsTick = HAL_GetTick();

  		  uint16_t anioGps; uint8_t mesGps, diaGps, horaGps, minutoGps, segundoGps;
  		  if (GPS_GetFechaHoraUtc(&anioGps, &mesGps, &diaGps, &horaGps, &minutoGps, &segundoGps)) {
  			  Reloj_SetHoraUtc(anioGps, mesGps, diaGps, horaGps, minutoGps, segundoGps);
  			  CalibFlash_SetUltimaHoraUtcConocida(Reloj_GetUnixTimeUtc());
  			  relojFueCorregidoEsteCiclo = true;
  			  ultimoResyncExitosoTick = HAL_GetTick();
  			  huboResyncAlgunaVez = true;
  			  printf("GPS: reloj sincronizado con hora satelital: %02u:%02u:%02u UTC, %02u/%02u/%04u\r\n",
  					 horaGps, minutoGps, segundoGps, mesGps, diaGps, anioGps);
  		  }
  	  }

  	  /* --- Fuente de respaldo: DeviceTimeReq por LoRaWAN. Antes del
  	   * primer sync (de cualquier fuente) compite libremente con el GPS
  	   * -- el que llegue primero gana. Despues del primer sync, se
  	   * limita a activarse solo si ya paso INTERVALO_FALLBACK_LORA_MS
  	   * desde el ultimo resync exitoso sin que el GPS lo haya logrado --
  	   * ej. antena sin vista al cielo por un rato largo. */
  	  bool necesitaFallbackLora = !huboResyncAlgunaVez ||
  			  (HAL_GetTick() - ultimoResyncExitosoTick >= INTERVALO_FALLBACK_LORA_MS);

  	  static bool horaSolicitada = false;
  	  static bool horaConsultada = false;

  	  if (RAK3172_EstaUnido() && necesitaFallbackLora && !horaSolicitada && RAK3172_ComandoListo()) {
  		  horaSolicitada = RAK3172_SolicitarHoraRed();
  		  if (horaSolicitada) {
  			  if (huboResyncAlgunaVez) {
  				  printf("RAK3172: sin resync por GPS en %lu min -- solicitando hora de red como respaldo (AT+TIMEREQ=1)\r\n",
  						 (unsigned long)(INTERVALO_FALLBACK_LORA_MS / 60000UL));
  			  } else {
  				  printf("RAK3172: hora de red solicitada (AT+TIMEREQ=1), compitiendo con el GPS por el primer sync...\r\n");
  			  }
  		  }
  	  }

  	  if (horaSolicitada && !horaConsultada && RAK3172_HoraDeRedDisponible() && RAK3172_ComandoListo()) {
  		  horaConsultada = RAK3172_ConsultarHoraRed();
  	  }

  	  if (horaConsultada && RAK3172_ComandoListo()) {
		  char respuesta[RAK3172_RX_BUFFER_SIZE];
		  if (RAK3172_GetUltimaRespuesta(respuesta, sizeof(respuesta))) {
			  /* Formato REAL confirmado en campo (2026-08-17):
			   * "AT+LTIME=15h08m55s on 08/17/2026" (MM/DD/YYYY) --
			   * NO es un epoch plano como se había asumido sin
			   * ejemplo. Se parsea directo a campos de calendario. */
			  unsigned int hora, minuto, segundo, mes, dia, anio;
			  int camposLeidos = sscanf(respuesta, "AT+LTIME=%2uh%2um%2us on %2u/%2u/%4u",
										 &hora, &minuto, &segundo, &mes, &dia, &anio);
			  if (camposLeidos == 6) {
				  Reloj_SetHoraUtc((uint16_t)anio, (uint8_t)mes, (uint8_t)dia,
								   (uint8_t)hora, (uint8_t)minuto, (uint8_t)segundo);
				  CalibFlash_SetUltimaHoraUtcConocida(Reloj_GetUnixTimeUtc());
				  relojFueCorregidoEsteCiclo = true;
				  ultimoResyncExitosoTick = HAL_GetTick();
				  huboResyncAlgunaVez = true;
				  printf("RAK3172: reloj sincronizado con la red (respaldo): %02u:%02u:%02u UTC, %02u/%02u/%04u\r\n",
						 hora, minuto, segundo, mes, dia, anio);
			  } else {
				  printf("RAK3172: respuesta de AT+LTIME=? no reconocida: '%s'\r\n", respuesta);
			  }
		  }
		  /* Listo para el proximo ciclo (exito o fallo -- si fallo,
		   * necesitaFallbackLora sigue en true y se reintenta en la
		   * proxima vuelta del loop en vez de quedar trabado). */
		  horaSolicitada = false;
		  horaConsultada = false;
	  }

  	  /* ================== Estado del motor + uplink LIVE extendido ==================
  	   * ACTIVO requiere motor operando Y presion de salida real de la
  	   * motobomba (no solo RPM > 0) -- ver README seccion 4.2/5.
  	   * PRESION_UMBRAL_ACTIVO_PSI es un primer valor conservador sin
  	   * dato de campo todavia: distingue "presion real de bombeo" de
  	   * ruido/offset residual con la bomba sin carga. Ajustar con datos
  	   * reales del sistema hidraulico cuando esten disponibles. */
	  static uint8_t estadoAnterior = ESTADO_APAGADO;
	  static uint32_t inicioEstadoLocal = 0;
	  static bool estadoInicializado = false;
	  static uint32_t fechaHoraLocalIterAnterior = 0;
	  /* Debounce de estado (agregado 2026-09-23, ver README seccion 2.1):
	   * con el tacometro sin conectar (linea al aire) se confirmo en campo
	   * que una rafaga de ruido electrico puede colar 1-2 pulsos que SI
	   * pasan el filtro de tacometro.c (ICFilter + periodo minimo + duty
	   * cycle, por diseno valida PULSOS individuales, no el contexto de
	   * varios segundos) y producen una lectura de RPM fisicamente absurda
	   * (2000+ RPM) sostenida solo 1 ciclo -- suficiente para que
	   * estadoActual cambiara un instante y reiniciara inicioEstadoLocal
	   * sin que el motor arrancara de verdad. Un motor diesel real no
	   * puede pasar de detenido a régimen en menos de ESTADO_DEBOUNCE_MS:
	   * exigir que el nuevo estado se sostenga ese tiempo ANTES de
	   * confirmarlo (y de ahi alimentar el uplink/telemetria y el ancla de
	   * inicio_operacion) filtra estos glitches sin afectar ninguna
	   * logica de seguridad -- el interruptor rapido que fuerza
	   * CONTROL_HABILITADO a 0 al arrancar el motor (mas abajo) sigue
	   * usando Tacometro_EstaDetenido() crudo, sin este debounce, a
	   * proposito (necesita reaccionar de inmediato). */
	  static uint8_t estadoCandidato = ESTADO_APAGADO;
	  static uint32_t estadoCandidatoDesdeTick = 0;

	  float rpmActual = Tacometro_GetRPMFiltrada();
	  /* TEMPORAL (2026-09-14): usa el canal de voltaje (PresionV_...) en vez
	   * del de corriente (Presion_..., que sigue corriendo arriba en
	   * paralelo) mientras se resuelve el circuito del lazo 4-20mA -- ver
	   * presion_voltaje.h. Revertir a Presion_GetPresionPsi()/
	   * Presion_SensorValido() cuando ese circuito quede listo. */
	  float presionActual = PresionV_GetPresionPsi();
	  uint8_t estadoInstantaneo;
	  if (rpmActual <= 0.0f) {
		  estadoInstantaneo = ESTADO_APAGADO;
	  } else if (PresionV_SensorValido() && presionActual > PRESION_UMBRAL_ACTIVO_PSI) {
		  estadoInstantaneo = ESTADO_ACTIVO;
	  } else {
		  estadoInstantaneo = ESTADO_ENCENDIDO;
	  }

	  if (estadoInstantaneo != estadoCandidato) {
		  estadoCandidato = estadoInstantaneo;
		  estadoCandidatoDesdeTick = HAL_GetTick();
	  }

	  uint8_t estadoActual = estadoAnterior;
	  if (!estadoInicializado) {
		  estadoActual = estadoInstantaneo;
	  } else if (estadoCandidato != estadoAnterior &&
				 (HAL_GetTick() - estadoCandidatoDesdeTick >= ESTADO_DEBOUNCE_MS)) {
		  estadoActual = estadoCandidato;
	  }

	  uint32_t fechaHoraLocalIterActual = Reloj_GetUnixTimeLocal();

	  if (!estadoInicializado || estadoActual != estadoAnterior) {
		  estadoAnterior = estadoActual;
		  inicioEstadoLocal = fechaHoraLocalIterActual;
		  estadoInicializado = true;
	  } else if (relojFueCorregidoEsteCiclo) {
		  /* El reloj se acaba de corregir (primer sync desde un valor
		   * aproximado/placeholder, o resync periodico contra la deriva
		   * del LSI) -- si el estado NO cambio, desplazar el ancla por
		   * el mismo salto en vez de reiniciarla a "ahora", para no
		   * perder el tiempo ya transcurrido en el estado actual (ej.
		   * el motor llevaba apagado un rato desde el arranque, y la
		   * sincronizacion no deberia reiniciar ese conteo a cero). */
		  inicioEstadoLocal += (fechaHoraLocalIterActual - fechaHoraLocalIterAnterior);
	  }
	  fechaHoraLocalIterAnterior = fechaHoraLocalIterActual;
  	  /* Envío periódico del uplink LIVE, solo una vez unido a la red Y con
  	   * el reloj ya sincronizado (antes de eso, fecha_hora/inicio_operacion
  	   * no tendrían ningún valor confiable que mandar). */
	  static uint32_t ultimoEnvioRPM = 0;
	  bool tocaEnviarPorIntervalo = (HAL_GetTick() - ultimoEnvioRPM >= INTERVALO_ENVIO_RPM_MS);
	  bool tocaEnviarPorForzado = CalibFlash_HayReporteForzado();

	  if (RAK3172_EstaUnido() && (tocaEnviarPorIntervalo || tocaEnviarPorForzado) && RAK3172_ComandoListo()) {
  		  ultimoEnvioRPM = HAL_GetTick();

  		  uint32_t fechaHoraLocal = fechaHoraLocalIterActual;
  		  uint32_t segundosTranscurridos = fechaHoraLocal - inicioEstadoLocal;

  		  /* Prioridad de posicion: GPS con fix vivo > ultima posicion
  		   * conocida persistida en flash (ver CalibFlash_SetUltimaPosicionConocida()
  		   * mas abajo) > coordenada fija de respaldo, para cuando el
  		   * modulo GPS nunca ha conseguido fix (recien instalado, sin
  		   * vista al cielo, etc.). */
  		  float latitudActual = LATITUD_FIJA;
  		  float longitudActual = LONGITUD_FIJA;
  		  if (GPS_TieneFix()) {
  			  latitudActual = GPS_GetLatitud();
  			  longitudActual = GPS_GetLongitud();
  		  } else if (CalibFlash_GetUltimaLatitudConocida() != 0.0f) {
  			  latitudActual = CalibFlash_GetUltimaLatitudConocida();
  			  longitudActual = CalibFlash_GetUltimaLongitudConocida();
  		  }

  		  bool encolado = RAK3172_EnviarUplinkLive(
  			  MOTOR_ID_NUMERIC,
  			  rpmActual,
  			  presionActual,
  			  estadoActual,
  			  fechaHoraLocal,
  			  inicioEstadoLocal,
  			  segundosTranscurridos,
  			  latitudActual,
  			  longitudActual,
  			  (uint8_t)s_codigoAlertaActual
  		  );

  		  (void)encolado;
  		  if (tocaEnviarPorForzado) {
  			  CalibFlash_LimpiarReporteForzado();
  		  }

	  }
  	  /* NOTA DE SEGURIDAD: RESET_REMOTO (CalibFlash_HayResetPendiente())
  	   * deliberadamente NO se conecta todavía. Antes de ejecutar un
  	   * reinicio remoto hay que definir qué hace el servo del
  	   * acelerador durante el reboot (mantener su última posición
  	   * física vs. quedar sin control por unos segundos) -- ver
  	   * discusión de diseño del gobernador de motor. Hasta entonces,
  	   * el comando se recibe y se valida (requiere byte de
  	   * confirmación 0xA5), pero no dispara ninguna acción. */

  	  /* Reporte de resultado del último comando AT (join o send).
  	   * ⚠️ Detecta el FLANCO de "comando recien terminado" (en_curso ->
  	   * listo), NO un cambio de VALOR -- con el chequeo viejo
  	   * (imprimir solo si resultadoActual != el ultimo mostrado), dos
  	   * comandos consecutivos con el MISMO resultado (ej. TIMEOUT tras
  	   * TIMEOUT si el modulo deja de responder) se volvian invisibles
  	   * despues del primero: el log parecia "dejo de intentar" cuando
  	   * en realidad seguia intentando y fallando cada vez igual. */
  	  static bool comandoEnCursoAnterior = true;
  	  bool comandoEnCursoAhora = !RAK3172_ComandoListo();
  	  if (comandoEnCursoAnterior && !comandoEnCursoAhora) {
  		  printf("RAK3172: resultado=%d (0=OK,1=ERROR,2=TIMEOUT,3=BUSY)\r\n", RAK3172_GetUltimoResultado());
  	  }
  	  comandoEnCursoAnterior = comandoEnCursoAhora;

  	  static uint32_t ultimoReporte = 0;
  	  if (HAL_GetTick() - ultimoReporte >= 1000) {
  		  ultimoReporte = HAL_GetTick();
  		  /* TEMPORAL (2026-09-14): se oculta el canal de corriente (Presion_...,
  		   * que sigue corriendo en paralelo arriba) del log mientras se usa
  		   * solo PresionV -- ver presion.h. Restaurar el campo "Presion: ...
  		   * | VDDA: ..." cuando se retome el circuito de 4-20mA. */
  		  /* Uptime = HAL_GetTick() desde el arranque (no el RTC/GPS/hora de
  		   * red) -- a proposito, para poder notar un reinicio espontaneo del
  		   * MCU con solo ver el log: si esto vuelve a 00:00:00 sin que nadie
  		   * lo haya reiniciado a mano, hubo un reset real, sin importar que
  		   * el reloj (fecha_hora) ya haya resincronizado y se vea "normal". */
  		  uint32_t uptimeS = HAL_GetTick() / 1000UL;
  		  uint32_t uptimeH = uptimeS / 3600UL;
  		  uint32_t uptimeM = (uptimeS % 3600UL) / 60UL;
  		  uint32_t uptimeSeg = uptimeS % 60UL;

  		  /* TiempoOperacion = mismo calculo que segundosTranscurridos del
  		   * uplink (inicioEstadoLocal, ver arriba), pero leido cada 1s en
  		   * vez de solo cuando sale un uplink -- para poder ver EN VIVO si
  		   * se reinicia sin esperar al proximo envio de 30s. Comparar
  		   * contra Uptime: si los dos vuelven a 0 juntos, fue un reset real
  		   * del MCU (estadoInicializado se perdio); si SOLO este vuelve a 0
  		   * y Uptime sigue corriendo, fue un cambio de estado (real o un
  		   * glitch del tacometro que paso desapercibido entre uplinks) --
  		   * no un reset. */
  		  uint32_t tiempoOpS = fechaHoraLocalIterActual - inicioEstadoLocal;
  		  uint32_t tiempoOpH = tiempoOpS / 3600UL;
  		  uint32_t tiempoOpM = (tiempoOpS % 3600UL) / 60UL;
  		  uint32_t tiempoOpSeg = tiempoOpS % 60UL;

  		  printf("Uptime: %02lu:%02lu:%02lu | TiempoOperacion: %02lu:%02lu:%02lu (estado=%d) | Frecuencia: %.1f Hz | RPM instant: %.1f | RPM filtrada: %.1f | Detenido: %d | RuidoFiltrado: %lu | RelojSync: %d | PresionV: %.2f PSI (%.3fV%s)\r\n",
  			  (unsigned long)uptimeH,
  			  (unsigned long)uptimeM,
  			  (unsigned long)uptimeSeg,
  			  (unsigned long)tiempoOpH,
  			  (unsigned long)tiempoOpM,
  			  (unsigned long)tiempoOpSeg,
  			  estadoActual,
  			  Tacometro_GetFrecuenciaHz(),
  			  Tacometro_GetRPMInstantanea(),
  			  Tacometro_GetRPMFiltrada(),
  			  Tacometro_EstaDetenido(),
  			  Tacometro_GetContadorRuidoFiltrado(),
  			  Reloj_EstaSincronizado(),
  			  PresionV_GetPresionPsi(),
  			  PresionV_GetVoltaje(),
  			  PresionV_SensorValido() ? "" : ",FALLA_SENSOR");
  	  }

  	  /* Log de alta frecuencia para identificar el modelo de la planta
  	   * (motor+servo) y probar escalones de SET_RPM -- ver README
  	   * sección 9. Prefijo "PID_TEST," fijo para poder filtrar estas
  	   * líneas con un script (ignorando el resto del log, que sigue
  	   * intercalado) sin parsear el resto del texto.
  	   * Usa HAL_GetTick() (ms desde el arranque) y no la hora del RTC a
  	   * propósito: el RTC se resincroniza cada ~30s por GPS/LoRaWAN (ver
  	   * sección 2.2) y esos saltos de corrección contaminarían el
  	   * tiempo relativo de un escalón en curso -- HAL_GetTick() es
  	   * monótono y no depende de si el reloj ya sincronizó.
  	   *
  	   * Gateado por (CONTROL_HABILITADO==10, modo sintonización PID -- O
  	   * ==6, el auto-escalón interno de este mismo lazo, ver
  	   * INTERNO_CAL_* mas abajo -- renumerado 2026-09-23, segunda
  	   * pasada: 10 antes era 9, ver README sección 4.4/9): fuera de esos
  	   * dos modos esto no imprime nada (el monitor se ve igual que antes
  	   * de que existiera este log). Se apaga solo en cuanto se sale de
  	   * ambos (a diferencia de una bandera de una sola vía, sigue el
  	   * mismo estado que ya gatea si PID_KP/KI se pueden tocar -- una
  	   * sola fuente de verdad). */
  	  static uint32_t ultimoLogPrueba = 0;
  	  if ((CalibFlash_GetControlHabilitado() == 10U || CalibFlash_GetControlHabilitado() == 6U)
  	      && HAL_GetTick() - ultimoLogPrueba >= 200U) {
  		  ultimoLogPrueba = HAL_GetTick();
  		  printf("PID_TEST,%lu,%.1f,%.1f,%u\r\n",
  			  (unsigned long)HAL_GetTick(),
  			  CalibFlash_GetSetRpm(),
  			  Tacometro_GetRPMFiltrada(),
  			  Servo_GetPulsoActualUs());
  	  }

  	  /* Imprime las ganancias vigentes del PID (Kp/Ki) apenas se
  	   * entra a CONTROL_HABILITADO=10, y cada vez que alguna cambia
  	   * mientras se sigue ahi -- cubre tanto el mando manual de serial
  	   * como un downlink LoRa (los dos pasan por
  	   * CalibFlash_ProcesarParametroConEstado, esto solo lee el
  	   * resultado ya aplicado). Agregado 2026-09-08 tras un incidente
  	   * de campo donde un reset de la calibracion en flash (magic
  	   * distinto por un cambio de estructura) dejo Kp/Ki en los
  	   * valores de fabrica sin que fuera obvio en el monitor serial --
  	   * ver README seccion 9. Kd removido 2026-09-23 (eliminado del
  	   * protocolo, ver PID_KD en calibracion_flash.h). */
  	  static uint8_t  modoAnteriorParaGanancias = 0U;
  	  static float    pidKpAnteriorImpreso = 0.0f;
  	  static float    pidKiAnteriorImpreso = 0.0f;
  	  uint8_t modoActualParaGanancias = CalibFlash_GetControlHabilitado();
  	  if (modoActualParaGanancias == 10U) {
  		  float pidKpActual = CalibFlash_GetPidKp();
  		  float pidKiActual = CalibFlash_GetPidKi();
  		  if (modoAnteriorParaGanancias != 10U ||
  		      pidKpActual != pidKpAnteriorImpreso ||
  		      pidKiActual != pidKiAnteriorImpreso) {
  			  printf("PID_GANANCIAS,Kp=%.4f,Ki=%.4f\r\n",
  				  pidKpActual, pidKiActual);
  			  pidKpAnteriorImpreso = pidKpActual;
  			  pidKiAnteriorImpreso = pidKiActual;
  		  }
  	  }
  	  modoAnteriorParaGanancias = modoActualParaGanancias;

  	  /* Log de calibración del servo -- gateado a CONTROL_HABILITADO=1
  	   * (manual) o =2 (barrido), ver README sección 2.4/4.4. Antes de
  	   * esto, la única forma de saber los límites vigentes en banco era
  	   * el ACK de cada downlink de SERVO_PULSO_MIN/MAX ("valor
  	   * vigente=X") -- esto muestra en vivo, mientras se observa el
  	   * servo moverse, dónde están los límites configurados y qué pulso
  	   * se le está aplicando en ese instante. Cadencia de 1s (a
  	   * diferencia de PID_TEST, esto es solo para lectura humana, no se
  	   * analiza después con un script). */
  	  static uint32_t ultimoLogServoCal = 0;
  	  uint8_t modoParaLogServo = CalibFlash_GetControlHabilitado();
  	  if ((modoParaLogServo == 1U || modoParaLogServo == 2U) &&
  	      HAL_GetTick() - ultimoLogServoCal >= 1000U) {
  		  ultimoLogServoCal = HAL_GetTick();
  		  printf("SERVO_CAL,min=%uus,max=%uus,pulso_logico=%uus,pulso_fisico=%uus\r\n",
  			  CalibFlash_GetServoPulsoMinUs(),
  			  CalibFlash_GetServoPulsoMaxUs(),
  			  Servo_GetPulsoActualUs(),
  			  Servo_GetPulsoFisicoActualUs());
  	  }


  	  /* Apagado local de seguridad: si el motor arranca mientras se
  	   * está en modo de calibración del SERVO (CONTROL_HABILITADO=1 o
  	   * =2), se sale de inmediato -- sin esperar ningún downlink -- para
  	   * no seguir moviendo el servo (en barrido o manual) con el motor
  	   * operando. Mismo criterio que el resto del firmware
  	   * (Tacometro_EstaDetenido()), nunca un switch remoto para algo de
  	   * seguridad física.
  	   * NO incluye los modos 6/8/9/10 (auto-escalón interno/presión/
  	   * remoto, sintonización de PID -- renumerados 2026-09-23) a
  	   * propósito: esos modos necesitan que el motor siga operando
  	   * durante toda la sesión (README sección 9) -- el servo en esos
  	   * modos lo maneja el
  	   * PID normal exactamente igual que en modo 0, así que no hay
  	   * barrido/posición manual que "se quede corriendo" sin control. */
  	  uint8_t modoCalibracionServo = CalibFlash_GetControlHabilitado();
  	  if (!Tacometro_EstaDetenido() && (modoCalibracionServo == 1U || modoCalibracionServo == 2U)) {
  		  CalibFlash_SetControlHabilitado(0U);
  		  CalibFlash_LimpiarObjetivoManualServo(); /* invalida cualquier
  		                                             * objetivo manual pendiente,
  		                                             * no arrastrarlo a una
  		                                             * futura sesion de calibracion */
  		  CalibFlash_SetSetRpm(0.0f); /* mismo criterio que al ENTRAR a 1/2/3
  		                                 * (calibracion_flash.c) -- si habia
  		                                 * quedado un SET_RPM viejo mandado
  		                                 * mientras se calibraba el servo, no
  		                                 * debe activar el PID solo al volver
  		                                 * a modo 0 por este apagado de
  		                                 * seguridad. */
  	  }

  	  /* PRUEBA DE BANCO -- servo en modo calibración mientras
  	   * CONTROL_HABILITADO != 0 (ver protocolo de downlinks, renumerado
  	   * 2026-09-23): =1 barrido automático MIN<->MAX, =2 manual (se
  	   * queda quieto salvo un downlink nuevo de SERVO_PULSO_MIN/MAX),
  	   * =10 sintonización de PID (el servo NO cambia de comportamiento
  	   * respecto a =0 -- sigue siendo pid.c quien lo maneja -- lo único
  	   * que cambia con =10 es que se desbloquean PID_KP/KI y se activa
  	   * el log PID_TEST, ver calibracion_flash.c). QUITAR/comentar este
  	   * bloque si algún día se retira también la calibración remota. */
  	  bool motorOperandoAhora = !Tacometro_EstaDetenido();
  	  float rpmMin = CalibFlash_GetRpmMin();
  	  float rpmMax = CalibFlash_GetRpmMax();

  	  /* ================== MODO: de donde sale el setpoint de RPM ==================
  	   * Agregado 2026-09-07 -- antes MODO se recibia/persistia/ACKeaba
  	   * pero nadie lo leia aca. ⚠️ RENUMERADO 2026-09-14 (ver el enum
  	   * CalibFlash_Modo_t en calibracion_flash.h) para que coincida con
  	   * la terminologia que ya usa el cliente en campo:
  	   *
  	   *   RALENTI (0): sin control activo, sin importar SET_RPM/PRESION/
  	   *     presion local -- setpoint forzado a "sin comandar" (0.0f,
  	   *     mismo valor que el default de SET_RPM). Cae en la rama de
  	   *     "sin control" de mas abajo (servo en SERVO_PULSO_MIN).
  	   *   PRESION_LOCAL (1): setpoint = salida del lazo EXTERNO de
  	   *     presion (presion_pid.c/PresionPid_CalcularSetpointRpm()),
  	   *     comparando PresionV_GetPresionPsi() (sensor propio del TID,
  	   *     temporal -- ver presion_voltaje.h) contra PRESION_OBJETIVO.
  	   *     Este es el lazo de "llenado de tuberia" / control de presion
  	   *     de salida de la motobomba con el sensor local. Ganancias
  	   *     propias PRESION_PID_KP/KI, se calibran EN CAMPO igual que
  	   *     PID_KP/KI (solo con CONTROL_HABILITADO=10).
  	   *   REMOTO (2): setpoint = salida de
  	   *     PresionPidRemoto_CalcularSetpointRpm() contra
  	   *     PRESION_OBJETIVO_REMOTO, con PRESION recibida por downlink de
  	   *     un nodo aspersor remoto (NO del sensor local -- ver
  	   *     CalibFlash_SetPresion()). PID en cascada EVENTO-DRIVEN,
  	   *     agregado 2026-09-23 en reemplazo de la formula lineal abierta
  	   *     que tenia antes este modo -- un PID no necesita conocer de
  	   *     antemano la relacion presion->RPM de cada instalacion (se
  	   *     autoajusta al error real), a diferencia de la formula que
  	   *     exigia calibrar PRESION_GANANCIA_RPM/OFFSET_RPM en cada sitio.
  	   *     Solo se recalcula cuando llega un reporte NUEVO del aspersor
  	   *     (su cadencia es muy irregular, 20s a 20-25min segun su propio
  	   *     estado) -- ver bloque mas abajo y presion_pid_remoto.h.
  	   *   MANUAL_BANCO (3, antes era el valor 1): setpoint = SET_RPM
  	   *     (downlink/serial directo) -- uso de banco/sintonizacion, sin
  	   *     presion de bombeo real (con el motor operando en vacio). Es
  	   *     el modo que se uso TODA la sesion de sintonizacion del PID
  	   *     interno (Kp=0.047/Ki=0.11, ver README seccion 9).
  	   *
  	   * ⚠️ MIGRACION 2026-09-14: como este parametro solo corrio hasta
  	   * ahora en la unidad de banco (sin flota desplegada todavia), no
  	   * hace falta coordinar con nodos en campo -- pero si esta unidad
  	   * quedo guardada en MODO=1 con el significado VIEJO (SET_RPM
  	   * directo), tras este cambio va a intentar control por presion
  	   * local en su lugar. Verificar/reasignar MODO explicitamente
  	   * (MODO 3 para seguir en banco, MODO 1 para presion local) antes
  	   * de operar tras reflashear. */
  	  CalibFlash_Modo_t modoMotor = CalibFlash_GetModo();
  	  float setpointRpmCrudo = 0.0f;

  	  /* Watchdog de TIMEOUT_SIN_COMANDO_S, solo aplica en REMOTO: si no
  	   * llega una PRESION valida en mas de ese tiempo (cuenta desde el
  	   * boot si nunca llego ninguna), se fuerza el setpoint a "sin
  	   * comandar" -- el motor cae a ralenti natural en vez de sostener
  	   * indefinidamente el ultimo valor de presion, ya viejo. */
  	  static bool presionExpiradaAntes = false;
  	  bool presionExpiradaAhora = false;

  	  /* Edge-detector de entrada a PRESION_LOCAL, mismo patron que
  	   * pidActivoAntes/pidActivoAhora mas abajo (pid.c) -- resetea el
  	   * estado interno del lazo externo (integral, ancla de tiempo) al
  	   * entrar a este modo, para no arrastrar un dt inflado por tiempo
  	   * inactivo ni una integral vieja de una sesion anterior. */
  	  static bool presionLocalActivoAntes = false;
  	  bool presionLocalActivoAhora = (modoMotor == CALIB_MODO_PRESION_LOCAL);
  	  /* Ancla de la rampa de llenado (TASA_LLENADO_PSI_S) -- se toma la
  	   * presion medida en el instante EXACTO de entrar a este modo, no
  	   * un valor fijo asumido. Si la tuberia ya estaba parcialmente
  	   * llena de una sesion anterior, la rampa arranca mas arriba y
  	   * tarda menos sola en llegar a PRESION_OBJETIVO, sin logica
  	   * especial para ese caso -- ver uso mas abajo. */
  	  static float presionLlenadoInicioPsi = 0.0f;
  	  static uint32_t presionLlenadoInicioMs = 0U;
  	  if (presionLocalActivoAhora && !presionLocalActivoAntes) {
  		  PresionPid_Init();
  		  presionLlenadoInicioPsi = PresionV_GetPresionPsi();
  		  presionLlenadoInicioMs = HAL_GetTick();
  		  s_codigoAlertaActual = ALERTA_SIN_ALERTA; /* arranque fresco, sin arrastrar una alerta vieja */
  	  }
  	  presionLocalActivoAntes = presionLocalActivoAhora;

  	  /* Edge-detector de entrada a REMOTO, mismo patron/motivo que
  	   * presionLocalActivoAntes arriba -- resetea el estado interno del
  	   * PID de presion remota (integral, ancla de tiempo) al entrar a
  	   * este modo. Ademas calcula un primer setpoint inmediato con el
  	   * ultimo dato de PRESION conocido (sin esperar a un reporte
  	   * "nuevo" -- si no, el motor se quedaria en 0 hasta que llegue el
  	   * proximo reporte, que puede tardar hasta 20-25min si el aspersor
  	   * esta en 0 PSI). remotoPidUltimoTickVisto se marca como "ya visto"
  	   * en ese mismo instante para que el bloque de mas abajo no vuelva a
  	   * recalcular hasta que realmente llegue un reporte posterior. */
  	  static bool remotoActivoAntes = false;
  	  bool remotoActivoAhora = (modoMotor == CALIB_MODO_REMOTO);
  	  static uint32_t remotoPidUltimoTickVisto = 0U;
  	  static float remotoPidUltimoSetpointRpm = 0.0f;
  	  if (remotoActivoAhora && !remotoActivoAntes) {
  		  PresionPidRemoto_Init();
  		  remotoPidUltimoTickVisto = CalibFlash_GetPresionUltimoTickMs();
  		  remotoPidUltimoSetpointRpm = PresionPidRemoto_CalcularSetpointRpm(
  				  CalibFlash_GetPresionObjetivoRemoto(), CalibFlash_GetPresion());
  		  s_codigoAlertaActual = ALERTA_SIN_ALERTA; /* arranque fresco -- ademas es la
  		                                              * unica forma de limpiar la alerta
  		                                              * final de fuga (MODO ya cayo a
  		                                              * RALENTI cuando esa alerta se
  		                                              * disparo, asi que el operador
  		                                              * reenviando MODO=2 es la señal de
  		                                              * "dale, probemos de nuevo") */
  	  }
  	  remotoActivoAntes = remotoActivoAhora;

  	  /* Igual que el watchdog de PRESION en REMOTO: si el sensor local
  	   * esta en falla, no tiene sentido cerrar el lazo contra una
  	   * lectura invalida -- se cae a "sin comandar" (ralenti natural)
  	   * en vez de sostener ciegamente el ultimo setpoint calculado. */
  	  static bool presionLocalSensorFallaAntes = false;
  	  bool presionLocalSensorFallaAhora = false;

  	  if (modoMotor == CALIB_MODO_PRESION_LOCAL) {
  		  presionLocalSensorFallaAhora = !PresionV_SensorValido();
  		  if (presionLocalSensorFallaAhora) {
  			  setpointRpmCrudo = 0.0f; /* sin comandar -- cae a ralenti natural mas abajo */
  		  } else {
  			  /* Rampa de llenado (agregada 2026-09-23): en vez de mandarle
  			   * PRESION_OBJETIVO directo al PID, se le manda un objetivo
  			   * que sube gradualmente desde presionLlenadoInicioPsi --
  			   * "if (rampaObjetivo < objetivoFinal)" recorta sola apenas
  			   * la rampa alcanza el objetivo final, asi que "en rampa" vs
  			   * "ya estable" es implicito, no hace falta una bandera de
  			   * fase aparte. Con TASA_LLENADO_PSI_S=0 (default), se salta
  			   * la rampa del todo (el "if" de arriba nunca entra) y
  			   * objetivoDelInstante se queda en objetivoFinal directo --
  			   * mismo comportamiento que existia antes de esto. */
  			  float objetivoFinal = CalibFlash_GetPresionObjetivo();
  			  float tasaLlenado = CalibFlash_GetTasaLlenadoPsiS();
  			  float objetivoDelInstante = objetivoFinal;
  			  if (tasaLlenado > 0.0f) {
  				  float segundosTranscurridos = (HAL_GetTick() - presionLlenadoInicioMs) / 1000.0f;
  				  float rampaObjetivo = presionLlenadoInicioPsi + tasaLlenado * segundosTranscurridos;
  				  if (rampaObjetivo < objetivoFinal) {
  					  objetivoDelInstante = rampaObjetivo;
  				  }
  			  }
  			  setpointRpmCrudo = PresionPid_CalcularSetpointRpm(
  					  objetivoDelInstante, PresionV_GetPresionPsi());
  		  }

  		  if (presionLocalSensorFallaAhora != presionLocalSensorFallaAntes) {
  			  if (presionLocalSensorFallaAhora) {
  				  printf("MODO_PRESION_LOCAL: sensor de presion en falla -- forzando ralenti\r\n");
  				  s_codigoAlertaActual = ALERTA_SENSOR_PRESION_LOCAL_FALLA;
  			  } else {
  				  printf("MODO_PRESION_LOCAL: sensor de presion valido de nuevo -- reanudando control por presion\r\n");
  				  s_codigoAlertaActual = ALERTA_SIN_ALERTA;
  			  }
  			  CalibFlash_ForzarReporte(); /* uplink LIVE inmediato -- su fechaHoraLocal
  			                                * es el timestamp de esta transicion */
  		  }
  	  } else {
  		  presionLocalSensorFallaAhora = false; /* fuera de PRESION_LOCAL, no aplica */
  	  }
  	  presionLocalSensorFallaAntes = presionLocalSensorFallaAhora;

  	  /* Supervisor de "no se puede llegar a PRESION_OBJETIVO" (ver
  	   * PRESION_SUPERVISOR_* mas arriba y README seccion 8/12). NO es
  	   * una proteccion contra sobre-acelerar el motor -- eso ya lo
  	   * garantiza presion_pid.c recortando su propia salida a
  	   * [0,RPM_MAX] antes de que llegue aca (ver PresionPid_CalcularSetpointRpm()).
  	   * Esto es solo deteccion: si el lazo externo queda pidiendo
  	   * RPM_MAX (el maximo que se le permite) y la presion real sigue
  	   * sin acercarse al objetivo, es señal de que la meta pedida no es
  	   * alcanzable con el RPM_MAX configurado (o que algo anda mal con
  	   * el sensor/sistema hidraulico) -- vale la pena avisar en vez de
  	   * quedarse ahi indefinidamente en silencio. Se ignora mientras el
  	   * sensor esta en falla (esa condicion ya tiene su propia alerta,
  	   * arriba, y setpointRpmCrudo=0 ahi no cuenta como "saturado"). */
  	  static uint32_t presionSupervisorInicioVentanaMs = 0U;
  	  static bool     presionSupervisorEnVentana = false;
  	  static uint32_t presionSupervisorIntentosFallidos = 0U;

  	  if (modoMotor == CALIB_MODO_PRESION_LOCAL && !presionLocalSensorFallaAhora) {
  		  float presionObjetivoActual = CalibFlash_GetPresionObjetivo();
  		  float presionMedidaActual = PresionV_GetPresionPsi();
  		  bool saturadoSinAlcanzar = (setpointRpmCrudo >= rpmMax)
  				  && (presionMedidaActual < presionObjetivoActual * (1.0f - PRESION_SUPERVISOR_TOLERANCIA_FRACCION));

  		  if (!saturadoSinAlcanzar) {
  			  if (presionSupervisorIntentosFallidos >= PRESION_SUPERVISOR_INTENTOS_MAX) {
  				  printf("ALERTA,PRESION_OBJETIVO_RECUPERADA\r\n");
  				  s_codigoAlertaActual = ALERTA_SIN_ALERTA;
  				  CalibFlash_ForzarReporte();
  			  }
  			  presionSupervisorEnVentana = false;
  			  presionSupervisorIntentosFallidos = 0U;
  		  } else if (!presionSupervisorEnVentana) {
  			  presionSupervisorEnVentana = true;
  			  presionSupervisorInicioVentanaMs = HAL_GetTick();
  		  } else if (HAL_GetTick() - presionSupervisorInicioVentanaMs >= PRESION_SUPERVISOR_VENTANA_MS) {
  			  presionSupervisorIntentosFallidos++;
  			  presionSupervisorInicioVentanaMs = HAL_GetTick();
  			  if (presionSupervisorIntentosFallidos >= PRESION_SUPERVISOR_INTENTOS_MAX) {
  				  printf("ALERTA,PRESION_OBJETIVO_NO_ALCANZADA,intentos=%lu,presion_objetivo=%.2f,presion_actual=%.2f,rpm_max=%.1f\r\n",
  					  (unsigned long)presionSupervisorIntentosFallidos, presionObjetivoActual,
  					  presionMedidaActual, rpmMax);
  				  s_codigoAlertaActual = ALERTA_PRESION_OBJETIVO_NO_ALCANZADA;
  				  CalibFlash_ForzarReporte();
  			  }
  		  }
  	  } else {
  		  presionSupervisorEnVentana = false;
  		  presionSupervisorIntentosFallidos = 0U;
  	  }

  	  if (modoMotor == CALIB_MODO_MANUAL_BANCO) {
  		  setpointRpmCrudo = CalibFlash_GetSetRpm();
  	  } else if (modoMotor == CALIB_MODO_REMOTO) {
  		  uint32_t timeoutMs = CalibFlash_GetTimeoutSinComandoS() * 1000UL;
  		  presionExpiradaAhora = (HAL_GetTick() - CalibFlash_GetPresionUltimoTickMs()) >= timeoutMs;
  		  if (presionExpiradaAhora) {
  			  setpointRpmCrudo = 0.0f; /* sin comandar -- cae a ralenti natural mas abajo */
  		  } else {
  			  /* PID evento-driven (agregado 2026-09-23, reemplaza la formula
  			   * lineal PRESION_OFFSET_RPM + PRESION_GANANCIA_RPM*PRESION):
  			   * solo se recalcula cuando llega un reporte NUEVO del aspersor
  			   * -- mismo detector de flanco que usa el supervisor mas abajo,
  			   * pero con su propia variable estatica (necesita correr ANTES
  			   * para que el supervisor evalue el setpoint ya actualizado).
  			   * Entre reportes se sostiene remotoPidUltimoSetpointRpm -- no
  			   * tiene sentido volver a llamar al PID con el mismo dato
  			   * viejo, y es justo el dt REAL entre llamadas (no un ciclo
  			   * fijo) lo que hace correcto el termino integral pese a la
  			   * cadencia irregular del aspersor. */
  			  uint32_t tickPresionActual = CalibFlash_GetPresionUltimoTickMs();
  			  if (tickPresionActual != remotoPidUltimoTickVisto) {
  				  remotoPidUltimoTickVisto = tickPresionActual;
  				  remotoPidUltimoSetpointRpm = PresionPidRemoto_CalcularSetpointRpm(
  						  CalibFlash_GetPresionObjetivoRemoto(), CalibFlash_GetPresion());
  			  }
  			  setpointRpmCrudo = remotoPidUltimoSetpointRpm;
  		  }

  		  if (presionExpiradaAhora != presionExpiradaAntes) {
  			  if (presionExpiradaAhora) {
  				  printf("MODO_REMOTO: sin PRESION valida hace mas de %lus -- forzando ralenti (TIMEOUT_SIN_COMANDO_S)\r\n",
  						 (unsigned long)CalibFlash_GetTimeoutSinComandoS());
  				  s_codigoAlertaActual = ALERTA_MODO_REMOTO_SIN_PRESION_VALIDA;
  			  } else {
  				  printf("MODO_REMOTO: PRESION valida de nuevo -- reanudando control por presion\r\n");
  				  s_codigoAlertaActual = ALERTA_SIN_ALERTA;
  			  }
  			  CalibFlash_ForzarReporte();
  		  }
  	  } else {
  		  presionExpiradaAhora = false; /* fuera de REMOTO, no aplica -- que la proxima entrada a REMOTO loguee fresco */
  	  }
  	  presionExpiradaAntes = presionExpiradaAhora;

  	  /* Supervisor ACTIVO de MODO=2 (Remoto), agregado 2026-09-23,
  	   * REDISEÑADO el mismo dia dos veces tras feedback del usuario --
  	   * version final:
  	   *
  	   * 1) Cuenta REPORTES NUEVOS consecutivos (via
  	   *    CalibFlash_GetPresionUltimoTickMs()), no segundos de reloj --
  	   *    la PRESION del aspersor solo cambia cuando llega un reporte
  	   *    real, y su cadencia (confirmada en campo 2026-09-23) varia
  	   *    segun su propio estado: ~20-25min con presion en 0, ~20s
  	   *    mientras esta en rampa (justo el estado de una fuga real,
  	   *    ver mas abajo), ~90s ya estabilizado en su propio objetivo.
  	   *    Contar segundos fijos no tiene sentido con esa variabilidad.
  	   *
  	   * 2) Compara contra PRESION_OBJETIVO_REMOTO (ID 30, la presion que
  	   *    se ESPERA que reporte el aspersor, configurada en este TID)
  	   *    cuando esta calibrada -- mismo criterio que MODO=1: saturado
  	   *    en RPM_MAX Y la PRESION remota real sigue lejos del objetivo
  	   *    (±5%). Si no esta calibrada (0, default), cae al criterio
  	   *    simple de "solo saturado".
  	   *
  	   * 3) A DIFERENCIA de MODO=1 (que solo alerta, nunca actua, decision
  	   *    original documentada en README seccion 8) -- este SI retrocede
  	   *    activamente, a pedido explicito del usuario, pensado para el
  	   *    escenario real de una fuga entre el motor y el aspersor: el
  	   *    aspersor nunca sube de presion por mas RPM que se le pida, asi
  	   *    que sin esto el motor se quedaria forzando RPM_MAX indefinida-
  	   *    mente sin lograr nada. Con 1 o 2 intentos fallidos, se fuerza
  	   *    "sin comandar" (mismo mecanismo que la rama de ralenti, SIN
  	   *    cambiar MODO de verdad -- evita escribir flash en cada
  	   *    reintento) durante PRESION_SUPERVISOR_REMOTO_RETROCESO_MS
  	   *    antes de reintentar. Recien en el 3er intento fallido
  	   *    (PRESION_SUPERVISOR_INTENTOS_MAX) se manda la alerta Y se pasa
  	   *    MODO a RALENTI de verdad (unica escritura real a flash de todo
  	   *    este ciclo).
  	   *
  	   * Todo el bloque se ignora mientras la PRESION esta vencida --
  	   * ese es un problema de SEÑAL (se resuelve solo con el watchdog de
  	   * arriba, que ya fuerza ralenti por su cuenta), no de fuga; si se
  	   * cae la señal a mitad de un ciclo de reintentos, se cancela el
  	   * reintento en vez de competir con ese watchdog. */
  	  static uint32_t remotoSupervisorUltimoTickVisto = 0U;
  	  static uint32_t remotoSupervisorIntentosFallidos = 0U;
  	  static bool     remotoEnRetroceso = false;
  	  static uint32_t remotoRetrocesoInicioMs = 0U;

  	  if (modoMotor == CALIB_MODO_REMOTO && !presionExpiradaAhora) {

  		  if (remotoEnRetroceso) {
  			  setpointRpmCrudo = 0.0f; /* sin comandar -- cae a ralenti natural mas abajo, mismo mecanismo que MODO=0 */
  			  if (HAL_GetTick() - remotoRetrocesoInicioMs >= PRESION_SUPERVISOR_REMOTO_RETROCESO_MS) {
  				  remotoEnRetroceso = false;
  			  }
  		  } else {
  			  uint32_t tickPresionActual = CalibFlash_GetPresionUltimoTickMs();
  			  bool llegoReporteNuevo = (tickPresionActual != remotoSupervisorUltimoTickVisto);

  			  if (llegoReporteNuevo) {
  				  remotoSupervisorUltimoTickVisto = tickPresionActual;

  				  float objetivoRemoto = CalibFlash_GetPresionObjetivoRemoto();
  				  float presionRemotaActual = CalibFlash_GetPresion();
  				  bool  objetivoConfigurado = objetivoRemoto > 0.0f;
  				  bool  saturadoEnMax = setpointRpmCrudo >= rpmMax;
  				  bool  lejosDelObjetivo = objetivoConfigurado
  						  && (presionRemotaActual < objetivoRemoto * (1.0f - PRESION_SUPERVISOR_TOLERANCIA_FRACCION));
  				  bool  alertarEsteReporte = objetivoConfigurado
  						  ? (saturadoEnMax && lejosDelObjetivo) : saturadoEnMax;

  				  if (!alertarEsteReporte) {
  					  if (remotoSupervisorIntentosFallidos >= PRESION_SUPERVISOR_REMOTO_INTENTOS_MAX) {
  						  printf("ALERTA,MODO_REMOTO_OBJETIVO_RECUPERADO\r\n");
  						  s_codigoAlertaActual = ALERTA_SIN_ALERTA;
  						  CalibFlash_ForzarReporte();
  					  }
  					  remotoSupervisorIntentosFallidos = 0U;
  				  } else {
  					  remotoSupervisorIntentosFallidos++;
  					  if (remotoSupervisorIntentosFallidos >= PRESION_SUPERVISOR_REMOTO_INTENTOS_MAX) {
  						  printf("ALERTA,MODO_REMOTO_OBJETIVO_NO_ALCANZADO,intentos=%lu,presion_remota=%.1f,objetivo_remoto=%.1f,presion_local=%.2f,rpm_max=%.1f\r\n",
  							  (unsigned long)remotoSupervisorIntentosFallidos, presionRemotaActual,
  							  objetivoRemoto, PresionV_GetPresionPsi(), rpmMax);
  						  s_codigoAlertaActual = ALERTA_MODO_REMOTO_OBJETIVO_NO_ALCANZADO;
  						  CalibFlash_ForzarReporte();
  						  CalibFlash_SetModo(CALIB_MODO_RALENTI);
  						  remotoSupervisorIntentosFallidos = 0U;
  					  } else {
  						  printf("MODO_REMOTO: intento %lu/%lu sin alcanzar el objetivo -- retrocediendo a ralenti %lus antes de reintentar\r\n",
  							  (unsigned long)remotoSupervisorIntentosFallidos, (unsigned long)PRESION_SUPERVISOR_REMOTO_INTENTOS_MAX,
  							  (unsigned long)(PRESION_SUPERVISOR_REMOTO_RETROCESO_MS / 1000U));
  						  remotoEnRetroceso = true;
  						  remotoRetrocesoInicioMs = HAL_GetTick();
  						  setpointRpmCrudo = 0.0f;
  					  }
  				  }
  			  }
  		  }
  	  } else {
  		  remotoSupervisorIntentosFallidos = 0U;
  		  remotoEnRetroceso = false;
  	  }

  	  /* RALENTI (0): setpointRpmCrudo se queda en 0.0f -- sin control, ver
  	   * comentario del bloque de arriba. */

  	  /* Modo 0 (ralentí) = sin control activo: el motor sube solo de
  	   * 0Hz a su ralentí natural (distinto en cada unidad, por eso
  	   * RPM_MIN es "límite duro / ralentí" y no una constante) sin que
  	   * el PID intervenga -- el servo se queda quieto en
  	   * SERVO_PULSO_MIN. El lazo solo se activa si el setpoint elegido
  	   * arriba (según MODO) supera ese ralentí configurado; 0.0f (sin
  	   * comandar, o MODO_RALENTI) o apenas unos pocos RPM nunca debe
  	   * forzar el servo. */
  	  bool controlSolicitado = setpointRpmCrudo > rpmMin;

  	  /* Log de alta frecuencia para el lazo EXTERNO de presión (cascada,
  	   * MODO=1), mismo prefijo fijo/formato-columna que PID_TEST (ver
  	   * arriba) para poder reusar tools/pid_tuning/pid_tuning.py sin
  	   * cambios -- ese script es agnóstico de qué variable física
  	   * representa "setpoint"/"medida", solo ajusta un modelo FOPDT a la
  	   * serie de tiempo. Gateado a (CONTROL_HABILITADO==10, sesion manual
  	   * de sintonizacion que desbloquea PRESION_PID_KP/KI -- O ==8, el
  	   * auto-escalon de este mismo lazo, ver mas abajo -- renumerado
  	   * 2026-09-23, segunda pasada: 10 antes era 9, 8 antes era 7) Y
  	   * MODO==1 (sin esto el lazo externo no corre, no tendría sentido
  	   * loguearlo) -- fuera de ahí no imprime nada. Incluye tanto la
  	   * presión (lo que este lazo controla) como el RPM resultante (su
  	   * salida, que a su vez es el setpoint del lazo interno) para poder
  	   * ver los dos lazos de la cascada en la misma línea. */
  	  static uint32_t ultimoLogPruebaPresion = 0;
  	  if ((CalibFlash_GetControlHabilitado() == 10U || CalibFlash_GetControlHabilitado() == 8U)
  	      && modoMotor == CALIB_MODO_PRESION_LOCAL
  	      && HAL_GetTick() - ultimoLogPruebaPresion >= 200U) {
  		  ultimoLogPruebaPresion = HAL_GetTick();
  		  printf("PRESION_PID_TEST,%lu,%.2f,%.2f,%.1f,%.1f\r\n",
  			  (unsigned long)HAL_GetTick(),
  			  CalibFlash_GetPresionObjetivo(),
  			  PresionV_GetPresionPsi(),
  			  setpointRpmCrudo,
  			  Tacometro_GetRPMFiltrada());
  	  }

  	  /* Auto-escalon INTERNO (PID#1) -- CONTROL_HABILITADO=6 (ver
  	   * INTERNO_CAL_* mas arriba). Solo automatiza mandar el escalon de
  	   * SET_RPM y esperar -- el servo lo sigue manejando pid.c
  	   * normalmente via MODO=3 (pidActivoAhora ya incluye
  	   * modoControl==6, ver mas abajo). Requiere MODO=3 (MANUAL_BANCO)
  	   * ya puesto por el operador. Base = RPM_MIN, mismo motivo que
  	   * REMOTO_CAL (SET_RPM ya quedo en 0.0f al entrar a este modo, ver
  	   * calibracion_flash.c). Estados: 1=esperando motor+MODO=3,
  	   * 2=corriendo. Variable de flanco PROPIA (no comparte
  	   * 'modoControlAnteriorParaCal' con PRESION_CAL/REMOTO_CAL) porque
  	   * este bloque corre justo antes de esos, mismo motivo de fondo. */
  	  static uint8_t  internoCalEstado = 1U;
  	  static uint32_t internoCalInicioMs = 0;
  	  static float    internoCalSetRpmBase = 0.0f;
  	  static uint8_t  modoControlAnteriorParaInterno = 0U;

  	  if (modoControlAnteriorParaInterno != 6U && CalibFlash_GetControlHabilitado() == 6U) {
  		  internoCalEstado = 1U;
  		  printf("INTERNO_CAL,INICIO,esperando_motor=%d,esperando_modo3=%d\r\n",
  			  (int)!motorOperandoAhora, (int)(modoMotor != CALIB_MODO_MANUAL_BANCO));
  	  }

  	  if (CalibFlash_GetControlHabilitado() == 6U) {
  		  if (internoCalEstado == 1U && motorOperandoAhora && modoMotor == CALIB_MODO_MANUAL_BANCO) {
  			  internoCalSetRpmBase = CalibFlash_GetRpmMin();
  			  CalibFlash_SetSetRpm(internoCalSetRpmBase + INTERNO_CAL_ESCALON_RPM);
  			  internoCalEstado = 2U;
  			  internoCalInicioMs = HAL_GetTick();
  			  printf("INTERNO_CAL,ESCALON,set_rpm_base=%.1f,set_rpm_nuevo=%.1f,duracion_s=%lu\r\n",
  				  internoCalSetRpmBase, internoCalSetRpmBase + INTERNO_CAL_ESCALON_RPM,
  				  (unsigned long)(INTERNO_CAL_DURACION_MS / 1000U));
  		  } else if (internoCalEstado == 2U && !motorOperandoAhora) {
  			  printf("INTERNO_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  CalibFlash_SetSetRpm(0.0f);
  			  internoCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  } else if (internoCalEstado == 2U && modoMotor != CALIB_MODO_MANUAL_BANCO) {
  			  printf("INTERNO_CAL,ABORTADO,motivo=modo_cambio\r\n");
  			  CalibFlash_SetSetRpm(0.0f);
  			  internoCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  } else if (internoCalEstado == 2U && HAL_GetTick() - internoCalInicioMs >= INTERNO_CAL_DURACION_MS) {
  			  printf("INTERNO_CAL,COMPLETO,duracion_s=%lu\r\n",
  				  (unsigned long)(INTERNO_CAL_DURACION_MS / 1000U));
  			  CalibFlash_SetSetRpm(0.0f);
  			  internoCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  }
  	  }
  	  modoControlAnteriorParaInterno = CalibFlash_GetControlHabilitado();

  	  /* Auto-escalon de presion local -- CONTROL_HABILITADO=8 (ver
  	   * PRESION_CAL_* mas arriba). Solo automatiza mandar el escalon y
  	   * esperar -- el servo lo sigue manejando presion_pid.c/pid.c
  	   * normalmente (pidActivoAhora ya incluye modoControl==8, ver mas
  	   * abajo), esto NO toca el servo directo como GANANCIA_CAL. Requiere
  	   * MODO=1 ya puesto por el operador (si no, se queda esperando, no
  	   * es un error). Estados: 1=esperando motor+MODO=1, 2=corriendo.
  	   * ⚠️ Renumerado 2026-09-23, antes era CONTROL_HABILITADO=7. */
  	  static uint8_t  presionCalEstado = 1U;
  	  static uint32_t presionCalInicioMs = 0;
  	  static float    presionCalObjetivoBase = 0.0f;
  	  /* Deteccion de flanco de entrada PROPIA para los bloques de auto-
  	   * escalon (6/8/9) -- independiente de 'modoControlAnterior' (usada
  	   * mas abajo por el if/else de auto-cals que mueven el servo
  	   * directo), porque ese se declara/actualiza mas adelante en el
  	   * archivo y estos bloques corren antes en el loop. */
  	  static uint8_t modoControlAnteriorParaCal = 0U;

  	  if (modoControlAnteriorParaCal != 8U && CalibFlash_GetControlHabilitado() == 8U) {
  		  presionCalEstado = 1U;
  		  printf("PRESION_CAL,INICIO,esperando_motor=%d,esperando_modo1=%d\r\n",
  			  (int)!motorOperandoAhora, (int)(modoMotor != CALIB_MODO_PRESION_LOCAL));
  	  }

  	  if (CalibFlash_GetControlHabilitado() == 8U) {
  		  if (presionCalEstado == 1U && motorOperandoAhora && modoMotor == CALIB_MODO_PRESION_LOCAL) {
  			  presionCalObjetivoBase = CalibFlash_GetPresionObjetivo();
  			  CalibFlash_SetPresionObjetivo(presionCalObjetivoBase + PRESION_CAL_ESCALON_PSI);
  			  presionCalEstado = 2U;
  			  presionCalInicioMs = HAL_GetTick();
  			  printf("PRESION_CAL,ESCALON,objetivo_base=%.2f,objetivo_nuevo=%.2f,duracion_s=%lu\r\n",
  				  presionCalObjetivoBase, presionCalObjetivoBase + PRESION_CAL_ESCALON_PSI,
  				  (unsigned long)(PRESION_CAL_DURACION_MS / 1000U));
  		  } else if (presionCalEstado == 2U && !motorOperandoAhora) {
  			  printf("PRESION_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  CalibFlash_SetPresionObjetivo(presionCalObjetivoBase);
  			  presionCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  } else if (presionCalEstado == 2U && modoMotor != CALIB_MODO_PRESION_LOCAL) {
  			  printf("PRESION_CAL,ABORTADO,motivo=modo_cambio\r\n");
  			  CalibFlash_SetPresionObjetivo(presionCalObjetivoBase);
  			  presionCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  } else if (presionCalEstado == 2U && HAL_GetTick() - presionCalInicioMs >= PRESION_CAL_DURACION_MS) {
  			  printf("PRESION_CAL,COMPLETO,duracion_s=%lu\r\n",
  				  (unsigned long)(PRESION_CAL_DURACION_MS / 1000U));
  			  CalibFlash_SetPresionObjetivo(presionCalObjetivoBase);
  			  presionCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  }
  	  }

  	  /* Log de IDENTIFICACION EN LAZO ABIERTO para el lazo de presion
  	   * REMOTA (MODO=2), agregado 2026-09-23. A diferencia de PID_TEST/
  	   * PRESION_PID_TEST (200ms fijos), este es EVENTO-DRIVEN -- solo
  	   * imprime cuando llega un reporte NUEVO de PRESION del aspersor
  	   * (mismo detector de flanco que ya usan el PID remoto y su
  	   * supervisor, pero con su propia variable estatica, independiente).
  	   * No tiene sentido mandarlo a intervalos fijos: la PRESION remota
  	   * ni cambia entre un reporte y el siguiente, y la cadencia real
  	   * (20s a 20-25min segun el estado del aspersor) no calza con un
  	   * muestreo de 200ms.
  	   *
  	   * Gateado a (CONTROL_HABILITADO==10, sesion manual -- O ==9, el
  	   * auto-escalon de este mismo lazo, ver mas abajo -- renumerado
  	   * 2026-09-23, segunda pasada: 10 antes era 9, 9 antes era 8) Y
  	   * MODO==3 (MANUAL_BANCO) -- a proposito NO es MODO==2: para
  	   * identificar la planta hace falta mandar el escalon de RPM
  	   * DIRECTO, bypaseando el PID que todavia no esta calibrado (Kp/Ki
  	   * en 0), asi que la prueba real se hace en modo banco (SET_RPM
  	   * manual), no en modo remoto. Ver tools/pid_tuning/pid_tuning.py
  	   * (identify-remoto, lazo ABIERTO -- distinto de
  	   * identify/identify-presion, que son en lazo cerrado). */
  	  static uint32_t remotoPidTestUltimoTickVisto = 0U;
  	  if ((CalibFlash_GetControlHabilitado() == 10U || CalibFlash_GetControlHabilitado() == 9U)
  	      && modoMotor == CALIB_MODO_MANUAL_BANCO) {
  		  uint32_t tickPresionActual = CalibFlash_GetPresionUltimoTickMs();
  		  if (tickPresionActual != remotoPidTestUltimoTickVisto) {
  			  remotoPidTestUltimoTickVisto = tickPresionActual;
  			  printf("REMOTO_PID_TEST,%lu,%.1f,%.1f\r\n",
  				  (unsigned long)HAL_GetTick(),
  				  CalibFlash_GetSetRpm(),
  				  CalibFlash_GetPresion());
  		  }
  	  }

  	  /* Auto-escalon remoto -- CONTROL_HABILITADO=9 (ver REMOTO_CAL_* mas
  	   * arriba, renumerado 2026-09-23, antes era 8). Mismo espiritu que
  	   * PRESION_CAL de arriba: solo automatiza mandar el escalon de
  	   * SET_RPM y esperar reportes -- el servo lo sigue manejando pid.c
  	   * normalmente via MODO=3 (pidActivoAhora ya incluye
  	   * modoControl==9, ver mas abajo). Requiere MODO=3 (MANUAL_BANCO)
  	   * ya puesto por el operador. Termina por lo que ocurra primero:
  	   * REMOTO_CAL_REPORTES_OBJETIVO reportes nuevos, o
  	   * REMOTO_CAL_TIMEOUT_MS de seguridad. Estados: 1=esperando
  	   * motor+MODO=3, 2=corriendo. */
  	  static uint8_t  remotoCalEstado = 1U;
  	  static uint32_t remotoCalInicioMs = 0;
  	  static float    remotoCalSetRpmBase = 0.0f;
  	  static uint32_t remotoCalUltimoTickVisto = 0U;
  	  static uint32_t remotoCalReportesContados = 0U;

  	  if (modoControlAnteriorParaCal != 9U && CalibFlash_GetControlHabilitado() == 9U) {
  		  remotoCalEstado = 1U;
  		  printf("REMOTO_CAL,INICIO,esperando_motor=%d,esperando_modo3=%d\r\n",
  			  (int)!motorOperandoAhora, (int)(modoMotor != CALIB_MODO_MANUAL_BANCO));
  	  }

  	  if (CalibFlash_GetControlHabilitado() == 9U) {
  		  if (remotoCalEstado == 1U && motorOperandoAhora && modoMotor == CALIB_MODO_MANUAL_BANCO) {
  			  /* Base = RPM_MIN, NO CalibFlash_GetSetRpm() -- SET_RPM ya
  			   * quedo en 0.0f al entrar a este modo (limpieza de
  			   * seguridad al cambiar CONTROL_HABILITADO, ver
  			   * calibracion_flash.c), asi que leerlo aca siempre daria
  			   * 0, no un valor util. Con SET_RPM=0 el motor ya esta
  			   * posado en su ralenti natural (controlSolicitado=false,
  			   * setpointRpmCrudo=0 < rpmMin) -- RPM_MIN es la
  			   * descripcion correcta de donde esta parado en este
  			   * instante, y ademas es un piso conocido/seguro para
  			   * arrancar el escalon sin depender de que el operador
  			   * haya mandado algo antes. */
  			  remotoCalSetRpmBase = CalibFlash_GetRpmMin();
  			  CalibFlash_SetSetRpm(remotoCalSetRpmBase + REMOTO_CAL_ESCALON_RPM);
  			  remotoCalUltimoTickVisto = CalibFlash_GetPresionUltimoTickMs();
  			  remotoCalReportesContados = 0U;
  			  remotoCalEstado = 2U;
  			  remotoCalInicioMs = HAL_GetTick();
  			  printf("REMOTO_CAL,ESCALON,set_rpm_base=%.1f,set_rpm_nuevo=%.1f,reportes_objetivo=%lu,timeout_s=%lu\r\n",
  				  remotoCalSetRpmBase, remotoCalSetRpmBase + REMOTO_CAL_ESCALON_RPM,
  				  (unsigned long)REMOTO_CAL_REPORTES_OBJETIVO, (unsigned long)(REMOTO_CAL_TIMEOUT_MS / 1000U));
  		  } else if (remotoCalEstado == 2U) {
  			  uint32_t tickRemotoCalActual = CalibFlash_GetPresionUltimoTickMs();
  			  if (tickRemotoCalActual != remotoCalUltimoTickVisto) {
  				  remotoCalUltimoTickVisto = tickRemotoCalActual;
  				  remotoCalReportesContados++;
  				  printf("REMOTO_CAL,REPORTE,n=%lu,presion=%.1f\r\n",
  					  (unsigned long)remotoCalReportesContados, CalibFlash_GetPresion());
  			  }

  			  if (!motorOperandoAhora) {
  				  printf("REMOTO_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  				  CalibFlash_SetSetRpm(0.0f); /* vuelve a sin comandar, no a RPM_MIN */
  				  remotoCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else if (modoMotor != CALIB_MODO_MANUAL_BANCO) {
  				  printf("REMOTO_CAL,ABORTADO,motivo=modo_cambio\r\n");
  				  CalibFlash_SetSetRpm(0.0f); /* vuelve a sin comandar, no a RPM_MIN */
  				  remotoCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else if (remotoCalReportesContados >= REMOTO_CAL_REPORTES_OBJETIVO) {
  				  printf("REMOTO_CAL,COMPLETO,motivo=reportes_objetivo,reportes=%lu\r\n",
  					  (unsigned long)remotoCalReportesContados);
  				  CalibFlash_SetSetRpm(0.0f); /* vuelve a sin comandar, no a RPM_MIN */
  				  remotoCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else if (HAL_GetTick() - remotoCalInicioMs >= REMOTO_CAL_TIMEOUT_MS) {
  				  printf("REMOTO_CAL,COMPLETO,motivo=timeout,reportes=%lu\r\n",
  					  (unsigned long)remotoCalReportesContados);
  				  CalibFlash_SetSetRpm(0.0f); /* vuelve a sin comandar, no a RPM_MIN */
  				  remotoCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  }
  		  }
  	  }

  	  /* Barrido de ganancia de PRESION (PID#2) -- CONTROL_HABILITADO=11
  	   * (agregado 2026-09-24, ver PRESION_GANANCIA_CAL_* mas arriba).
  	   * Equivalente de GANANCIA_CAL (CONTROL_HABILITADO=5) pero para el
  	   * lazo de presion: en vez de mover el servo directo, manda SET_RPM
  	   * directo en MODO=3 (sin presion_pid corriendo -- lazo ABIERTO para
  	   * la relacion RPM->presion). Requiere MODO=3 ya puesto por el
  	   * operador (si no, se queda esperando, no es error) y ASUME que la
  	   * tuberia ya esta llena/presurizada -- ver nota de riesgo en el
  	   * bloque de constantes y README seccion 12, Fase D.
  	   *
  	   * Estados: 1=esperando motor+MODO=3, 2=midiendo presion base en
  	   * RPM_MIN (promedio simple sobre PRESION_GANANCIA_CAL_BASE_PROMEDIO_MS),
  	   * 3=rampeando SET_RPM hacia el objetivo del paso en curso (NO un
  	   * salto -- ver nota de riesgo #1), 4=asentando (esperando a que la
  	   * PRESION responda ya con el SET_RPM del paso ya alcanzado). */
  	  static uint8_t  presionGanCalEstado = 1U;
  	  static uint32_t presionGanCalInicioEtapaMs = 0;
  	  static uint32_t presionGanCalInicioTotalMs = 0; /* ancla de PRESION_GANANCIA_CAL_TIMEOUT_MS -- SEPARADA de InicioEtapaMs, que se resetea en cada paso/rampa. Arranca al confirmar motor+MODO=3 (estado 1->2), no antes (no cuenta el tiempo esperando al operador) */
  	  static float    presionGanCalPsiBase = 0.0f;
  	  static float    presionGanCalPsiTecho = 0.0f;
  	  static float    presionGanCalPsiAcumulada = 0.0f;
  	  static uint32_t presionGanCalMuestrasBase = 0U;
  	  static float    presionGanCalSetRpmAncla = 0.0f;
  	  static float    presionGanCalSetRpmObjetivo = 0.0f;
  	  static float    presionGanCalPsiAnterior = 0.0f;

  	  if (modoControlAnteriorParaCal != 11U && CalibFlash_GetControlHabilitado() == 11U) {
  		  presionGanCalEstado = 1U;
  		  printf("PRESION_GANANCIA_CAL,INICIO,esperando_motor=%d,esperando_modo3=%d\r\n",
  			  (int)!motorOperandoAhora, (int)(modoMotor != CALIB_MODO_MANUAL_BANCO));
  	  }

  	  if (CalibFlash_GetControlHabilitado() == 11U) {
  		  if (presionGanCalEstado == 1U && motorOperandoAhora && modoMotor == CALIB_MODO_MANUAL_BANCO) {
  			  /* SET_RPM=0.0f, NO RPM_MIN -- mismo motivo que REMOTO_CAL/
  			   * INTERNO_CAL: controlSolicitado exige > rpmMin (estricto),
  			   * asi que RPM_MIN exacto igual cae en la rama "sin control"
  			   * (servo a SERVO_PULSO_MIN, ralenti natural) -- 0.0f es la
  			   * forma correcta de describir ese estado, ver calibracion_flash.c. */
  			  CalibFlash_SetSetRpm(0.0f);
  			  presionGanCalPsiAcumulada = 0.0f;
  			  presionGanCalMuestrasBase = 0U;
  			  presionGanCalEstado = 2U;
  			  presionGanCalInicioEtapaMs = HAL_GetTick();
  			  presionGanCalInicioTotalMs = HAL_GetTick();
  		  } else if (presionGanCalEstado == 2U && !motorOperandoAhora) {
  			  printf("PRESION_GANANCIA_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  CalibFlash_SetSetRpm(0.0f);
  			  presionGanCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  } else if (presionGanCalEstado == 2U && modoMotor != CALIB_MODO_MANUAL_BANCO) {
  			  printf("PRESION_GANANCIA_CAL,ABORTADO,motivo=modo_cambio\r\n");
  			  CalibFlash_SetSetRpm(0.0f);
  			  presionGanCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  } else if (presionGanCalEstado == 2U) {
  			  presionGanCalPsiAcumulada += Presion_GetPresionPsi();
  			  presionGanCalMuestrasBase++;
  			  if (HAL_GetTick() - presionGanCalInicioEtapaMs >= PRESION_GANANCIA_CAL_BASE_PROMEDIO_MS) {
  				  presionGanCalPsiBase = presionGanCalPsiAcumulada / (float)presionGanCalMuestrasBase;
  				  presionGanCalPsiTecho = presionGanCalPsiBase + PRESION_GANANCIA_CAL_DELTA_TECHO_PSI;
  				  presionGanCalPsiAnterior = presionGanCalPsiBase;
  				  presionGanCalSetRpmAncla = CalibFlash_GetRpmMin();
  				  presionGanCalSetRpmObjetivo = presionGanCalSetRpmAncla + PRESION_GANANCIA_CAL_PASO_RPM;
  				  printf("PRESION_GANANCIA_CAL,BASE,psi_base=%.2f,psi_techo=%.2f\r\n",
  					  presionGanCalPsiBase, presionGanCalPsiTecho);
  				  presionGanCalEstado = 3U;
  				  presionGanCalInicioEtapaMs = HAL_GetTick();
  			  }
  		  } else if (presionGanCalEstado == 3U && !motorOperandoAhora) {
  			  printf("PRESION_GANANCIA_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  CalibFlash_SetSetRpm(0.0f);
  			  presionGanCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  } else if (presionGanCalEstado == 3U && modoMotor != CALIB_MODO_MANUAL_BANCO) {
  			  printf("PRESION_GANANCIA_CAL,ABORTADO,motivo=modo_cambio\r\n");
  			  CalibFlash_SetSetRpm(0.0f);
  			  presionGanCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  } else if (presionGanCalEstado == 3U) {
  			  /* Rampa propia -- NO un salto -- ver nota de riesgo #1 en el
  			   * bloque de constantes: sin esto, el PID#1 interno llegaria
  			   * al SET_RPM nuevo en pocos segundos pese al dwell largo. */
  			  float segundosTranscurridos = (HAL_GetTick() - presionGanCalInicioEtapaMs) / 1000.0f;
  			  float rpmRampeado = presionGanCalSetRpmAncla + PRESION_GANANCIA_CAL_RAMPA_RPM_S * segundosTranscurridos;
  			  if (rpmRampeado >= presionGanCalSetRpmObjetivo) {
  				  CalibFlash_SetSetRpm(presionGanCalSetRpmObjetivo);
  				  presionGanCalEstado = 4U;
  				  presionGanCalInicioEtapaMs = HAL_GetTick();
  			  } else {
  				  CalibFlash_SetSetRpm(rpmRampeado);
  			  }
  		  } else if (presionGanCalEstado == 4U && !motorOperandoAhora) {
  			  printf("PRESION_GANANCIA_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  CalibFlash_SetSetRpm(0.0f);
  			  presionGanCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  } else if (presionGanCalEstado == 4U && modoMotor != CALIB_MODO_MANUAL_BANCO) {
  			  printf("PRESION_GANANCIA_CAL,ABORTADO,motivo=modo_cambio\r\n");
  			  CalibFlash_SetSetRpm(0.0f);
  			  presionGanCalEstado = 1U;
  			  CalibFlash_SetControlHabilitado(0U);
  		  } else if (presionGanCalEstado == 4U &&
  		             HAL_GetTick() - presionGanCalInicioEtapaMs >= PRESION_GANANCIA_CAL_DWELL_MS) {
  			  float psiActual = Presion_GetPresionPsi();
  			  float saltoPsi = psiActual - presionGanCalPsiAnterior;
  			  printf("PRESION_GANANCIA_CAL,PASO,rpm=%.1f,psi=%.2f,salto=%.2f\r\n",
  				  presionGanCalSetRpmObjetivo, psiActual, saltoPsi);
  			  presionGanCalPsiAnterior = psiActual;

  			  if (saltoPsi >= PRESION_GANANCIA_CAL_SALTO_ANORMAL_PSI) {
  				  printf("PRESION_GANANCIA_CAL,ABORTADO,motivo=salto_psi_anormal\r\n");
  				  CalibFlash_SetSetRpm(0.0f);
  				  presionGanCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else if (psiActual >= presionGanCalPsiTecho) {
  				  printf("PRESION_GANANCIA_CAL,COMPLETO,motivo=techo_psi_alcanzado,rpm_final=%.1f,psi_final=%.2f\r\n",
  					  presionGanCalSetRpmObjetivo, psiActual);
  				  CalibFlash_SetSetRpm(0.0f);
  				  presionGanCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else if (presionGanCalSetRpmObjetivo >= GANANCIA_CAL_RPM_TECHO) {
  				  printf("PRESION_GANANCIA_CAL,COMPLETO,motivo=techo_rpm_alcanzado,rpm_final=%.1f,psi_final=%.2f\r\n",
  					  presionGanCalSetRpmObjetivo, psiActual);
  				  CalibFlash_SetSetRpm(0.0f);
  				  presionGanCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else if (HAL_GetTick() - presionGanCalInicioTotalMs >= PRESION_GANANCIA_CAL_TIMEOUT_MS) {
  				  /* No deberia dispararse en un barrido normal (~13-18min
  				   * estimados) -- red de seguridad de ultimo recurso si algo
  				   * quedo trabado (ej. sensor de presion sin variar). */
  				  printf("PRESION_GANANCIA_CAL,COMPLETO,motivo=timeout,rpm_final=%.1f,psi_final=%.2f\r\n",
  					  presionGanCalSetRpmObjetivo, psiActual);
  				  CalibFlash_SetSetRpm(0.0f);
  				  presionGanCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else {
  				  presionGanCalSetRpmAncla = presionGanCalSetRpmObjetivo;
  				  presionGanCalSetRpmObjetivo = presionGanCalSetRpmAncla + PRESION_GANANCIA_CAL_PASO_RPM;
  				  presionGanCalEstado = 3U;
  				  presionGanCalInicioEtapaMs = HAL_GetTick();
  			  }
  		  }
  	  }

  	  modoControlAnteriorParaCal = CalibFlash_GetControlHabilitado();

  	  uint8_t modoControl = CalibFlash_GetControlHabilitado(); /* ⚠️ RENUMERADO 2026-09-24: 0=desactivado, 1=manual, 2=barrido, 3=auto ALPHA, 4=auto zona muerta (antes 5), 5=mapeo ganancia (antes 6), 6=auto-escalon interno NUEVO (PID#1), 7=auto ralenti (antes 4), 8=auto-escalon presion local (antes 7), 9=auto-escalon remoto (antes 8), 10=sintonizacion PID (antes 9), 11=barrido de ganancia de presion NUEVO (PID#2) */

  	  /* Auto-calibracion de ralenti -- CONTROL_HABILITADO=7 (renumerado
  	   * 2026-09-23, antes era 4 -- ver calibracion_flash.c: solo se
  	   * puede pedir viniendo de modo 0, con o sin el motor operando).
  	   * Mientras se esta en modo 7:
  	   *   - si el motor no esta operando, se espera -- el servo ya
  	   *     queda en SERVO_PULSO_MIN automaticamente (pidActivoAhora es
  	   *     siempre false en modo 7, cae en la rama de abajo).
  	   *   - apenas el motor arranca (o si ya estaba andando al entrar a
  	   *     este modo), arranca la ventana de medicion.
  	   *   - si el motor se detiene a mitad de la ventana, se aborta la
  	   *     medicion en curso y se vuelve a esperar (sin salir de modo
  	   *     7) -- no tiene sentido promediar con el motor parado, pero
  	   *     tampoco hace falta reenviar el downlink si se puede
  	   *     reintentar solo arrancando el motor de nuevo.
  	   *   - al completar los 2 minutos, se aplica RPM_MIN y se vuelve
  	   *     sola a modo 0 (mismo criterio que el apagado de seguridad de
  	   *     1/2 en este mismo loop, que tambien fuerza CONTROL_HABILITADO
  	   *     localmente sin esperar un downlink). */
  	  {
  		  static bool     ralentiCalEsperandoMotor = false;
  		  static bool     ralentiCalMidiendo = false;
  		  static uint32_t ralentiCalInicioMs = 0;
  		  static uint32_t ralentiCalUltimoLogMs = 0;
  		  static float    ralentiCalSuma = 0.0f;
  		  static uint32_t ralentiCalMuestras = 0;

  		  if (modoControl == 7U) {
  			  if (!ralentiCalEsperandoMotor && !ralentiCalMidiendo) {
  				  ralentiCalEsperandoMotor = true;
  				  printf("RALENTI_CAL,INICIO,esperando_motor=%d\r\n", (int)!motorOperandoAhora);
  			  }

  			  if (ralentiCalEsperandoMotor && motorOperandoAhora) {
  				  ralentiCalEsperandoMotor = false;
  				  ralentiCalMidiendo = true;
  				  ralentiCalInicioMs = HAL_GetTick();
  				  ralentiCalUltimoLogMs = ralentiCalInicioMs;
  				  ralentiCalSuma = 0.0f;
  				  ralentiCalMuestras = 0;
  				  printf("RALENTI_CAL,MIDIENDO,duracion_total_s=%lu,ventana_promedio_s=%lu\r\n",
  					  (unsigned long)(RALENTI_CAL_DURACION_TOTAL_MS / 1000U),
  					  (unsigned long)(RALENTI_CAL_VENTANA_PROMEDIO_MS / 1000U));
  			  }

  			  if (ralentiCalMidiendo) {
  				  if (!motorOperandoAhora) {
  					  printf("RALENTI_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  					  ralentiCalMidiendo = false;
  					  ralentiCalEsperandoMotor = true;
  				  } else {
  					  uint32_t transcurridoMs = HAL_GetTick() - ralentiCalInicioMs;

  					  if (transcurridoMs >= (RALENTI_CAL_DURACION_TOTAL_MS - RALENTI_CAL_VENTANA_PROMEDIO_MS)) {
  						  ralentiCalSuma += Tacometro_GetRPMFiltrada();
  						  ralentiCalMuestras++;
  					  }

  					  if (HAL_GetTick() - ralentiCalUltimoLogMs >= 5000U) {
  						  ralentiCalUltimoLogMs = HAL_GetTick();
  						  printf("RALENTI_CAL,PROGRESO,transcurrido_s=%lu,rpm_actual=%.1f\r\n",
  							  (unsigned long)(transcurridoMs / 1000U), Tacometro_GetRPMFiltrada());
  					  }

  					  if (transcurridoMs >= RALENTI_CAL_DURACION_TOTAL_MS) {
  						  if (ralentiCalMuestras > 0U) {
  							  float promedio = ralentiCalSuma / (float)ralentiCalMuestras;
  							  float nuevoRpmMin = promedio + RALENTI_CAL_MARGEN_RPM;
  							  bool ok = CalibFlash_SetRpmMin(nuevoRpmMin);
  							  printf("RALENTI_CAL,COMPLETO,promedio=%.1f,RPM_MIN_nuevo=%.1f,muestras=%lu,ok=%d\r\n",
  								  promedio, nuevoRpmMin, (unsigned long)ralentiCalMuestras, (int)ok);
  							  CalibFlash_ForzarReporte(); /* "ACK" por LoRa -- uplink
  							                                 * inmediato con el reporte
  							                                 * estandar, en vez de
  							                                 * esperar al proximo
  							                                 * intervalo periodico */
  						  } else {
  							  printf("RALENTI_CAL,ERROR,sin_muestras\r\n");
  						  }
  						  ralentiCalMidiendo = false;
  						  CalibFlash_SetControlHabilitado(0U); /* vuelve sola a operacion normal */
  					  }
  				  }
  			  }
  		  } else {
  			  ralentiCalEsperandoMotor = false;
  			  ralentiCalMidiendo = false;
  		  }
  	  }

  	  /* Auto-calibracion de ALPHA -- CONTROL_HABILITADO=3 (ver
  	   * calibracion_flash.c: solo se puede pedir viniendo de modo 0, con
  	   * o sin el motor operando). Mismo patron que el bloque de modo 7
  	   * (ralenti) -- espera el motor, mide, aplica, vuelve sola a modo 0. */
  	  {
  		  static bool     alphaCalEsperandoMotor = false;
  		  static bool     alphaCalMidiendo = false;
  		  static uint32_t alphaCalInicioMs = 0;
  		  static uint32_t alphaCalUltimoMuestreoMs = 0;
  		  static uint32_t alphaCalMuestras = 0;
  		  static float    alphaCalMedia = 0.0f;
  		  static float    alphaCalM2 = 0.0f; /* Welford -- evita cancelacion catastrofica al restar RPM~cientos/miles al cuadrado */

  		  if (modoControl == 3U) {
  			  if (!alphaCalEsperandoMotor && !alphaCalMidiendo) {
  				  alphaCalEsperandoMotor = true;
  				  printf("ALPHA_CAL,INICIO,esperando_motor=%d\r\n", (int)!motorOperandoAhora);
  			  }

  			  if (alphaCalEsperandoMotor && motorOperandoAhora) {
  				  alphaCalEsperandoMotor = false;
  				  alphaCalMidiendo = true;
  				  alphaCalInicioMs = HAL_GetTick();
  				  alphaCalUltimoMuestreoMs = alphaCalInicioMs;
  				  alphaCalMuestras = 0U;
  				  alphaCalMedia = 0.0f;
  				  alphaCalM2 = 0.0f;
  				  printf("ALPHA_CAL,MIDIENDO,duracion_s=%lu\r\n", (unsigned long)(ALPHA_CAL_VENTANA_MS / 1000U));
  			  }

  			  if (alphaCalMidiendo) {
  				  if (!motorOperandoAhora) {
  					  printf("ALPHA_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  					  alphaCalMidiendo = false;
  					  alphaCalEsperandoMotor = true;
  				  } else {
  					  uint32_t transcurridoMs = HAL_GetTick() - alphaCalInicioMs;

  					  if (HAL_GetTick() - alphaCalUltimoMuestreoMs >= ALPHA_CAL_INTERVALO_MUESTREO_MS) {
  						  alphaCalUltimoMuestreoMs = HAL_GetTick();
  						  float muestra = Tacometro_GetRPMInstantanea();
  						  alphaCalMuestras++;
  						  float delta = muestra - alphaCalMedia;
  						  alphaCalMedia += delta / (float)alphaCalMuestras;
  						  float delta2 = muestra - alphaCalMedia;
  						  alphaCalM2 += delta * delta2;
  					  }

  					  if (transcurridoMs >= ALPHA_CAL_VENTANA_MS) {
  						  if (alphaCalMuestras >= 2U) {
  							  float varianza = alphaCalM2 / (float)(alphaCalMuestras - 1U);
  							  float desviacionCruda = sqrtf(varianza);
  							  float r = (desviacionCruda > ALPHA_CAL_OBJETIVO_DESVIACION_RPM)
  										  ? (ALPHA_CAL_OBJETIVO_DESVIACION_RPM / desviacionCruda)
  										  : 1.0f; /* ruido crudo ya en/por debajo del objetivo -- el techo de abajo evita que esto se traduzca en "sin filtro" */
  							  float alphaCalculado = (2.0f * r * r) / (1.0f + r * r);
  							  if (alphaCalculado > ALPHA_CAL_TECHO_ALPHA) {
  								  alphaCalculado = ALPHA_CAL_TECHO_ALPHA;
  							  }
  							  bool ok = CalibFlash_SetAlphaFiltro(alphaCalculado);
  							  printf("ALPHA_CAL,COMPLETO,desviacion_cruda=%.2f,alpha_nuevo=%.4f,muestras=%lu,ok=%d\r\n",
  								  desviacionCruda, CalibFlash_GetAlphaFiltro(), (unsigned long)alphaCalMuestras, (int)ok);
  							  CalibFlash_ForzarReporte(); /* "ACK" por LoRa, mismo criterio que ralenti/servo-min */
  						  } else {
  							  printf("ALPHA_CAL,ERROR,sin_muestras_suficientes\r\n");
  						  }
  						  alphaCalMidiendo = false;
  						  CalibFlash_SetControlHabilitado(0U); /* vuelve sola a operacion normal */
  					  }
  				  }
  			  }
  		  } else {
  			  alphaCalEsperandoMotor = false;
  			  alphaCalMidiendo = false;
  		  }
  	  }

  	  static bool pidActivoAntes = false;
  	  /* Modos 6/8/9/10 usan el MISMO camino de control que el modo 0 --
  	   * el servo lo maneja pid.c/presion_pid.c/presion_pid_remoto.c
  	   * igual en todos los casos, segun MODO (ver bloque de arriba), no
  	   * segun CONTROL_HABILITADO. La unica diferencia con modo 0 es que
  	   * estos cinco desbloquean cosas O manejan su propia maquina de
  	   * estados de auto-escalon (ver bloques INTERNO_CAL/PRESION_CAL/
  	   * REMOTO_CAL/PRESION_GANANCIA_CAL, junto a PID_TEST/
  	   * PRESION_PID_TEST/REMOTO_PID_TEST):
  	   * =6 auto-escalon interno (PID #1), =8 auto-escalon presion
  	   * local, =9 auto-escalon remoto, =10 sintonizacion manual
  	   * (desbloquea PID_KP/KI, activa los logs *_PID_TEST), =11 barrido
  	   * de ganancia de presion NUEVO (PID#2, agregado 2026-09-24) --
  	   * necesita que el PID interno (PID#1) siga corriendo para que el
  	   * SET_RPM que manda PRESION_GANANCIA_CAL mueva el servo de verdad.
  	   * Renumerado 2026-09-23 (segunda pasada) -- antes eran 7/8/9
  	   * respectivamente, sin el modo 6 (que no existia). No hace falta
  	   * un branch aparte para ninguno de los cinco en el if/else de
  	   * abajo -- como no son 1/2/3/4/5/7, caen solos en esta rama
  	   * (7=ralenti es la unica excepcion real, ese fuerza el servo a
  	   * SERVO_PULSO_MIN aparte). */
  	  bool pidActivoAhora = (modoControl == 0U || modoControl == 6U || modoControl == 8U
  	          || modoControl == 9U || modoControl == 10U || modoControl == 11U)
  	          && motorOperandoAhora && controlSolicitado;
  	  if (pidActivoAhora && !pidActivoAntes) {
  		  PID_Init(); /* flanco de entrada -- evita un dt inflado por tiempo inactivo */
  	  }
  	  pidActivoAntes = pidActivoAhora;

  	  static uint8_t modoControlAnterior = 0U;

  	  if (modoControl == 2U) {
  		  /* Pausa en cada extremo (SERVO_BARRIDO_PAUSA_EXTREMOS_MS,
  		   * servo.h) antes de invertir direccion -- el pulso PWM llega
  		   * exacto al limite en cuanto Servo_MoverHacia() lo reporta,
  		   * pero el servo fisico tiene su propio tiempo de asentamiento;
  		   * sin esta espera, el firmware invertia la marcha en la misma
  		   * vuelta del loop en que el pulso llegaba al extremo, y el
  		   * brazo real nunca alcanzaba a terminar de llegar. */
  		  static bool haciaMaximoServo = true;
  		  static bool esperandoEnExtremo = false;
  		  static uint32_t inicioEsperaExtremoMs = 0;
  		  uint16_t minimo = CalibFlash_GetServoPulsoMinUs();
  		  uint16_t maximo = CalibFlash_GetServoPulsoMaxUs();
  		  uint16_t destinoServo = haciaMaximoServo ? maximo : minimo;
  		  uint16_t aplicadoServo = Servo_MoverHacia(destinoServo);
  		  if (!esperandoEnExtremo) {
  			  if (aplicadoServo == destinoServo) {
  				  esperandoEnExtremo = true;
  				  inicioEsperaExtremoMs = HAL_GetTick();
  			  }
  		  } else if (HAL_GetTick() - inicioEsperaExtremoMs >= SERVO_BARRIDO_PAUSA_EXTREMOS_MS) {
  			  haciaMaximoServo = !haciaMaximoServo;
  			  esperandoEnExtremo = false;
  		  }
  	  } else if (modoControl == 1U) {
  		  /* Modo manual de calibración (CONTROL_HABILITADO=1): a
  		   * diferencia del barrido (=2), el servo se queda quieto en su
  		   * posición -- solo se mueve cuando llega un downlink de
  		   * SERVO_PULSO_MIN o SERVO_PULSO_MAX, yendo directo a ese valor
  		   * (para posicionar el servo en un punto exacto durante el
  		   * ajuste en banco, sin esperar una vuelta completa del
  		   * barrido). El "llegó un downlink nuevo" se consume como
  		   * bandera (CalibFlash_HayObjetivoManualServo(), fijada en
  		   * calibracion_flash.c en el momento del downlink) -- NO se
  		   * detecta comparando contra el valor anterior, porque el
  		   * valor pedido puede coincidir con el que ya estaba (ej. los
  		   * defaults de fábrica, 1000/2000) y un downlink real igual
  		   * debe mover el servo. */
  		  static uint16_t objetivoManualServoUs;
  		  if (modoControlAnterior != 1U) {
  			  /* Flanco de entrada a este modo -- se queda donde está,
  			   * no salta a MIN/MAX hasta que llegue un downlink real. */
  			  objetivoManualServoUs = Servo_GetPulsoActualUs();
  		  }
  		  if (CalibFlash_HayObjetivoManualServo()) {
  			  objetivoManualServoUs = CalibFlash_GetObjetivoManualServoUs();
  			  CalibFlash_LimpiarObjetivoManualServo();
  		  }
  		  Servo_MoverHacia(objetivoManualServoUs);
  	  } else if (modoControl == 4U) {
  		  /* Calibración automática de SERVO_PULSO_MIN (umbral de
  		   * aceleración) -- a diferencia de 1/2, SÍ mira la RPM en cada
  		   * paso, así que puede entrar sin el motor operando y
  		   * simplemente esperar. Estados: 1=esperando motor,
  		   * 2=midiendo línea base fresca en el SERVO_PULSO_MIN actual,
  		   * 3=barriendo hacia arriba en pasos chicos. Ver README
  		   * sección 10 para el diseño completo y el análisis de riesgo. */
  		  static uint8_t  servoMinCalEstado = 1U;
  		  static uint32_t servoMinCalInicioEtapaMs = 0;
  		  static float    servoMinCalSuma = 0.0f;
  		  static uint32_t servoMinCalMuestras = 0;
  		  static float    servoMinCalBaselineRpm = 0.0f;
  		  static uint16_t servoMinCalPulsoBase = 0;
  		  static uint16_t servoMinCalPulsoPaso = 0;
  		  static uint8_t  servoMinCalConfirmaciones = 0;

  		  if (modoControlAnterior != 4U) {
  			  /* Flanco de entrada -- siempre arranca esperando el motor,
  			   * aunque ya esté operando (el siguiente bloque lo detecta
  			   * en la misma vuelta del loop si ya está corriendo). */
  			  servoMinCalEstado = 1U;
  			  printf("SERVOMIN_CAL,INICIO,esperando_motor=%d\r\n", (int)!motorOperandoAhora);
  		  }

  		  if (servoMinCalEstado == 1U && motorOperandoAhora) {
  			  servoMinCalPulsoBase = CalibFlash_GetServoPulsoMinUs();
  			  servoMinCalEstado = 2U;
  			  servoMinCalInicioEtapaMs = HAL_GetTick();
  			  servoMinCalSuma = 0.0f;
  			  servoMinCalMuestras = 0U;
  			  printf("SERVOMIN_CAL,BASELINE,pulso_base=%u,duracion_s=%lu\r\n",
  				  servoMinCalPulsoBase, (unsigned long)(SERVOMIN_CAL_BASELINE_MS / 1000U));
  		  }

  		  if (servoMinCalEstado == 2U && !motorOperandoAhora) {
  			  printf("SERVOMIN_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  servoMinCalEstado = 1U;
  		  } else if (servoMinCalEstado == 2U) {
  			  servoMinCalSuma += Tacometro_GetRPMFiltrada();
  			  servoMinCalMuestras++;
  			  if (HAL_GetTick() - servoMinCalInicioEtapaMs >= SERVOMIN_CAL_BASELINE_MS) {
  				  servoMinCalBaselineRpm = servoMinCalSuma / (float)servoMinCalMuestras;
  				  servoMinCalPulsoPaso = servoMinCalPulsoBase;
  				  servoMinCalConfirmaciones = 0U;
  				  servoMinCalEstado = 3U;
  				  servoMinCalInicioEtapaMs = HAL_GetTick();
  				  printf("SERVOMIN_CAL,BARRIENDO,baseline_rpm=%.1f,pulso_inicial=%u\r\n",
  					  servoMinCalBaselineRpm, servoMinCalPulsoPaso);
  			  }
  		  }

  		  if (servoMinCalEstado == 3U && !motorOperandoAhora) {
  			  printf("SERVOMIN_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  servoMinCalEstado = 1U;
  		  } else if (servoMinCalEstado == 3U &&
  		             HAL_GetTick() - servoMinCalInicioEtapaMs >= SERVOMIN_CAL_DWELL_MS) {
  			  float rpmActual = Tacometro_GetRPMFiltrada();
  			  float subidaRpm = rpmActual - servoMinCalBaselineRpm;
  			  printf("SERVOMIN_CAL,PASO,pulso=%u,rpm=%.1f,subida=%.1f,confirmaciones=%u\r\n",
  				  servoMinCalPulsoPaso, rpmActual, subidaRpm, servoMinCalConfirmaciones);

  			  if (subidaRpm >= SERVOMIN_CAL_SALTO_ANORMAL_RPM) {
  				  /* Salto mucho mas grande de lo esperado en un solo
  				   * paso -- frenar ya, no seguir empujando ni aplicar
  				   * ningun resultado automatico. */
  				  printf("SERVOMIN_CAL,ABORTADO,motivo=salto_rpm_anormal\r\n");
  				  servoMinCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else if (subidaRpm >= SERVOMIN_CAL_UMBRAL_DETECCION_RPM) {
  				  servoMinCalConfirmaciones++;
  				  if (servoMinCalConfirmaciones >= SERVOMIN_CAL_CONFIRMACIONES_NECESARIAS) {
  					  uint16_t margenUs = (uint16_t)(SERVOMIN_CAL_MARGEN_PASOS * SERVOMIN_CAL_PASO_US);
  					  uint16_t nuevoServoMin = servoMinCalPulsoBase;
  					  if (servoMinCalPulsoPaso > (uint16_t)(servoMinCalPulsoBase + margenUs)) {
  						  nuevoServoMin = servoMinCalPulsoPaso - margenUs;
  					  }
  					  bool ok = CalibFlash_SetServoPulsoMinUs(nuevoServoMin);
  					  printf("SERVOMIN_CAL,COMPLETO,pulso_umbral=%u,SERVO_PULSO_MIN_nuevo=%u,ok=%d\r\n",
  						  servoMinCalPulsoPaso, nuevoServoMin, (int)ok);
  					  CalibFlash_ForzarReporte();
  					  servoMinCalEstado = 1U;
  					  CalibFlash_SetControlHabilitado(0U);
  				  } else {
  					  /* Sostener el mismo pulso un paso mas para
  					   * confirmar antes de dar la deteccion por buena. */
  					  servoMinCalInicioEtapaMs = HAL_GetTick();
  				  }
  			  } else {
  				  servoMinCalConfirmaciones = 0U;
  				  uint16_t siguientePulso = (uint16_t)(servoMinCalPulsoPaso + SERVOMIN_CAL_PASO_US);
  				  if (siguientePulso > (uint16_t)(servoMinCalPulsoBase + SERVOMIN_CAL_TECHO_US)) {
  					  printf("SERVOMIN_CAL,ERROR,motivo=techo_alcanzado_sin_deteccion\r\n");
  					  servoMinCalEstado = 1U;
  					  CalibFlash_SetControlHabilitado(0U);
  				  } else {
  					  servoMinCalPulsoPaso = siguientePulso;
  					  servoMinCalInicioEtapaMs = HAL_GetTick();
  				  }
  			  }
  		  }

  		  /* El pulso real aplicado sigue siempre el estado actual: en
  		   * SERVO_PULSO_MIN mientras se espera el motor o se mide la
  		   * línea base, y en el pulso del paso en curso mientras se
  		   * barre. Servo_MoverHacia() respeta SERVO_VELOCIDAD_MAX_US_S,
  		   * así que cada paso llega rampeado, no de un salto. */
  		  uint16_t destinoServoMinCal = (servoMinCalEstado == 3U) ? servoMinCalPulsoPaso : CalibFlash_GetServoPulsoMinUs();
  		  Servo_MoverHacia(destinoServoMinCal);
  	  } else if (modoControl == 5U) {
  		  /* Mapeo de curva de ganancia -- ver definicion de las
  		   * constantes GANANCIA_CAL_* mas arriba. Barre en lazo ABIERTO
  		   * (sin PID) desde SERVO_PULSO_MIN, logueando (pulso, RPM) en
  		   * cada paso, hasta GANANCIA_CAL_RPM_TECHO o SERVO_PULSO_MAX.
  		   * Solo mide/loguea -- no aplica ninguna correccion todavia.
  		   * Estados: 1=esperando motor, 2=barriendo. */
  		  static uint8_t  ganCalEstado = 1U;
  		  static uint32_t ganCalInicioEtapaMs = 0;
  		  static uint16_t ganCalPulsoPaso = 0;
  		  static float    ganCalRpmAnterior = 0.0f;

  		  if (modoControlAnterior != 5U) {
  			  ganCalEstado = 1U;
  			  printf("GANANCIA_CAL,INICIO,esperando_motor=%d\r\n", (int)!motorOperandoAhora);
  		  }

  		  if (ganCalEstado == 1U && motorOperandoAhora) {
  			  ganCalPulsoPaso = CalibFlash_GetServoPulsoMinUs();
  			  ganCalRpmAnterior = Tacometro_GetRPMFiltrada();
  			  ganCalEstado = 2U;
  			  ganCalInicioEtapaMs = HAL_GetTick();
  			  printf("GANANCIA_CAL,BARRIENDO,pulso_inicial=%u,rpm_techo=%.0f\r\n",
  				  ganCalPulsoPaso, GANANCIA_CAL_RPM_TECHO);
  		  }

  		  if (ganCalEstado == 2U && !motorOperandoAhora) {
  			  printf("GANANCIA_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  ganCalEstado = 1U;
  		  } else if (ganCalEstado == 2U &&
  		             HAL_GetTick() - ganCalInicioEtapaMs >= GANANCIA_CAL_DWELL_MS) {
  			  float rpmActual = Tacometro_GetRPMFiltrada();
  			  float saltoRpm = rpmActual - ganCalRpmAnterior;
  			  printf("GANANCIA_CAL,PASO,pulso=%u,rpm=%.1f,salto=%.1f\r\n",
  				  ganCalPulsoPaso, rpmActual, saltoRpm);
  			  ganCalRpmAnterior = rpmActual;

  			  if (saltoRpm >= GANANCIA_CAL_SALTO_ANORMAL_RPM) {
  				  /* Protege contra pasarse del techo de RPM de un salto
  				   * grande en la zona de mas ganancia, y detecta
  				   * lecturas fuera de lo esperado -- frenar ya. */
  				  printf("GANANCIA_CAL,ABORTADO,motivo=salto_rpm_anormal\r\n");
  				  ganCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else if (rpmActual >= GANANCIA_CAL_RPM_TECHO) {
  				  printf("GANANCIA_CAL,COMPLETO,motivo=techo_rpm_alcanzado,pulso_final=%u,rpm_final=%.1f\r\n",
  					  ganCalPulsoPaso, rpmActual);
  				  ganCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else {
  				  uint16_t siguientePulso = (uint16_t)(ganCalPulsoPaso + GANANCIA_CAL_PASO_US);
  				  uint16_t maximoServo = CalibFlash_GetServoPulsoMaxUs();
  				  if (siguientePulso > maximoServo) {
  					  printf("GANANCIA_CAL,ERROR,motivo=servo_pulso_max_alcanzado_sin_llegar_al_techo\r\n");
  					  ganCalEstado = 1U;
  					  CalibFlash_SetControlHabilitado(0U);
  				  } else {
  					  ganCalPulsoPaso = siguientePulso;
  					  ganCalInicioEtapaMs = HAL_GetTick();
  				  }
  			  }
  		  }

  		  uint16_t destinoGanCal = (ganCalEstado == 2U) ? ganCalPulsoPaso : CalibFlash_GetServoPulsoMinUs();
  		  Servo_MoverHacia(destinoGanCal);
  	  } else if (pidActivoAhora) {
  		  /* Motor operando (en modo 0 normal, o en modo 10 sintonizando
  		   * el PID -- ver arriba, mismo comportamiento del servo en
  		   * ambos), y con un SET_RPM por encima del ralentí -- lazo de
  		   * control real. Setpoint = SET_RPM (downlink directo, uso de
  		   * prueba hasta que exista la maquina Modo 0/1/2, ver README
  		   * seccion 4.3), recortado solo contra el techo RPM_MAX (el
  		   * piso ya lo garantiza controlSolicitado). */
  		  float setpointRpm = setpointRpmCrudo;
  		  if (setpointRpm > rpmMax) setpointRpm = rpmMax;

  		  uint16_t salidaPidUs = PID_CalcularSalidaUs(setpointRpm, Tacometro_GetRPMFiltrada());
  		  Servo_MoverHacia(salidaPidUs);
  	  } else {
  		  /* Sin control activo: motor detenido, o motor en su ralentí
  		   * natural sin SET_RPM comandado por encima de RPM_MIN (Modo
  		   * 0). El servo siempre va (o se queda) en SERVO_PULSO_MIN, la
  		   * posición segura sin aceleración. Cubre también la salida
  		   * del modo calibración por el apagado de seguridad de arriba
  		   * -- en ese caso regresa gradualmente (respetando
  		   * SERVO_VELOCIDAD_MAX_US_S), no de un salto, sin importar en
  		   * qué punto del barrido se haya quedado. */
  		  Servo_MoverHacia(CalibFlash_GetServoPulsoMinUs());
  	  }
  	  modoControlAnterior = modoControl;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  hlpuart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  hlpuart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 99;
  hrtc.Init.SynchPrediv = 319;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    Tacometro_CaptureCallback(htim);
}

#ifdef __GNUC__
int __io_putchar(int ch)
{

    HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
     /* RAK3172 ahora en USART1 */
    if (huart->Instance == USART1) {
        RAK3172_RxEventCallback(huart, Size);
    }
    /* GPS (SIM7600X) en USART2 -- ver gps.h */
    else if (huart->Instance == USART2) {
        GPS_RxEventCallback(huart, Size);
    }
}

/* Comando manual por serial (LPUART1, ver comando_serial.h) -- lee un
 * byte a la vez por interrupción en vez de polling, para no perder
 * caracteres mientras __io_putchar()/printf() satura el mismo UART con
 * transmisiones bloqueantes (ver ComandoSerial_RxCpltCallback). */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1) {
        ComandoSerial_RxCpltCallback(huart);
    }
}

/* Sin este callback, un solo error de UART (overrun/framing/ruido --
 * ej. el USB-a-PC metiendo ruido al plano de tierra, ver hallazgos de
 * hardware) deja la recepcion por DMA de esa UART muerta para el
 * resto de la sesion: HAL la aborta internamente al entrar en error y
 * HAL_UARTEx_RxEventCallback() nunca vuelve a dispararse para ella,
 * aunque el modulo del otro lado (RAK3172 o GPS) siga funcionando
 * bien -- todo comando futuro por esa UART solo expira por TIMEOUT sin
 * que el host jamas vea la respuesta real. Coincide con el sintoma de
 * campo "dejo de responder y ya no volvio a intentar". */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        RAK3172_ErrorCallback(huart);
    } else if (huart->Instance == USART2) {
        GPS_ErrorCallback(huart);
    } else if (huart->Instance == LPUART1) {
        ComandoSerial_ErrorCallback(huart);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
