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
#include "modo_fsm.h"
#include "rak3172.h"
#include "servo.h"
#include "pid.h"
#include "autotune_rpm.h"
#include "autotune_psi.h"
#include "autotune_asp.h"
#include "rtc_reloj.h"
#include "gps.h"
#include "comando_serial.h"
#include "presion.h"
#include "presion_voltaje.h"
#include "presion_pid.h"
#include "presion_pid_remoto.h"
#include "numero_texto.h"
#include "horometro.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INTERVALO_ENVIO_RPM_MS   30000U   // ritmo del resync GPS (el uplink LIVE usa INTERVALO_ENVIO_OPERATIVO_S/STANDBY_S desde 2026-10-01)

/* Debounce del estado del motor (ENCENDIDO/APAGADO/ACTIVO) antes de
 * confirmarlo para telemetria/inicio_operacion -- ver comentario junto a
 * estadoCandidato en el superloop. Un motor diesel real tarda varios
 * segundos en pasar de detenido a régimen; 3s es sobrado para filtrar
 * una rafaga de ruido de <1s en el tacometro sin retrasar de forma
 * perceptible una transicion real. */
#define ESTADO_DEBOUNCE_MS   3000U

/* Auto-calibracion de ralenti (CALIB=8) --
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

/* Auto-calibracion de SERVO_PULSO_MIN (CALIB=4) -- barre el
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

/* Auto-calibracion de ALPHA -- CALIB=3 (ver
 * calibracion_flash.c) -- mide la dispersion real de RPM_instantanea
 * (sin filtrar) durante una ventana corta con el motor ya estable, y
 * calcula el ALPHA necesario para que la RPM ya filtrada quede dentro
 * de un objetivo de ruido fijo (ALPHA_CAL_OBJETIVO_DESVIACION_RPM),
 * usando la relacion de atenuacion de un filtro EMA:
 * alpha = 2r^2/(1+r^2), con r = objetivo/desviacion_cruda. No mueve el
 * servo (en CALIB=3 no hay control activo: queda en SERVO_PULSO_MIN).
 * Ver README seccion 12.
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

/* CALIB 5/6/7 (PID#1) y 9/11 (PID#2) ya no viven aca: desde 2026-10-01
 * son etapas sueltas de los autotunes, ver autotune_rpm.h / autotune_psi.h. */

/* El RTC de este nodo corre del LSI interno (~32kHz nominal, sin cristal
 * externo -- ver hallazgos de hardware), que no esta calibrado ni
 * compensado por temperatura. Deriva medida en campo: ~0.8% (equivale a
 * ~12 min/dia sin correccion). Por eso se resincroniza periodicamente
 * en vez de una sola vez al arrancar -- ver bloque de sincronizacion en
 * el superloop.
 *
 * Fuente PRIMARIA: hora UTC del propio GPS (satelital, mas precisa, y
 * no compite por el canal AT del RAK3172 ni gasta duty cycle) -- se
 * intenta cada INTERVALO_RESYNC_GPS_MS (30 s) si hay fix vivo. Intentarlo
 * no cuesta nada, y el SIM7600X solo refresca su reporte cada 10 s
 * (AT+CGPSINFO=10), asi que preguntar mas seguido no trae dato mas fresco.
 * Fuente de RESPALDO: DeviceTimeReq por LoRaWAN -- compite libremente
 * con el GPS para el primer sync (el que llegue primero gana). Una vez
 * que YA hubo un sync exitoso (de cualquier fuente), el respaldo por
 * LoRa se limita a activarse solo si pasan INTERVALO_FALLBACK_LORA_MS
 * sin ningun resync exitoso nuevo (ej. GPS sin vista al cielo por un
 * rato largo) -- para no competir por el canal AT del RAK3172 sin
 * necesidad mientras el GPS este sincronizando bien. */
#define INTERVALO_RESYNC_GPS_MS      INTERVALO_ENVIO_RPM_MS   // intentar resync por GPS cada 30 s
#define INTERVALO_FALLBACK_LORA_MS   (30UL * 60UL * 1000UL)   // forzar resync por LoRaWAN si no hay exito en 30 min
#define SENSOR_PRESION_FALLA_A_MODO0_MS  (60UL * 1000UL)   // sensor de presion local invalido este tiempo seguido en MODO 1/2/3 -> MODO 0 (constante, no parametro de flash: no bumpea el magic)
#define ENLACE_FALLAS_COMANDO_MAX    3U // comandos AT seguidos con resultado != 0 para considerar el enlace malo (ver bloque "Enlace LoRaWAN perdido" en main loop)

/* El propio RAK3172 ya reintenta el join 8 veces cada 10s dentro de UN
 * solo comando (ver AT+JOIN=1:0:10:8 en RAK3172_Join(), ~80s en total),
 * pero si esas 8 fallan (o el modulo se queda sin cobertura un rato)
 * nada volvia a pedir el join de nuevo -- el nodo quedaba sin unirse
 * para siempre. 2 min da margen sobre esos ~80s antes de reintentar. */
#define INTERVALO_REINTENTO_JOIN_MS  (120UL * 1000UL)
/* ⚠️ SUPUESTO / placeholder: por ahora hardcodeado a 1 (equivale a
 * "DSL-0001"). Cuando exista mas de un nodo motor, resolver esto desde
 * el DevEUI/nombre del dispositivo (habia un parametro NODE_ID, se elimino
 * 2026-10-01 sin haberse usado nunca). */
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

/* Supervisor de "no se puede llegar a PRESION_OBJETIVO_LOCAL" en MODO=1 (ver
 * el bloque MODO mas abajo, y README secciones 4.3/8/12). Cada "intento
 * fallido" es una ventana sostenida de esta duracion con el lazo
 * saturado en RPM_MAX_CARGA y la presion fuera de tolerancia -- tras
 * PRESION_SUPERVISOR_INTENTOS_MAX ventanas seguidas, alerta por serial y
 * por LoRa (ALERTA_PRESION_OBJETIVO_NO_ALCANZADA). Solo avisa, no actua. */
#define PRESION_SUPERVISOR_VENTANA_MS            30000U
#define PRESION_SUPERVISOR_INTENTOS_MAX              3U
#define PRESION_SUPERVISOR_TOLERANCIA_FRACCION     0.05f

/* Supervisor de fuga de MODO=2 (escenario real: fuga entre el motor y el
 * aspersor -- el aspersor nunca alcanza su objetivo por mas RPM que se
 * pida). A diferencia del de MODO=1, este SI actua: cada reporte que
 * confirma "saturado y lejos del objetivo" retrocede a ralenti durante
 * PRESION_SUPERVISOR_REMOTO_RETROCESO_MS antes de reintentar; al llegar a
 * PRESION_SUPERVISOR_REMOTO_INTENTOS_MAX (3 retrocesos + el 4to fallo)
 * manda la alerta y pasa a MODO=0. */
#define PRESION_SUPERVISOR_REMOTO_RETROCESO_MS   20000U
#define PRESION_SUPERVISOR_REMOTO_INTENTOS_MAX       4U

/* MODO=2 (triple cascada, 2026-09-30): el objetivo LOCAL que pide el PID#3
 * nunca pasa de este porcentaje de PRESION_MAX -- deja margen para que la
 * guarda de PRESION_MAX no sea la que corte en operacion normal. Mismo valor
 * que usan las autosintonias de presion. */
#define REMOTO_TECHO_FRAC                        0.95f

/* Codigos de alerta mandados por LoRa en el byte 27 del uplink LIVE
 * (ver RAK3172_EnviarUplinkLive(), payload de 28 bytes).
 * s_codigoAlertaActual es un SNAPSHOT del estado actual (0 = sin alerta)
 * y va en CADA uplink LIVE. Para saber CUANDO ocurrio una transicion,
 * cada punto que cambia la alerta llama a CalibFlash_ForzarReporte(): el
 * uplink inmediato lleva en fechaHoraLocal el timestamp del evento. Si se
 * agrega una alerta nueva, sumarla aca Y a NOMBRES_ALERTA en decoder.py
 * (AWS). */
typedef enum {
    ALERTA_SIN_ALERTA                          = 0,
    ALERTA_PRESION_OBJETIVO_NO_ALCANZADA       = 1, /* supervisor MODO=1 */
    ALERTA_MODO_REMOTO_OBJETIVO_NO_ALCANZADO   = 2, /* supervisor MODO=2 (fuga) */
    ALERTA_SENSOR_PRESION_LOCAL_FALLA          = 3, /* MODO=1, sensor invalido */
    ALERTA_MODO_REMOTO_SIN_PRESION_VALIDA      = 4, /* MODO=2 sin reporte del aspersor en TIMEOUT_SIN_COMANDO_S -> MODO=1 */
    ALERTA_PRESION_MAX_EXCEDIDA                = 5, /* guarda PRESION_MAX, cualquier modo != RALENTI */
    /* Resultado de la autosintonia (CALIB=13/14, ver autotune.h) -- NO son
     * fallas: es la forma de que el operador vea el resultado por LoRa, sin
     * laptop. Se limpian al iniciar otra autosintonia o al salir de
     * MODO=CALIBRACION. Sumar a NOMBRES_ALERTA de decoder.py (AWS). */
    ALERTA_AUTOTUNE_PID1_OK                    = 6,
    ALERTA_AUTOTUNE_PID1_FALLO                 = 7,
    ALERTA_AUTOTUNE_PID2_OK                    = 8,
    ALERTA_AUTOTUNE_PID2_FALLO                 = 9,
    ALERTA_AUTOTUNE_PID3_OK                    = 10,
    ALERTA_AUTOTUNE_PID3_FALLO                 = 11,
    /* Enlace LoRaWAN perdido (agregado 2026-10-01): main.c degrado MODO
     * por si solo (2->1, 3->1 o 3->0, 4->0). Se limpia cuando el enlace
     * vuelve. Sumar a NOMBRES_ALERTA de decoder.py (AWS). */
    ALERTA_ENLACE_PERDIDO                      = 12,
    /* Un downlink/comando MODO fue RECHAZADO (motor detenido, objetivo remoto sin
     * configurar, o MODO 2 sin venir de MODO 1). Es TRANSITORIA: va en el uplink
     * inmediato que se fuerza y despues se restaura la alerta anterior. Sumar a
     * NOMBRES_ALERTA de decoder.py (AWS). */
    ALERTA_MODO_RECHAZADO                      = 13
} CodigoAlerta_t;

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

/* Alerta activa AHORA MISMO (ver CodigoAlerta_t arriba). File-scope porque
 * el envio del uplink LIVE esta antes en el loop que los bloques que la
 * calculan. */
static CodigoAlerta_t s_codigoAlertaActual = ALERTA_SIN_ALERTA;
/* Alerta transitoria "modo rechazado": se guarda la alerta que habia para
 * restaurarla despues de enviar el uplink que la lleva (ver main loop). */
static bool alertaRechazoActiva = false;
static CodigoAlerta_t alertaAntesDeRechazo = ALERTA_SIN_ALERTA;

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

  /* Diagnostico de desgaste de flash (2026-10-01): el STM32G4 tiene ECC en
   * flash y FLASH->ECCR marca si hubo un bit corregido (ECCC) o un error no
   * corregible (ECCD) desde el ultimo reset, con la direccion afectada. Se
   * lee ANTES de CalibFlash_Init() (que lee la pagina de calibracion) y se
   * limpia escribiendo 1. */
  uint32_t eccr = FLASH->ECCR;
  printf("FLASH: ECC corregido=%d no_corregible=%d dir=0x%05lX\r\n",
         (eccr & FLASH_ECCR_ECCC) ? 1 : 0, (eccr & FLASH_ECCR_ECCD) ? 1 : 0,
         (unsigned long)(eccr & FLASH_ECCR_ADDR_ECC));
  FLASH->ECCR = eccr & (FLASH_ECCR_ECCC | FLASH_ECCR_ECCD);

  CalibFlash_Init();
  printf("FLASH: pagina de calibracion %s\r\n",
         CalibFlash_ArranqueConDefaults() ? "INVALIDA (magic no coincide) -- valores por defecto" : "valida");
  Horometro_Init();
  Reloj_Init(&hrtc);

  /* 2026-10-01: ya NO se carga ni se guarda una "ultima hora conocida" en
   * flash. Hasta el primer sync real (GPS o LoRaWAN) el RTC queda en el
   * default de MX_RTC_Init() (ano 2000) a proposito: una fecha obviamente
   * invalida es mejor que una plausible pero vieja (desfasada los dias que
   * estuvo apagado el equipo), y la hora real llega sola en el primer sync. */

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

  /* AT+MASK=0002 (sub-banda 2; misma sintaxis en AU915, canales 8-15 +
   * 65): fix real y necesario para el bug
   * documentado de RUI3 en US915/AU915 (con AT+MASK=00FF el join falla con
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

  /* Consola serial en el puerto de debug (LPUART1): parametros y
   * pass-through AT al RAK3172 -- ver comando_serial.h (permanente). */
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

  	  RAK3172_Update();
  	  GPS_Update();
  	  ComandoSerial_Update();

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
  	  /* Ademas de lo anterior: si el propio modulo pidio un backoff
  	   * ("Restricted_Wait_<ms>_ms", ver RAK3172_EstaRestringido()), no
  	   * tiene sentido reintentar el join durante ese tiempo -- no va a
  	   * lograr nada y puede resetear/extender la restriccion. Se
  	   * pausa el reintento entero (incluida la reafirmacion de
  	   * AT+MASK) hasta que pase. */
  	  static uint32_t ultimoIntentoJoinTick = 0;
  	  static bool reenviandoMaskAntesDeJoin = false;
  	  if (RAK3172_EstaRestringido()) {
  		  /* no tocar reenviandoMaskAntesDeJoin -- si ya se habia mandado
  		   * el MASK, se completa el paso 2 (JOIN) en cuanto la
  		   * restriccion se levante, en vez de perder ese progreso. */
  	  } else if (reenviandoMaskAntesDeJoin && RAK3172_ComandoListo()) {
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
  			  /* La hora del GPS es la del MOMENTO del reporte (uno cada 10 s):
  			   * se le suma lo que paso desde entonces, redondeado al segundo,
  			   * para no dejar el RTC hasta 10 s atrasado (2026-10-02). */
  			  uint32_t edadReporteS = (GPS_GetEdadReporteMs() + 500UL) / 1000UL;
  			  Reloj_SetUnixTimeUtc(Reloj_CalendarioAEpoch(anioGps, mesGps, diaGps,
  					  horaGps, minutoGps, segundoGps) + edadReporteS);
  			  relojFueCorregidoEsteCiclo = true;
  			  ultimoResyncExitosoTick = HAL_GetTick();
  			  huboResyncAlgunaVez = true;
  			  printf("GPS: reloj sincronizado con hora satelital: %02u:%02u:%02u UTC, %02u/%02u/%04u (+%lus de edad del reporte)\r\n",
  					 horaGps, minutoGps, segundoGps, mesGps, diaGps, anioGps, (unsigned long)edadReporteS);
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
			   * ejemplo. Se lee a mano campo por campo (antes sscanf,
			   * 2026-10-01: ahorra el lector de la libreria de C):
			   * cada numero seguido de su separador. */
			  static const char *const sepLtime[6] = { "h", "m", "s on ", "/", "/", "" };
			  static const uint8_t digLtime[6] = { 2U, 2U, 2U, 2U, 2U, 4U };
			  uint32_t campoLtime[6] = { 0 };
			  const char *cursorLtime = respuesta + 9; /* tras "AT+LTIME=" */
			  uint8_t camposLeidos = 0U;
			  if (strncmp(respuesta, "AT+LTIME=", 9) == 0) {
				  while (camposLeidos < 6U
						 && NumeroTexto_LeerUint(&cursorLtime, digLtime[camposLeidos], &campoLtime[camposLeidos])
						 && strncmp(cursorLtime, sepLtime[camposLeidos], strlen(sepLtime[camposLeidos])) == 0) {
					  cursorLtime += strlen(sepLtime[camposLeidos]);
					  camposLeidos++;
				  }
			  }
			  unsigned int hora = campoLtime[0], minuto = campoLtime[1], segundo = campoLtime[2];
			  unsigned int mes = campoLtime[3], dia = campoLtime[4], anio = campoLtime[5];
			  if (camposLeidos == 6U) {
				  Reloj_SetHoraUtc((uint16_t)anio, (uint8_t)mes, (uint8_t)dia,
								   (uint8_t)hora, (uint8_t)minuto, (uint8_t)segundo);
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
	   * CALIB a 0 al arrancar el motor (mas abajo) sigue
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

	  /* Horometro (2026-10-02): suma con el motor ENCENDIDO o ACTIVO (estado
	   * ya con debounce), guarda al apagarse y cada 30 min. */
	  Horometro_Update(estadoActual != ESTADO_APAGADO);

	  uint32_t fechaHoraLocalIterActual = Reloj_GetUnixTimeLocal();

	  if (!estadoInicializado || estadoActual != estadoAnterior) {
		  /* Cambio de estado (APAGADO/ENCENDIDO/ACTIVO, ya con debounce):
		   * uplink LIVE inmediato (2026-10-02), igual que con las alertas --
		   * si no, GIO seguia mostrando el estado viejo hasta el proximo envio
		   * periodico (hasta INTERVALO_ENVIO_STANDBY_S con el motor apagado).
		   * Su fecha_hora marca el momento del cambio. No en el primer
		   * calculo del arranque (no es un cambio real). */
		  if (estadoInicializado) {
			  printf("ESTADO_MOTOR: %u -> %u (1=ACTIVO, 2=APAGADO, 3=ENCENDIDO) -- uplink inmediato\r\n",
					 (unsigned)estadoAnterior, (unsigned)estadoActual);
			  CalibFlash_ForzarReporte();
		  }
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
  	  /* Envío periódico del uplink LIVE, en cuanto hay red. Sin hora
  	   * sincronizada, fecha_hora/inicio_operacion van en 0 ("Sin hora"). */
	  static uint32_t ultimoEnvioRPM = 0;
	  /* Intervalo configurable (IDs 15/16, conectado 2026-10-01): motor
	   * apagado -> INTERVALO_ENVIO_STANDBY_S, si no INTERVALO_ENVIO_OPERATIVO_S. */
	  uint32_t intervaloEnvioMs = 1000UL * ((estadoActual == ESTADO_APAGADO)
			  ? CalibFlash_GetIntervaloEnvioStandbyS() : CalibFlash_GetIntervaloEnvioOperativoS());
	  bool tocaEnviarPorIntervalo = (HAL_GetTick() - ultimoEnvioRPM >= intervaloEnvioMs);
	  bool tocaEnviarPorForzado = CalibFlash_HayReporteForzado();

	  if (RAK3172_EstaUnido() && (tocaEnviarPorIntervalo || tocaEnviarPorForzado) && RAK3172_ComandoListo()
			  && !RAK3172_EstaRestringido()) {
  		  ultimoEnvioRPM = HAL_GetTick();

  		  /* "Sin hora" (2026-10-01): mientras el RTC no se haya sincronizado ni
  		   * por GPS ni por LoRaWAN, fecha_hora e inicio_operacion se mandan en
  		   * 0 (el decoder lo muestra como "Sin hora") en vez de una hora
  		   * inventada -- por eso ya no se guarda ninguna hora en flash.
  		   * segundosTranscurridos SI es valido: es una diferencia entre dos
  		   * lecturas del mismo reloj. Tras el primer sync, inicioEstadoLocal
  		   * ya viene desplazado por el salto (ver relojFueCorregidoEsteCiclo). */
  		  bool sinHora = !Reloj_EstaSincronizado();
  		  uint32_t segundosTranscurridos = fechaHoraLocalIterActual - inicioEstadoLocal;
  		  uint32_t fechaHoraLocal = sinHora ? 0U : fechaHoraLocalIterActual;
  		  uint32_t inicioOperacionUplink = sinHora ? 0U : inicioEstadoLocal;

  		  /* "Sin posicion" (2026-10-01): sin fix GPS vivo se manda 0/0 (el
  		   * decoder lo interpreta) -- ya no hay posicion guardada en flash
  		   * ni coordenada fija de respaldo, mismo criterio que "Sin hora". */
  		  bool hayFix = GPS_TieneFix();
  		  float latitudActual = hayFix ? GPS_GetLatitud() : 0.0f;
  		  float longitudActual = hayFix ? GPS_GetLongitud() : 0.0f;

  		  bool encolado = RAK3172_EnviarUplinkLive(
  			  MOTOR_ID_NUMERIC,
  			  rpmActual,
  			  presionActual,
  			  estadoActual,
  			  fechaHoraLocal,
  			  inicioOperacionUplink,
  			  segundosTranscurridos,
  			  latitudActual,
  			  longitudActual,
  			  (uint8_t)s_codigoAlertaActual,
  			  0xFFU,                              /* bateria del nodo: sin hardware de medicion todavia */
  			  Horometro_GetSegundos() / 360UL     /* decimas de hora */
  		  );

  		  (void)encolado;
  		  /* Alerta "modo rechazado" ya enviada: se restaura la que habia. Si
  		   * mientras tanto otra alerta la reemplazo, se deja esa. */
  		  if (encolado && alertaRechazoActiva) {
  			  if (s_codigoAlertaActual == ALERTA_MODO_RECHAZADO) {
  				  s_codigoAlertaActual = alertaAntesDeRechazo;
  			  }
  			  alertaRechazoActiva = false;
  		  }
  		  if (tocaEnviarPorForzado) {
  			  CalibFlash_LimpiarReporteForzado();
  		  }

	  }
  	  /* REPORTAR_PARAMETROS (ID 27, 2026-10-01): al arrancar (en cuanto hay
  	   * red) y cada vez que se pide, manda el valor guardado de cada
  	   * parametro por FPort 3, en partes de 7 grupos [ID][8][VAL_H][VAL_L]
  	   * (28 bytes, el tamano del LIVE). Una parte cada 5 s como minimo,
  	   * cediendo el canal al LIVE y al ACK. Si el modulo rechaza una parte
  	   * (resultado != OK) se reintenta la misma hasta 3 veces. */
  	  {
  		  static uint8_t  reporteIndice = 0U;      /* fila de la tabla donde empieza la parte actual */
  		  static uint8_t  reporteIndiceSig = 0U;   /* fila donde empieza la siguiente */
  		  static uint8_t  reporteIntentos = 0U;
  		  static bool     reporteEsperando = false; /* parte enviada, falta ver el resultado */
  		  static uint32_t reporteUltimoMs = 0U;
  		  #define REPORTE_GRUPOS_POR_PARTE  7U
  		  #define REPORTE_SEPARACION_MS     5000U

  		  if (reporteEsperando && RAK3172_ComandoListo()) {
  			  reporteEsperando = false;
  			  if (RAK3172_GetUltimoResultado() == RAK3172_OK || reporteIntentos >= 3U) {
  				  reporteIndice = reporteIndiceSig; /* siguiente parte */
  				  reporteIntentos = 0U;
  			  }
  		  }
  		  if (CalibFlash_HayReportePendiente() && !reporteEsperando
  				  && RAK3172_EstaUnido() && RAK3172_ComandoListo() && !RAK3172_EstaRestringido()
  				  && !RAK3172_HayAckPendiente()
  				  && (HAL_GetTick() - reporteUltimoMs >= REPORTE_SEPARACION_MS)) {
  			  uint8_t parte[REPORTE_GRUPOS_POR_PARTE * 4U];
  			  reporteIndiceSig = reporteIndice;
  			  uint8_t len = CalibFlash_ArmarReporte(&reporteIndiceSig, parte, REPORTE_GRUPOS_POR_PARTE);
  			  if (len == 0U) {
  				  printf("REPORTE_PARAMETROS,FIN\r\n");
  				  CalibFlash_TerminarReporte();
  				  reporteIndice = 0U;
  				  reporteIntentos = 0U;
  			  } else if (RAK3172_EnviarUplink(RAK3172_FPORT_ACK, parte, len)) {
  				  printf("REPORTE_PARAMETROS,PARTE,grupos=%u,intento=%u\r\n",
  						 (unsigned)(len / 4U), (unsigned)(reporteIntentos + 1U));
  				  reporteEsperando = true;
  				  reporteIntentos++;
  				  reporteUltimoMs = HAL_GetTick();
  			  }
  		  }
  	  }
  	  /* RESET_REMOTO (conectado 2026-10-01). Durante el arranque el servo
  	   * queda sin PWM ~2.5-8.5 s (hasta Servo_Init mas arriba); el LD-25MG
  	   * se queda en su posicion (confirmado por el usuario), y despues MODO
  	   * arranca en RALENTI. Por eso solo se acepta con el motor detenido o
  	   * en ralenti sin nada comandado (CalibFlash_ResetEsSeguro). Espera a
  	   * que salga el ACK (minimo 2 s, maximo 15 s) y vuelve a verificar la
  	   * regla justo antes, por si algo cambio mientras tanto. */
  	  if (CalibFlash_HayResetPendiente()) {
  		  uint32_t esperaMs = HAL_GetTick() - CalibFlash_GetResetPendienteTickMs();
  		  bool ackSalio = !RAK3172_HayAckPendiente() && RAK3172_ComandoListo();
  		  if ((ackSalio && esperaMs >= 2000U) || esperaMs >= 15000U) {
  			  if (CalibFlash_ResetEsSeguro(!Tacometro_EstaDetenido())) {
  				  printf("RESET_REMOTO,EJECUTANDO,espera_ms=%lu\r\n", (unsigned long)esperaMs);
  				  HAL_Delay(100); /* que salga el printf por la UART */
  				  NVIC_SystemReset();
  			  }
  			  printf("RESET_REMOTO,CANCELADO,motivo=motor_fuera_de_ralenti\r\n");
  			  CalibFlash_LimpiarResetPendiente();
  		  }
  	  }

  	  /* Reporte de resultado del último comando AT (join o send).
  	   * ⚠️ Detecta el FLANCO de "comando recien terminado" (en_curso ->
  	   * listo), NO un cambio de VALOR -- con el chequeo viejo
  	   * (imprimir solo si resultadoActual != el ultimo mostrado), dos
  	   * comandos consecutivos con el MISMO resultado (ej. TIMEOUT tras
  	   * TIMEOUT si el modulo deja de responder) se volvian invisibles
  	   * despues del primero: el log parecia "dejo de intentar" cuando
  	   * en realidad seguia intentando y fallando cada vez igual. */
  	  static bool comandoEnCursoAnterior = true;
  	  static uint8_t fallasComandoConsecutivas = 0U; /* resultado != 0 seguidos -- ver bloque de enlace LoRaWAN mas abajo */
  	  bool comandoEnCursoAhora = !RAK3172_ComandoListo();
  	  if (comandoEnCursoAnterior && !comandoEnCursoAhora) {
  		  printf("RAK3172: resultado=%d (0=OK,1=ERROR,2=TIMEOUT,3=BUSY)\r\n", RAK3172_GetUltimoResultado());
  		  if (RAK3172_GetUltimoResultado() == 0) {
  			  fallasComandoConsecutivas = 0U;
  		  } else if (fallasComandoConsecutivas < 255U) {
  			  fallasComandoConsecutivas++;
  		  }
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

  		  printf("Uptime: %02lu:%02lu:%02lu | Frecuencia: %.1f Hz | RPM: %.1f | MODO: %d | RuidoFiltrado: %lu | PresionV: %.2f PSI (%.3fV%s)\r\n",
  			  (unsigned long)uptimeH,
  			  (unsigned long)uptimeM,
  			  (unsigned long)uptimeSeg,
  			  Tacometro_GetFrecuenciaHz(),
  			  Tacometro_GetRPMFiltrada(),
  			  (int)CalibFlash_GetModo(),
  			  Tacometro_GetContadorRuidoFiltrado(),
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
  	   * Gateado por las etapas del autotune PID#1 que tienen escalon
  	   * (CALIB 6 = identificacion, 7 = validacion, 13 = completo, ver
  	   * autotune_rpm.h). Fuera de esas no imprime nada. */
  	  static uint32_t ultimoLogPrueba = 0;
  	  if ((CalibFlash_GetCalib() == CALIB_PID1_ID
  	       || CalibFlash_GetCalib() == CALIB_PID1_VALIDAR
  	       || CalibFlash_GetCalib() == CALIB_AUTOTUNE_PID1)
  	      && HAL_GetTick() - ultimoLogPrueba >= 200U) {
  		  ultimoLogPrueba = HAL_GetTick();
  		  printf("PID_TEST,%lu,%.1f,%.1f,%u\r\n",
  			  (unsigned long)HAL_GetTick(),
  			  CalibFlash_GetSetRpm(),
  			  Tacometro_GetRPMFiltrada(),
  			  Servo_GetPulsoActualUs());
  	  }

  	  /* Imprime las ganancias vigentes de los 3 PID apenas quedan habilitadas
  	   * para tocarse (MODO=CALIBRACION + CALIB=0), y cada vez que cambia
  	   * Kp/Ki del PID#1 mientras se sigue ahi. Nacio tras un incidente de
  	   * campo: un cambio de magic dejo Kp/Ki de fabrica sin que se notara. */
  	  static bool     gananciasHabilitadasAntes = false;
  	  static float    pidRpmKpAnteriorImpreso = 0.0f;
  	  static float    pidRpmKiAnteriorImpreso = 0.0f;
  	  bool gananciasHabilitadasAhora = (CalibFlash_GetModo() == CALIB_MODO_CALIBRACION)
  			  && (CalibFlash_GetCalib() == 0U);
  	  if (gananciasHabilitadasAhora) {
  		  float pidRpmKpActual = CalibFlash_GetPidRpmKp();
  		  float pidRpmKiActual = CalibFlash_GetPidRpmKi();
  		  float pidPsiKpActual = CalibFlash_GetPidPsiKp();
  		  float pidPsiKiActual = CalibFlash_GetPidPsiKi();
  		  float pidAspKpActual = CalibFlash_GetPidAspKp();
  		  float pidAspKiActual = CalibFlash_GetPidAspKi();
  		  if (!gananciasHabilitadasAntes ||
  		      pidRpmKpActual != pidRpmKpAnteriorImpreso ||
  		      pidRpmKiActual != pidRpmKiAnteriorImpreso) {
  			  printf("PID_1_GANANCIAS,Kp=%.4f,Ki=%.4f\r\n"
  				 "PID_2_GANANCIAS,Kp=%.4f,Ki=%.4f\r\n"
  				 "PID_3_GANANCIAS,Kp=%.4f,Ki=%.4f\r\n",
  				pidRpmKpActual, pidRpmKiActual,
				pidPsiKpActual, pidPsiKiActual,
				pidAspKpActual, pidAspKiActual
				);
  			  pidRpmKpAnteriorImpreso = pidRpmKpActual;
  			  pidRpmKiAnteriorImpreso = pidRpmKiActual;
  		  }
  	  }
  	  gananciasHabilitadasAntes = gananciasHabilitadasAhora;

  	  /* Log de calibración del servo -- gateado a CALIB=1
  	   * (manual) o =2 (barrido), ver README sección 2.4/4.4. Antes de
  	   * esto, la única forma de saber los límites vigentes en banco era
  	   * el ACK de cada downlink de SERVO_PULSO_MIN/MAX ("valor
  	   * vigente=X") -- esto muestra en vivo, mientras se observa el
  	   * servo moverse, dónde están los límites configurados y qué pulso
  	   * se le está aplicando en ese instante. Cadencia de 1s (a
  	   * diferencia de PID_TEST, esto es solo para lectura humana, no se
  	   * analiza después con un script). */
  	  static uint32_t ultimoLogServoCal = 0;
  	  uint8_t modoParaLogServo = CalibFlash_GetCalib();
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
  	   * está en modo de calibración del SERVO (CALIB=1 o
  	   * =2), se sale de inmediato -- sin esperar ningún downlink -- para
  	   * no seguir moviendo el servo (en barrido o manual) con el motor
  	   * operando. Mismo criterio que el resto del firmware
  	   * (Tacometro_EstaDetenido()), nunca un switch remoto para algo de
  	   * seguridad física. Solo CALIB 1/2: las demas pruebas necesitan el
  	   * motor operando y tienen su propio manejo. */
  	  uint8_t modoCalibracionServo = CalibFlash_GetCalib();
  	  if (!Tacometro_EstaDetenido() && (modoCalibracionServo == 1U || modoCalibracionServo == 2U)) {
  		  CalibFlash_SetCalib(0U);
  		  CalibFlash_LimpiarObjetivoManualServo(); /* invalida cualquier
  		                                             * objetivo manual pendiente,
  		                                             * no arrastrarlo a una
  		                                             * futura sesion de calibracion */
  		  CalibFlash_SetSetRpm(0.0f); /* un SET_RPM viejo no debe activar el
  		                                 * PID solo al volver a CALIB=0 */
  	  }

  	  /* Al SALIR de MODO 4, CALIB vuelve a 0 y la prueba se cancela, para
  	   * TODOS los CALIB (Nota 2 de la hoja "Maquina de estados"). El candado
  	   * de calibracion_flash.c solo se evalua al ACEPTAR el downlink, asi que
  	   * sin este chequeo continuo una prueba seguia corriendo (o quedaba
  	   * armada) despues de cambiar de MODO. Unica excepcion: CALIB 12
  	   * (autotune PID#3), que tambien corre desde MODO 1. Las rutinas ya
  	   * arrancadas detectan el cambio y restauran sus valores solas. */
  	  uint8_t modoCalibracionSinPropioChequeoDeModo = CalibFlash_GetCalib();
  	  if (modoCalibracionSinPropioChequeoDeModo != 0U
  	      && CalibFlash_GetModo() != CALIB_MODO_CALIBRACION
  	      && !(modoCalibracionSinPropioChequeoDeModo == CALIB_AUTOTUNE_PID3
  	           && CalibFlash_GetModo() == CALIB_MODO_PRESION_LOCAL)) {
  		  CalibFlash_SetCalib(0U);
  		  CalibFlash_LimpiarObjetivoManualServo();
  		  CalibFlash_SetSetRpm(0.0f);
  	  }

  	  bool motorOperandoAhora = !Tacometro_EstaDetenido();
  	  float rpmMin = CalibFlash_GetRpmMin();
  	  float rpmMax = CalibFlash_GetRpmMax();
  	  /* RPM_MAX_CARGA: techo CON CARGA (hidraulico, de esta instalacion,
  	   * SIEMPRE <= rpmMax, el techo MECANICO del motor). Aplica cuando la
  	   * bomba esta embragada (modoConCarga, mas abajo). */
  	  float rpmMaxCarga = CalibFlash_GetRpmMaxCarga();

  	  /* Forzar MODO=RALENTI cada vez que el motor se detiene (caso real de
  	   * campo: el motor se queda sin diesel con MODO=1/2, tarda horas en
  	   * reanudarse y la tuberia se vacia). Sin esto, al reencender el PID de
  	   * presion arrancaria "en caliente" (sin PresionPid_Init() ni rampa de
  	   * llenado, que solo se rearman con un flanco de MODO) contra una
  	   * lectura vieja, y podria pedir RPM alta de golpe. Asi el operador
  	   * SIEMPRE confirma MODO=1/2 a mano despues de un apagado. */
  	  static bool motorOperandoAntesParaModoSeguro = false;
  	  /* MODO=4 (CALIBRACION) queda EXENTO (2026-10-01, decision del usuario,
  	   * ver hoja "Maquina de estados"): permite motor apagado, y si se
  	   * apaga estando en MODO=4 se mantiene en MODO=4. Las pruebas CALIB
  	   * en curso ya se abortan solas cuando el motor se detiene (cada una
  	   * tiene su propio chequeo de motorOperandoAhora). */
  	  if (motorOperandoAntesParaModoSeguro && !motorOperandoAhora) {
  		  /* En CUALQUIER MODO (2026-10-02, hallazgo B8): SET_RPM/SET_PRESION a
  		   * 0, para que al rearrancar el motor no acelere solo a un valor
  		   * viejo. MODO 4 no pasa a MODO 0 (abajo), y sin esto volvia al
  		   * SET_RPM anterior en cuanto el tacometro lo veia girar. */
  		  CalibFlash_LimpiarComandosManuales();
  		  if (CalibFlash_GetModo() != CALIB_MODO_CALIBRACION) {
  			  CalibFlash_SetModo(CALIB_MODO_RALENTI);
  		  }
  	  }
  	  motorOperandoAntesParaModoSeguro = motorOperandoAhora;

  	  /* Aviso de MODO rechazado (2026-10-01): un downlink/comando MODO que no se
  	   * pudo aplicar (motor detenido, objetivo remoto sin configurar, MODO 2 sin
  	   * venir de MODO 1) fuerza un uplink inmediato con la alerta "modo
  	   * rechazado"; despues de enviarlo se restaura la alerta que habia (ver el
  	   * bloque de envio). El motivo exacto va en el ACK del downlink. */
  	  if (CalibFlash_ConsumirRechazoModo()) {
  		  if (!alertaRechazoActiva) {
  			  alertaAntesDeRechazo = s_codigoAlertaActual;
  		  }
  		  alertaRechazoActiva = true;
  		  s_codigoAlertaActual = ALERTA_MODO_RECHAZADO;
  		  CalibFlash_ForzarReporte();
  	  }

  	  /* ================== Enlace LoRaWAN perdido: degradar MODO ==================
  	   * Agregado 2026-10-01, ver hoja "Maquina de estados" de Documentacion.xlsx.
  	   *
  	   * "Enlace bueno" = RAK3172_EstaUnido() Y menos de ENLACE_FALLAS_COMANDO_MAX
  	   * comandos AT seguidos con resultado != 0. Debe estar malo
  	   * HISTERESIS_MODO_S (fijo, 30 s) SEGUIDOS para declararlo perdido. Solo se actua en el
  	   * FLANCO bueno -> perdido: si nunca hubo enlace desde el arranque (banco
  	   * sin red) no se toca nada -- MODO=3/4 siguen funcionando sin enlace.
  	   *
  	   *   MODO=2 -> MODO=1 (real, no solo "se comporta como"), y se marca
  	   *            recuperable: vuelve solo a MODO=2 cuando el enlace regresa
  	   *            y hay PRESION_REMOTO fresca, salvo que el operador mande
  	   *            cualquier MODO mientras tanto (o el motor se detenga).
  	   *   MODO=3 -> MODO=1 si el motor opera y la presion local (valida) es
  	   *            >= PRESION_OBJETIVO_LOCAL; si no, MODO=0.
  	   *   MODO=4 -> MODO=0 (la prueba CALIB en curso se aborta sola por el
  	   *            chequeo continuo de MODO).
  	   * El watchdog de TIMEOUT_SIN_COMANDO_S (aspersor sin reportar 360 s) de
  	   * MODO=2 vive aqui tambien: misma degradacion 2->1 recuperable, con la
  	   * alerta 4 en vez de 12. Corre ANTES del bloque de MODO, asi que ese
  	   * bloque nunca ve un MODO=2 con la presion remota vencida.
  	   *
  	   *
  	   * Los uplinks NO son confirmados: un AT+SEND en OK no garantiza
  	   * que algun gateway lo oyera, asi que esto detecta modulo caido / join
  	   * perdido / comandos fallando, NO una perdida de cobertura RF silenciosa. */
  	  static bool enlaceVigente = false;
  	  static bool enlaceMalPendiente = false;
  	  static uint32_t enlaceMalDesdeTick = 0U;
  	  static bool remotoRecuperable = false;
  	  static uint32_t remotoRecuperableDownlinks = 0U;
  	  {
  		  bool enlaceBueno = RAK3172_EstaUnido() && fallasComandoConsecutivas < ENLACE_FALLAS_COMANDO_MAX;
  		  bool degradarRemoto = false;
  		  CodigoAlerta_t alertaDegradacion = ALERTA_ENLACE_PERDIDO;
  		  if (enlaceBueno) {
  			  enlaceMalPendiente = false;
  			  if (!enlaceVigente) {
  				  enlaceVigente = true;
  				  printf("ENLACE: LoRaWAN OK\r\n");
  				  if (s_codigoAlertaActual == ALERTA_ENLACE_PERDIDO) {
  					  s_codigoAlertaActual = ALERTA_SIN_ALERTA;
  					  CalibFlash_ForzarReporte();
  				  }
  			  }
  		  } else if (enlaceVigente) {
  			  if (!enlaceMalPendiente) {
  				  enlaceMalPendiente = true;
  				  enlaceMalDesdeTick = HAL_GetTick();
  			  } else if (HAL_GetTick() - enlaceMalDesdeTick >= (uint32_t)CalibFlash_GetHisteresisModoS() * 1000UL) {
  				  enlaceVigente = false;
  				  enlaceMalPendiente = false;
  				  CalibFlash_Modo_t modoAlPerder = CalibFlash_GetModo();
  				  printf("ENLACE: LoRaWAN PERDIDO (MODO=%d)\r\n", (int)modoAlPerder);
  				  /* La decision vive en modo_fsm.c (Tabla B de la hoja). */
  				  Modo_Accion_t accionEnlace = Modo_AccionPerdidaEnlace(modoAlPerder, motorOperandoAhora,
  						  PresionV_SensorValido(), PresionV_GetPresionPsi(), CalibFlash_GetPresionObjetivoLocal());
  				  if (accionEnlace == MODO_ACCION_A_MODO1_RECUPERABLE) {
  					  degradarRemoto = true;
  				  } else if (accionEnlace != MODO_ACCION_NINGUNA) {
  					  CalibFlash_SetModo((accionEnlace == MODO_ACCION_A_MODO1) ? CALIB_MODO_PRESION_LOCAL : CALIB_MODO_RALENTI);
  					  printf("ENLACE: MODO=%d -> MODO=%d (presion local %.1f, objetivo %.1f)\r\n", (int)modoAlPerder,
  							 (accionEnlace == MODO_ACCION_A_MODO1) ? 1 : 0,
  							 PresionV_GetPresionPsi(), CalibFlash_GetPresionObjetivoLocal());
  					  s_codigoAlertaActual = ALERTA_ENLACE_PERDIDO;
  					  CalibFlash_ForzarReporte();
  				  }
  			  }
  		  }

  		  if (!degradarRemoto && CalibFlash_GetModo() == CALIB_MODO_REMOTO) {
  			  uint32_t timeoutMs = CalibFlash_GetTimeoutSinComandoS() * 1000UL;
  			  if (HAL_GetTick() - CalibFlash_GetPresionRemotoUltimoTickMs() >= timeoutMs) {
  				  degradarRemoto = true;
  				  alertaDegradacion = ALERTA_MODO_REMOTO_SIN_PRESION_VALIDA;
  			  }
  		  }
  		  if (degradarRemoto) {
  			  CalibFlash_SetModo(CALIB_MODO_PRESION_LOCAL);
  			  remotoRecuperable = true;
  			  remotoRecuperableDownlinks = CalibFlash_GetModoDownlinkCount();
  			  s_codigoAlertaActual = alertaDegradacion;
  			  CalibFlash_ForzarReporte();
  			  printf("MODO_REMOTO: MODO=2 -> MODO=1 (%s) -- vuelve a MODO=2 solo cuando haya enlace y PRESION_REMOTO fresca\r\n",
  					 (alertaDegradacion == ALERTA_ENLACE_PERDIDO) ? "enlace perdido" : "TIMEOUT_SIN_COMANDO_S");
  		  }

  		  if (remotoRecuperable) {
  			  if (CalibFlash_GetModo() != CALIB_MODO_PRESION_LOCAL || !motorOperandoAhora
  					  || CalibFlash_GetModoDownlinkCount() != remotoRecuperableDownlinks) {
  				  remotoRecuperable = false; /* el operador, el motor o un supervisor ya decidio otra cosa */
  			  } else if (enlaceVigente && CalibFlash_GetPresionObjetivoRemoto() > 0.0f
  					  && (HAL_GetTick() - CalibFlash_GetPresionRemotoUltimoTickMs())
  					     < CalibFlash_GetTimeoutSinComandoS() * 1000UL) {
  				  CalibFlash_SetModo(CALIB_MODO_REMOTO);
  				  remotoRecuperable = false;
  				  printf("MODO_REMOTO: enlace y PRESION_REMOTO validos de nuevo -- MODO=1 -> MODO=2\r\n");
  				  CalibFlash_ForzarReporte();
  			  }
  		  }
  	  }

  	  /* ================== MODO: de donde sale el setpoint de RPM ==================
  	   *   RALENTI (0): sin control activo -- setpoint 0 ("sin comandar"),
  	   *     servo en SERVO_PULSO_MIN.
  	   *   PRESION_LOCAL (1): setpoint = PID#2 (presion_pid.c) contra
  	   *     PRESION_OBJETIVO_LOCAL con el sensor de presion del TID.
  	   *   REMOTO (2): triple cascada -- el PID#3 (presion_pid_remoto.c, solo
  	   *     con reportes NUEVOS del aspersor) ajusta el objetivo local que
  	   *     sigue el PID#2. Ver bloque MODO=2 mas abajo.
  	   *   MANUAL_BANCO (3): setpoint = SET_RPM directo, o la cascada del
  	   *     PID#2 contra SET_PRESION si se mando SET_PRESION.
  	   *   CALIBRACION (4): como MANUAL_BANCO, salvo lo que decida la prueba
  	   *     CALIB en curso (autotunes, auto-calibraciones). */
  	  CalibFlash_Modo_t modoMotor = CalibFlash_GetModo();
  	  float setpointRpmCrudo = 0.0f;

  	  /* presionLocalActivoAhora = "la cascada de presion local (PID#2) corre
  	   * en esta vuelta". Su flanco de entrada reinicia el PID#2 (integral,
  	   * ancla de tiempo) y el ancla de la rampa de llenado. Todos los casos
  	   * que la usan comparten el mismo camino (init, rampa, falla de sensor,
  	   * supervisor):
  	   *   - MODO=1, y MODO=2 (triple cascada: el PID#2 sigue sosteniendo la
  	   *     presion local, el PID#3 solo le mueve el objetivo);
  	   *   - MODO=4 con el autotune PID#3 (CALIB 12) en su fase de cascada;
  	   *   - MODO=4 con el autotune PID#2 (CALIB 9/10/11/14) solo en el escalon
  	   *     de identificacion y la validacion -- la subida es lazo abierto (SET_RPM);
  	   *   - MODO=3 con SET_PRESION != 0 (sin rampa de llenado: es un valor
  	   *     crudo del operador, igual que SET_RPM). */
  	  static bool presionLocalActivoAntes = false;
  	  bool presionLocalActivoAhora = (modoMotor == CALIB_MODO_PRESION_LOCAL)
  			  || (modoMotor == CALIB_MODO_REMOTO)
  			  || (modoMotor == CALIB_MODO_CALIBRACION && CalibFlash_GetCalib() == CALIB_AUTOTUNE_PID3
  			      && AutotuneAsp_CascadaActiva())
  			  || (modoMotor == CALIB_MODO_CALIBRACION && AutotunePsi_EsCalib(CalibFlash_GetCalib())
  			      && AutotunePsi_CascadaActiva())
  			  || (modoMotor == CALIB_MODO_MANUAL_BANCO && CalibFlash_GetSetPresion() != 0.0f);
  	  /* Objetivo vigente para la cascada y el supervisor: SET_PRESION en
  	   * MODO=3, el objetivo de prueba del autotune PID#2 en MODO=4, y si no
  	   * PRESION_OBJETIVO_LOCAL. MODO=2 y el autotune PID#3 lo reemplazan mas
  	   * abajo con su propio objetivo. */
  	  float presionObjetivoVigente = (modoMotor == CALIB_MODO_MANUAL_BANCO)
  			  ? CalibFlash_GetSetPresion()
  			  : ((modoMotor == CALIB_MODO_CALIBRACION && AutotunePsi_EsCalib(CalibFlash_GetCalib()))
  					  ? AutotunePsi_ObjetivoPsi() /* objetivo de prueba de la autosintonia, en RAM -- no toca PRESION_OBJETIVO_LOCAL */
  					  : CalibFlash_GetPresionObjetivoLocal());
  	  /* Ancla de la rampa de llenado -- la presion medida en el instante de
  	   * entrar a la cascada. Si la tuberia ya estaba parcialmente llena, la
  	   * rampa arranca mas arriba y llega antes al objetivo, sola. */
  	  static float presionLlenadoInicioPsi = 0.0f;
  	  static uint32_t presionLlenadoInicioMs = 0U;
  	  if (presionLocalActivoAhora && !presionLocalActivoAntes) {
  		  PresionPid_Init();
  		  presionLlenadoInicioPsi = PresionV_GetPresionPsi();
  		  presionLlenadoInicioMs = HAL_GetTick();
  		  if (s_codigoAlertaActual != ALERTA_ENLACE_PERDIDO) {
  			  s_codigoAlertaActual = ALERTA_SIN_ALERTA; /* arranque fresco, sin arrastrar una alerta vieja */
  		  }
  	  }
  	  presionLocalActivoAntes = presionLocalActivoAhora;

  	  /* MODO=2 (Remoto) -- TRIPLE CASCADA (2026-09-30, decision del usuario,
  	   * reemplaza al PID remoto -> RPM directo):
  	   *
  	   *   PSI aspersor --PID#3--> objetivo local --PID#2--> RPM --PID#1--> servo
  	   *
  	   * La BASE de MODO=2 es PRESION_OBJETIVO_LOCAL que MODO=1 ya dejo
  	   * sostenida (por eso MODO=2 solo se arma viniendo de MODO=1, ver
  	   * calibracion_flash.c). El PID#3 (presion_pid_remoto.c, evento-driven:
  	   * solo con reportes nuevos del aspersor) calcula un AJUSTE en PSI --
  	   * positivo o negativo -- que se suma a esa base para que el aspersor
  	   * llegue a PRESION_OBJETIVO_REMOTO. El objetivo local resultante:
  	   *   - cambia al ritmo de llenado (CalibFlash_GetTasaLlenadoPsiS) en las
  	   *     dos direcciones -- regla de la tuberia del usuario;
  	   *   - se recorta a [0, PRESION_MAX * REMOTO_TECHO_FRAC];
  	   *   - vive solo en RAM: PRESION_OBJETIVO_LOCAL nunca se modifica.
  	   * La cascada local (PID#2, rampa, falla de sensor) es la MISMA de MODO=1
  	   * (presionLocalActivoAhora incluye MODO=2), y como MODO=2 viene de
  	   * MODO=1 no hay flanco: el PID#2 sigue sin re-inicializarse, sin salto.
  	   *
  	   * Sin reportes del aspersor en TIMEOUT_SIN_COMANDO_S, el bloque de
  	   * enlace (arriba) ya paso MODO a 1 de verdad, recuperable: vuelve solo
  	   * a MODO=2 cuando regresan los reportes. */
  	  static bool remotoActivoAntes = false;
  	  bool remotoActivoAhora = (modoMotor == CALIB_MODO_REMOTO);
  	  static uint32_t remotoPidUltimoTickVisto = 0U;
  	  static float remotoAjustePsi = 0.0f;
  	  float remotoTechoPsi = CalibFlash_GetPresionMax() * REMOTO_TECHO_FRAC;
  	  if (remotoActivoAhora && !remotoActivoAntes) {
  		  PresionPidRemoto_Init();
  		  remotoAjustePsi = 0.0f;
  		  /* El reporte que ya se conocia NO se vuelve a usar: el primer
  		   * ajuste sale del proximo reporte nuevo (mientras tanto sostiene la
  		   * base, exactamente lo que ya hacia MODO=1). */
  		  remotoPidUltimoTickVisto = CalibFlash_GetPresionRemotoUltimoTickMs();
  		  PresionPidRemoto_ReiniciarRampa(PresionV_GetPresionPsi());
  		  s_codigoAlertaActual = ALERTA_SIN_ALERTA; /* arranque fresco -- ademas es la
  		                                              * unica forma de limpiar la alerta
  		                                              * final de fuga (MODO ya cayo a
  		                                              * RALENTI cuando esa alerta se
  		                                              * disparo) */
  	  }
  	  remotoActivoAntes = remotoActivoAhora;

  	  if (remotoActivoAhora) {
  		  float baseLocal = CalibFlash_GetPresionObjetivoLocal();
  		  uint32_t tickPresionActual = CalibFlash_GetPresionRemotoUltimoTickMs();
  		  if (tickPresionActual != remotoPidUltimoTickVisto) {
  			  remotoPidUltimoTickVisto = tickPresionActual;
  			  remotoAjustePsi = PresionPidRemoto_CalcularAjustePsi(
  					  CalibFlash_GetPresionObjetivoRemoto(), CalibFlash_GetPresionRemoto(),
  					  baseLocal, remotoTechoPsi);
  			  printf("ASP_TEST,%lu,%.2f,%.2f,%.2f,%.2f\r\n",
  				  (unsigned long)HAL_GetTick(), CalibFlash_GetPresionRemoto(),
  				  CalibFlash_GetPresionObjetivoRemoto(), remotoAjustePsi, PresionV_GetPresionPsi());
  		  }
  		  presionObjetivoVigente = PresionPidRemoto_ObjetivoAplicado(baseLocal + remotoAjustePsi,
  				  CalibFlash_GetTasaLlenadoPsiS());
  	  }

  	  /* Autosintonia del PID#3 (CALIB=12): su propio objetivo local de prueba
  	   * (RAM), sea que haya arrancado desde MODO=4 o desde MODO=1. */
  	  if (CalibFlash_GetCalib() == CALIB_AUTOTUNE_PID3 && AutotuneAsp_CascadaActiva()) {
  		  presionObjetivoVigente = AutotuneAsp_ObjetivoLocal();
  	  }

  	  /* Si el sensor local esta en falla, no tiene sentido cerrar el lazo
  	   * contra una lectura invalida -- se cae a "sin comandar" (ralenti
  	   * natural) en vez de sostener ciegamente el ultimo setpoint. */
  	  static bool presionLocalSensorFallaAntes = false;
  	  static uint32_t sensorFallaDesdeTick = 0U; /* ver guarda de falla prolongada abajo; se resetea tambien fuera de la cascada */
  	  bool presionLocalSensorFallaAhora = false;

  	  if (presionLocalActivoAhora) {
  		  presionLocalSensorFallaAhora = !PresionV_SensorValido();
  		  if (presionLocalSensorFallaAhora) {
  			  setpointRpmCrudo = 0.0f; /* sin comandar -- cae a ralenti natural mas abajo */
  		  } else {
  			  /* Rampa de llenado: en vez de mandarle el objetivo directo al
  			   * PID#2, se le manda uno que sube desde presionLlenadoInicioPsi a
  			   * CalibFlash_GetTasaLlenadoPsiS() (PRESION_OBJETIVO_LOCAL /
  			   * TIEMPO_LLENADO_S). El "if (rampaObjetivo < objetivoFinal)"
  			   * recorta solo al llegar, asi que no hace falta una fase aparte.
  			   * TIEMPO_LLENADO_S = 0 -> tasa 0 -> sin rampa.
  			   * Sin rampa tambien: MODO=3 (SET_PRESION es un valor crudo del
  			   * operador) y el autotune PID#2 (su escalon tiene que ser limpio
  			   * para que la identificacion y la validacion sirvan). */
  			  float objetivoFinal = presionObjetivoVigente;
  			  float objetivoDelInstante = objetivoFinal;
  			  bool esAutotunePsi = (modoMotor == CALIB_MODO_CALIBRACION
  					  && AutotunePsi_EsCalib(CalibFlash_GetCalib()));
  			  /* MODO=2 tampoco pasa por esta rampa: su objetivo local ya llega
  			   * limitado al mismo ritmo por PresionPidRemoto_ObjetivoAplicado()
  			   * (ver bloque MODO=2 mas arriba). */
  			  if (modoMotor != CALIB_MODO_MANUAL_BANCO && modoMotor != CALIB_MODO_REMOTO
  			      && !esAutotunePsi) {
  				  float tasaLlenado = CalibFlash_GetTasaLlenadoPsiS();
  				  if (tasaLlenado > 0.0f) {
  					  float segundosTranscurridos = (HAL_GetTick() - presionLlenadoInicioMs) / 1000.0f;
  					  float rampaObjetivo = presionLlenadoInicioPsi + tasaLlenado * segundosTranscurridos;
  					  if (rampaObjetivo < objetivoFinal) {
  						  objetivoDelInstante = rampaObjetivo;
  					  }
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

  		  /* Falla PROLONGADA del sensor (agregado 2026-10-01): forzar ralenti
  		   * arriba solo baja el setpoint de RPM pero MODO se quedaba en 1/2/3,
  		   * asi que al volver el sensor el control retomaba solo. Pasados
  		   * SENSOR_PRESION_FALLA_A_MODO0_MS seguidos en falla, MODO pasa a 0 de
  		   * verdad (operador debe reconfirmar). Solo MODO 1/2/3: en MODO=4 las
  		   * pruebas CALIB ya tienen su propio chequeo del sensor. La alerta 3
  		   * queda puesta hasta la proxima entrada a un modo con control. */
  		  if (presionLocalSensorFallaAhora && modoMotor != CALIB_MODO_CALIBRACION) {
  			  if (sensorFallaDesdeTick == 0U) {
  				  sensorFallaDesdeTick = HAL_GetTick() | 1U; /* nunca 0, 0 = "no en falla" */
  			  } else if (HAL_GetTick() - sensorFallaDesdeTick >= SENSOR_PRESION_FALLA_A_MODO0_MS) {
  				  CalibFlash_SetModo(CALIB_MODO_RALENTI);
  				  sensorFallaDesdeTick = 0U;
  				  s_codigoAlertaActual = ALERTA_SENSOR_PRESION_LOCAL_FALLA;
  				  printf("MODO_PRESION_LOCAL: sensor de presion en falla por mas de %lus -- MODO=%d -> MODO=0\r\n",
  						 (unsigned long)(SENSOR_PRESION_FALLA_A_MODO0_MS / 1000UL), (int)modoMotor);
  				  CalibFlash_ForzarReporte();
  			  }
  		  } else {
  			  sensorFallaDesdeTick = 0U;
  		  }
  	  } else {
  		  presionLocalSensorFallaAhora = false; /* fuera de PRESION_LOCAL, no aplica */
  		  sensorFallaDesdeTick = 0U;
  	  }
  	  presionLocalSensorFallaAntes = presionLocalSensorFallaAhora;

  	  /* Supervisor de "no se puede llegar al objetivo de presion" (ver
  	   * PRESION_SUPERVISOR_* arriba y README seccion 8). Solo detecta, no
  	   * protege (el recorte de RPM ya lo hace presion_pid.c): si la cascada
  	   * queda pidiendo RPM_MAX_CARGA y la presion sigue lejos del objetivo,
  	   * la meta no es alcanzable (o algo anda mal con el sensor o la
  	   * hidraulica) y vale la pena avisar. Se ignora con el sensor en falla
  	   * (esa condicion tiene su propia alerta). */
  	  static uint32_t presionSupervisorInicioVentanaMs = 0U;
  	  static bool     presionSupervisorEnVentana = false;
  	  static uint32_t presionSupervisorIntentosFallidos = 0U;

  	  /* En MODO=2 no corre este supervisor: la presion local sigue un objetivo
  	   * que mueve el PID#3, y el que vigila "no se llega" es el supervisor de
  	   * fuga de MODO=2 (mas abajo), contra el objetivo del ASPERSOR. */
  	  if (presionLocalActivoAhora && !presionLocalSensorFallaAhora && modoMotor != CALIB_MODO_REMOTO) {
  		  float presionObjetivoActual = presionObjetivoVigente;
  		  float presionMedidaActual = PresionV_GetPresionPsi();
  		  bool saturadoSinAlcanzar = (setpointRpmCrudo >= rpmMaxCarga)
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
  				  printf("ALERTA,PRESION_OBJETIVO_NO_ALCANZADA,intentos=%lu,presion_objetivo=%.2f,presion_actual=%.2f,rpm_max_carga=%.1f\r\n",
  					  (unsigned long)presionSupervisorIntentosFallidos, presionObjetivoActual,
  					  presionMedidaActual, rpmMaxCarga);
  				  s_codigoAlertaActual = ALERTA_PRESION_OBJETIVO_NO_ALCANZADA;
  				  CalibFlash_ForzarReporte();
  			  }
  		  }
  	  } else {
  		  presionSupervisorEnVentana = false;
  		  presionSupervisorIntentosFallidos = 0U;
  	  }

  	  if ((modoMotor == CALIB_MODO_MANUAL_BANCO || modoMotor == CALIB_MODO_CALIBRACION)
  			  && !presionLocalActivoAhora) {
  		  /* MODO=3 y MODO=4: SET_RPM directo, salvo que la cascada de presion
  		   * ya este corriendo (SET_PRESION o un autotune de presion) -- el
  		   * "!presionLocalActivoAhora" evita pisar su setpoint (bug real
  		   * 2026-09-25). MODO=3 y 4 se comportan igual aqui; la diferencia es
  		   * de candados: solo MODO=4 acepta pruebas CALIB. MODO=2 no tiene
  		   * rama propia: su setpoint sale de la cascada de arriba. */
  		  setpointRpmCrudo = CalibFlash_GetSetRpm();
  	  }

  	  /* Supervisor de fuga de MODO=2 (ver PRESION_SUPERVISOR_REMOTO_* arriba):
  	   * 1) Cuenta REPORTES NUEVOS del aspersor, no segundos: su cadencia
  	   *    varia mucho (~20-25 min con presion en 0, ~20 s en rampa, ~90 s
  	   *    estable).
  	   * 2) "Falla" = no puede dar mas (RPM_MAX_CARGA, o el PID#3 ya pide el
  	   *    techo de presion local) Y, si PRESION_OBJETIVO_REMOTO esta
  	   *    configurado, la presion remota sigue mas de 5 % por debajo.
  	   * 3) Cada falla retrocede a "sin comandar" durante
  	   *    PRESION_SUPERVISOR_REMOTO_RETROCESO_MS (sin cambiar MODO); al
  	   *    llegar a PRESION_SUPERVISOR_REMOTO_INTENTOS_MAX manda la alerta 2
  	   *    y pasa a MODO=0. */
  	  static uint32_t remotoSupervisorUltimoTickVisto = 0U;
  	  static uint32_t remotoSupervisorIntentosFallidos = 0U;
  	  static bool     remotoEnRetroceso = false;
  	  static uint32_t remotoRetrocesoInicioMs = 0U;

  	  if (modoMotor == CALIB_MODO_REMOTO) {

  		  if (remotoEnRetroceso) {
  			  setpointRpmCrudo = 0.0f; /* sin comandar -- cae a ralenti natural mas abajo, mismo mecanismo que MODO=0 */
  			  if (HAL_GetTick() - remotoRetrocesoInicioMs >= PRESION_SUPERVISOR_REMOTO_RETROCESO_MS) {
  				  remotoEnRetroceso = false;
  				  /* Triple cascada (2026-09-30): al retomar, el PID#2 arranca
  				   * limpio (su integral pudo cargarse mientras el motor estaba
  				   * forzado a ralenti) y el objetivo local vuelve a subir DESDE
  				   * la presion real, al ritmo de llenado -- nunca de un salto. */
  				  PresionPid_Init();
  				  PresionPidRemoto_ReiniciarRampa(PresionV_GetPresionPsi());
  			  }
  		  } else {
  			  uint32_t tickPresionActual = CalibFlash_GetPresionRemotoUltimoTickMs();
  			  bool llegoReporteNuevo = (tickPresionActual != remotoSupervisorUltimoTickVisto);

  			  if (llegoReporteNuevo) {
  				  remotoSupervisorUltimoTickVisto = tickPresionActual;

  				  float objetivoRemoto = CalibFlash_GetPresionObjetivoRemoto();
  				  float presionRemotaActual = CalibFlash_GetPresionRemoto();
  				  bool  objetivoConfigurado = objetivoRemoto > 0.0f;
  				  /* "No puede dar mas" en triple cascada: o el motor ya esta en
  				   * RPM_MAX_CARGA, o el PID#3 ya pide el techo de presion local. */
  				  bool  saturadoEnMax = (setpointRpmCrudo >= rpmMaxCarga)
  						  || (CalibFlash_GetPresionObjetivoLocal() + remotoAjustePsi >= remotoTechoPsi - 0.01f);
  				  bool  lejosDelObjetivo = objetivoConfigurado
  						  && (presionRemotaActual < objetivoRemoto * (1.0f - PRESION_SUPERVISOR_TOLERANCIA_FRACCION));
  				  bool  alertarEsteReporte = objetivoConfigurado
  						  ? (saturadoEnMax && lejosDelObjetivo) : saturadoEnMax;

  				  if (!alertarEsteReporte) {
  					  remotoSupervisorIntentosFallidos = 0U;
  				  } else {
  					  remotoSupervisorIntentosFallidos++;
  					  if (remotoSupervisorIntentosFallidos >= PRESION_SUPERVISOR_REMOTO_INTENTOS_MAX) {
  						  printf("ALERTA,MODO_REMOTO_OBJETIVO_NO_ALCANZADO,intentos=%lu,presion_remota=%.1f,objetivo_remoto=%.1f,presion_local=%.2f,rpm_max_carga=%.1f\r\n",
  							  (unsigned long)remotoSupervisorIntentosFallidos, presionRemotaActual,
  							  objetivoRemoto, PresionV_GetPresionPsi(), rpmMaxCarga);
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

  	  /* Log PRESION_PID_TEST del lazo de presion (PID#2), mismo formato de
  	   * columnas que PID_TEST para reusar tools/pid_tuning/pid_tuning.py.
  	   * Solo durante el autotune PID#2 (CALIB 9/10/11/14) y con la cascada
  	   * corriendo de verdad. Lleva la presion y el RPM resultante para ver
  	   * los dos lazos en la misma linea. */
  	  static uint32_t ultimoLogPruebaPresion = 0;
  	  if (AutotunePsi_EsCalib(CalibFlash_GetCalib())
  	      && presionLocalActivoAhora
  	      && HAL_GetTick() - ultimoLogPruebaPresion >= 200U) {
  		  ultimoLogPruebaPresion = HAL_GetTick();
  		  printf("PRESION_PID_TEST,%lu,%.2f,%.2f,%.1f,%.1f\r\n",
  			  (unsigned long)HAL_GetTick(),
  			  presionObjetivoVigente,
  			  PresionV_GetPresionPsi(),
  			  setpointRpmCrudo,
  			  Tacometro_GetRPMFiltrada());
  	  }

  	  /* Autosintonia del lazo interno (PID#1) -- CALIB_AUTOTUNE_PID1 (=13)
  	   * completa, o sus etapas sueltas CALIB 5/6/7 (curva / identificacion /
  	   * validacion, sin escribir flash), ver autotune_rpm.h. El completo
  	   * identifica, calcula SIMC, valida en el motor real y solo si pasa
  	   * guarda Kp/Ki. La maquina de estados vive en autotune_rpm.c; aca solo se cablea:
  	   * flanco de entrada, paso por vuelta, y flanco de salida (que
  	   * limpia las ganancias temporales por CUALQUIER motivo de salida,
  	   * incluido cancelar por downlink). El resultado se publica como
  	   * codigo de alerta del uplink (ver ALERTA_AUTOTUNE_*) para verlo
  	   * sin laptop. Variable de flanco PROPIA, mismo criterio que las de
  	   * los bloques vecinos. */
  	  static uint8_t autotunePid1Anterior = 0U;
  	  static uint16_t autotunePid1PulsoDirectoUs = 0U; /* destino del servo en la fase de barrido, ver AutotuneRpm_ServoDirecto() */
  	  uint8_t calibPid1 = CalibFlash_GetCalib();
  	  if (AutotuneRpm_EsCalib(autotunePid1Anterior) && calibPid1 != autotunePid1Anterior) {
  		  AutotuneRpm_Limpiar();
  		  Autotune_Resultado_t resultadoPid1 = AutotuneRpm_UltimoResultado();
  		  if (resultadoPid1 != AUTOTUNE_SIN_RESULTADO) {
  			  s_codigoAlertaActual = (resultadoPid1 == AUTOTUNE_RESULTADO_OK)
  					  ? ALERTA_AUTOTUNE_PID1_OK : ALERTA_AUTOTUNE_PID1_FALLO;
  			  CalibFlash_ForzarReporte();
  		  }
  	  }
  	  if (AutotuneRpm_EsCalib(calibPid1) && calibPid1 != autotunePid1Anterior) {
  		  AutotuneRpm_Iniciar(calibPid1);
  		  s_codigoAlertaActual = ALERTA_SIN_ALERTA;
  	  }
  	  if (AutotuneRpm_EsCalib(calibPid1)) {
  		  AutotuneRpm_Paso(motorOperandoAhora, modoMotor == CALIB_MODO_CALIBRACION);
  	  }
  	  /* Si Paso() termino y dejo CALIB=0, la salida se detecta en la proxima vuelta. */
  	  autotunePid1Anterior = calibPid1;
  	  if (modoMotor != CALIB_MODO_CALIBRACION
  	      && (s_codigoAlertaActual == ALERTA_AUTOTUNE_PID1_OK || s_codigoAlertaActual == ALERTA_AUTOTUNE_PID1_FALLO)) {
  		  s_codigoAlertaActual = ALERTA_SIN_ALERTA; /* salio de la sesion de calibracion: el aviso ya cumplio */
  	  }

  	  /* Autosintonia del lazo de presion local (PID#2) -- CALIB_AUTOTUNE_PID2
  	   * (=14), ver autotune_psi.c. ⚠️ NO PROBADA CON MOTOBOMBA REAL. Mismo
  	   * cableado que PID#1 de arriba. Las fases en lazo abierto (base,
  	   * curva, asentar, identificacion) mandan SET_RPM directo -- cae en la
  	   * rama de MODO=CALIBRACION de 'setpointRpmCrudo'; solo la fase de
  	   * validacion enciende la cascada real (ver presionLocalActivoAhora,
  	   * AutotunePsi_CascadaActiva). */
  	  static uint8_t autotunePid2Anterior = 0U;
  	  uint8_t calibPid2 = CalibFlash_GetCalib();
  	  if (AutotunePsi_EsCalib(autotunePid2Anterior) && calibPid2 != autotunePid2Anterior) {
  		  AutotunePsi_Limpiar();
  		  Autotune_Resultado_t resultadoPid2 = AutotunePsi_UltimoResultado();
  		  if (resultadoPid2 != AUTOTUNE_SIN_RESULTADO) {
  			  s_codigoAlertaActual = (resultadoPid2 == AUTOTUNE_RESULTADO_OK)
  					  ? ALERTA_AUTOTUNE_PID2_OK : ALERTA_AUTOTUNE_PID2_FALLO;
  			  CalibFlash_ForzarReporte();
  		  }
  	  }
  	  if (AutotunePsi_EsCalib(calibPid2) && calibPid2 != autotunePid2Anterior) {
  		  AutotunePsi_Iniciar(calibPid2);
  		  s_codigoAlertaActual = ALERTA_SIN_ALERTA;
  	  }
  	  if (AutotunePsi_EsCalib(calibPid2)) {
  		  AutotunePsi_Paso(motorOperandoAhora, modoMotor == CALIB_MODO_CALIBRACION);
  	  }
  	  /* Si Paso() termino y dejo CALIB=0, la salida se detecta en la proxima vuelta. */
  	  autotunePid2Anterior = calibPid2;
  	  if (modoMotor != CALIB_MODO_CALIBRACION
  	      && (s_codigoAlertaActual == ALERTA_AUTOTUNE_PID2_OK || s_codigoAlertaActual == ALERTA_AUTOTUNE_PID2_FALLO)) {
  		  s_codigoAlertaActual = ALERTA_SIN_ALERTA;
  	  }

  	  /* Autosintonia del PID#3 (MODO=2, triple cascada) -- CALIB_AUTOTUNE_PID3
  	   * (=12), ver autotune_asp.c. ⚠️ NO PROBADA CON MOTOBOMBA NI ASPERSOR.
  	   * Reemplaza (2026-09-30) al viejo auto-escalon remoto de CALIB=12 (escalon
  	   * de SET_RPM desde RPM_MIN esperando 15 reportes, sin terminar) y a su log
  	   * REMOTO_PID_TEST. Unico CALIB que tambien arranca desde MODO=1 (re-ajuste
  	   * rapido tras mover el aspersor); desde MODO=4 hace ademas el llenado.
  	   * Mismo cableado que PID#1/PID#2 (flanco de entrada, paso, flanco de
  	   * salida con el resultado como codigo de alerta del uplink). */
  	  static uint8_t autotunePid3Anterior = 0U;
  	  if (autotunePid3Anterior != CALIB_AUTOTUNE_PID3 && CalibFlash_GetCalib() == CALIB_AUTOTUNE_PID3) {
  		  AutotuneAsp_Iniciar();
  		  s_codigoAlertaActual = ALERTA_SIN_ALERTA;
  	  }
  	  if (CalibFlash_GetCalib() == CALIB_AUTOTUNE_PID3) {
  		  AutotuneAsp_Paso(motorOperandoAhora, modoMotor);
  	  }
  	  if (autotunePid3Anterior == CALIB_AUTOTUNE_PID3 && CalibFlash_GetCalib() != CALIB_AUTOTUNE_PID3) {
  		  AutotuneAsp_Limpiar();
  		  Autotune_Resultado_t resultadoPid3 = AutotuneAsp_UltimoResultado();
  		  if (resultadoPid3 != AUTOTUNE_SIN_RESULTADO) {
  			  s_codigoAlertaActual = (resultadoPid3 == AUTOTUNE_RESULTADO_OK)
  					  ? ALERTA_AUTOTUNE_PID3_OK : ALERTA_AUTOTUNE_PID3_FALLO;
  			  CalibFlash_ForzarReporte();
  		  }
  	  }
  	  autotunePid3Anterior = CalibFlash_GetCalib();
  	  if ((modoMotor != CALIB_MODO_CALIBRACION && modoMotor != CALIB_MODO_PRESION_LOCAL)
  	      && (s_codigoAlertaActual == ALERTA_AUTOTUNE_PID3_OK || s_codigoAlertaActual == ALERTA_AUTOTUNE_PID3_FALLO)) {
  		  s_codigoAlertaActual = ALERTA_SIN_ALERTA; /* salio de la sesion (MODO 4, o MODO 1 si arranco desde ahi) */
  	  }


  	  uint8_t modoControl = CalibFlash_GetCalib(); /* valores de CALIB: ver calibracion_flash.h / hoja CALIB */

  	  /* Auto-calibracion de ralenti -- CALIB=8 (solo se pide viniendo de
  	   * CALIB=0, con o sin el motor operando).
  	   * Mientras se esta en CALIB=8:
  	   *   - si el motor no esta operando, se espera -- el servo queda en
  	   *     SERVO_PULSO_MIN (en CALIB=8 no hay control activo).
  	   *   - apenas el motor arranca (o si ya estaba andando al entrar a
  	   *     este modo), arranca la ventana de medicion.
  	   *   - si el motor se detiene a mitad de la ventana, se aborta la
  	   *     medicion en curso y se vuelve a esperar (sin salir de modo
  	   *     8) -- no tiene sentido promediar con el motor parado, pero
  	   *     tampoco hace falta reenviar el downlink si se puede
  	   *     reintentar solo arrancando el motor de nuevo.
  	   *   - al completar los 2 minutos, se aplica RPM_MIN y se vuelve
  	   *     sola a modo 0 (mismo criterio que el apagado de seguridad de
  	   *     1/2 en este mismo loop, que tambien fuerza CALIB
  	   *     localmente sin esperar un downlink). */
  	  {
  		  static bool     ralentiCalEsperandoMotor = false;
  		  static bool     ralentiCalMidiendo = false;
  		  static uint32_t ralentiCalInicioMs = 0;
  		  static uint32_t ralentiCalUltimoLogMs = 0;
  		  static float    ralentiCalSuma = 0.0f;
  		  static uint32_t ralentiCalMuestras = 0;

  		  if (modoControl == 8U) {
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
  						  CalibFlash_SetCalib(0U); /* vuelve sola a operacion normal */
  					  }
  				  }
  			  }
  		  } else {
  			  ralentiCalEsperandoMotor = false;
  			  ralentiCalMidiendo = false;
  		  }
  	  }

  	  /* Auto-calibracion de ALPHA -- CALIB=3 (ver
  	   * calibracion_flash.c: solo se puede pedir viniendo de modo 0, con
  	   * o sin el motor operando). Mismo patron que el bloque de CALIB=8
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
  						  CalibFlash_SetCalib(0U); /* vuelve sola a operacion normal */
  					  }
  				  }
  			  }
  		  } else {
  			  alphaCalEsperandoMotor = false;
  			  alphaCalMidiendo = false;
  		  }
  	  }

  	  /* modoConCarga = la bomba esta embragada: MODO=1/2/3, y en MODO=4 los
  	   * autotunes de presion (CALIB 12 y 9/10/11/14). El resto de MODO=4 son
  	   * rutinas de banco con la bomba DESEMBRAGADA. Decide la guarda de
  	   * PRESION_MAX, la rampa de bajada y el techo de RPM (RPM_MAX_CARGA vs
  	   * RPM_MAX).
  	   *
  	   * Guarda PRESION_MAX -- analoga a RPM_MAX_CARGA, NO a RPM_MAX: es un
  	   * limite de la INSTALACION (tuberia), asi que aplica solo con la bomba
  	   * cargada. MODO=0 (RALENTI) y las rutinas de banco sin carga quedan
  	   * exentas. Si la presion local medida supera PRESION_MAX se fuerza
  	   * el setpoint a "sin comandar" (ralenti) -- corre DESPUES de todos
  	   * los bloques que calculan setpointRpmCrudo, asi cubre cascada,
  	   * SET_RPM manual y los auto-escalon. Histeresis: se libera al bajar
  	   * a 90% de PRESION_MAX, para no oscilar en el umbral. Con sensor
  	   * en falla no decide nada aca (ya cae a ralenti por su propia
  	   * alerta en MODO=1). */
  	  static bool presionMaxExcedidaAntes = false;
  	  bool modoConCarga = (modoMotor == CALIB_MODO_PRESION_LOCAL)
  			  || (modoMotor == CALIB_MODO_REMOTO)
  			  || (modoMotor == CALIB_MODO_MANUAL_BANCO)
  			  || (modoMotor == CALIB_MODO_CALIBRACION
  				  && (CalibFlash_GetCalib() == 12U
  					  || AutotunePsi_EsCalib(CalibFlash_GetCalib())));
  	  bool presionMaxExcedidaAhora = false;
  	  if (modoConCarga && PresionV_SensorValido()) {
  		  float presionMax = CalibFlash_GetPresionMax();
  		  float presionMedida = PresionV_GetPresionPsi();
  		  presionMaxExcedidaAhora = presionMaxExcedidaAntes
  				  ? (presionMedida > presionMax * 0.9f)
  				  : (presionMedida > presionMax);
  	  }
  	  if (presionMaxExcedidaAhora) {
  		  setpointRpmCrudo = 0.0f;
  		  controlSolicitado = false;
  	  }
  	  if (presionMaxExcedidaAhora != presionMaxExcedidaAntes) {
  		  if (presionMaxExcedidaAhora) {
  			  printf("ALERTA,PRESION_MAX_EXCEDIDA,presion_max=%.2f,presion_actual=%.2f -- forzando ralenti\r\n",
  					  CalibFlash_GetPresionMax(), PresionV_GetPresionPsi());
  			  s_codigoAlertaActual = ALERTA_PRESION_MAX_EXCEDIDA;
  		  } else {
  			  printf("ALERTA,PRESION_MAX_RECUPERADA\r\n");
  			  if (s_codigoAlertaActual == ALERTA_PRESION_MAX_EXCEDIDA) {
  				  s_codigoAlertaActual = ALERTA_SIN_ALERTA;
  			  }
  		  }
  		  CalibFlash_ForzarReporte();
  	  }
  	  presionMaxExcedidaAntes = presionMaxExcedidaAhora;

  	  /* Rampa de BAJADA (2026-10-01, decision del usuario): una caida brusca
  	   * de presion con la bomba cargada puede reventar la tuberia (golpe de
  	   * ariete), igual que una subida brusca. Cuando se RETIRA el control
  	   * (MODO->0 por cualquier causa, retroceso del supervisor de MODO=2,
  	   * fin/aborto de un autotune de presion, SET_RPM=0) el RPM baja hacia
  	   * RPM_MIN a CalibFlash_GetTasaMaxCambioRpmLlenadoS() (fijo, 10 RPM/s)
  	   * en vez de soltar el servo de golpe; el PID#1 sigue activo mientras baja.
  	   *   - Solo si la bomba venia cargada (modoConCarga, recordado aunque
  	   *     MODO ya sea 0): sin carga (autotune PID#1) no hace falta y su
  	   *     vuelta a la base tiene timeout corto.
  	   *   - NO limita mientras hay control: frenar la salida del PID#2 le
  	   *     cambiaria la dinamica. Si el control vuelve a mitad de la bajada,
  	   *     se sigue bajando hasta encontrar su setpoint.
  	   *   - Excepciones inmediatas: PRESION_MAX excedida (cortar rapido es la
  	   *     proteccion) y motor detenido. Un reinicio no se puede rampear. */
  	  static float    bajadaRpm = 0.0f;
  	  static bool     bajadaEnCurso = false;
  	  static bool     bajadaConCarga = false;
  	  static uint32_t bajadaTickMs = 0U;
  	  {
  		  uint32_t ahoraMs = HAL_GetTick();
  		  float dtS = (float)(ahoraMs - bajadaTickMs) / 1000.0f;
  		  bajadaTickMs = ahoraMs;
  		  float tasa = CalibFlash_GetTasaMaxCambioRpmLlenadoS();
  		  float destino = controlSolicitado ? setpointRpmCrudo : rpmMin;
  		  if (modoConCarga) bajadaConCarga = true;

  		  if (!motorOperandoAhora || presionMaxExcedidaAhora || tasa <= 0.0f) {
  			  bajadaEnCurso = false;
  			  bajadaConCarga = modoConCarga && motorOperandoAhora;
  		  } else if (!controlSolicitado && bajadaConCarga && bajadaRpm > rpmMin
  		             && (!presionLocalActivoAhora || setpointRpmCrudo <= 0.0f)) {
  			  /* "Retirar el control" = la cascada ya no corre, o el setpoint se
  			   * forzo a 0 (retroceso/watchdog de MODO=2, sensor en falla). Si la
  			   * cascada sigue y su propia salida baja a RPM_MIN, no se toca. */
  			  if (!bajadaEnCurso) {
  				  printf("RAMPA_BAJADA,INICIO,desde_rpm=%.1f,tasa_rpm_s=%.1f\r\n", bajadaRpm, tasa);
  			  }
  			  bajadaEnCurso = true;
  		  }

  		  if (bajadaEnCurso) {
  			  float siguiente = bajadaRpm - tasa * dtS;
  			  if (siguiente <= destino) {
  				  bajadaEnCurso = false;
  				  bajadaRpm = destino;
  				  if (!modoConCarga) bajadaConCarga = false;
  				  printf("RAMPA_BAJADA,FIN,rpm=%.1f\r\n", destino);
  			  } else {
  				  bajadaRpm = siguiente;
  				  setpointRpmCrudo = siguiente;
  				  controlSolicitado = siguiente > rpmMin;
  			  }
  		  } else {
  			  bajadaRpm = destino;
  			  if (!modoConCarga && !controlSolicitado) bajadaConCarga = false;
  		  }
  	  }

  	  static bool pidActivoAntes = false;
  	  /* El PID#1 (RPM -> servo) corre con CALIB=0 y durante los autotunes
  	   * (PID#1: 5/6/7/13, PID#2: 9/10/11/14, PID#3: 12), que necesitan el lazo
  	   * interno vivo para que sus escalones de SET_RPM o de presion muevan
  	   * el servo. CALIB 1/2/4 y las fases en lazo abierto de los autotunes
  	   * manejan el servo directo (ver if/else de abajo); CALIB 3/8 lo dejan
  	   * en SERVO_PULSO_MIN. */
  	  bool pidActivoAhora = (modoControl == 0U || AutotuneRpm_EsCalib(modoControl)
  	          || modoControl == 12U || AutotunePsi_EsCalib(modoControl))
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
  		  /* Modo manual de calibración (CALIB=1): a
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
  				  CalibFlash_SetCalib(0U);
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
  					  CalibFlash_SetCalib(0U);
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
  					  CalibFlash_SetCalib(0U);
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
  	  } else if (AutotuneRpm_EsCalib(modoControl) && motorOperandoAhora
  	             && AutotuneRpm_ServoDirecto(&autotunePid1PulsoDirectoUs)) {
  		  /* Fases en lazo abierto de la autosintonia PID#1 (curva y escalon
  		   * de identificacion, CALIB 5/6/13): el pulso lo decide
  		   * autotune_rpm.c. El resto de las fases caen al
  		   * PID normal (pidActivoAhora) o al 'sin control activo' de abajo. */
  		  Servo_MoverHacia(autotunePid1PulsoDirectoUs);
  	  } else if (pidActivoAhora) {
  		  /* Lazo de control real: motor operando y un setpoint por encima del
  		   * ralenti (el piso ya lo garantiza controlSolicitado). El techo es
  		   * RPM_MAX_CARGA con la bomba embragada (modoConCarga) y RPM_MAX con
  		   * la bomba desembragada (rutinas de banco de MODO=4). */
  		  float rpmMaxAplicable = modoConCarga ? rpmMaxCarga : rpmMax;
  		  float setpointRpm = setpointRpmCrudo;
  		  if (setpointRpm > rpmMaxAplicable) setpointRpm = rpmMaxAplicable;

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
