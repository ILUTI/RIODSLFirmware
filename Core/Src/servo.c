/**
 * @file    servo.c
 * @brief   Implementación del control del servo vía PWM.
 */

#include "servo.h"
#include "calibracion_flash.h"

/* Sentido de giro del montaje mecanico actual (fijo, no configurable
 * por downlink -- es una propiedad de como quedo instalado el servo,
 * no algo que un operador deba tocar en campo).
 *
 * Confirmado en banco 2026-09-07, con el servo montado directo sobre
 * el eje de la palanca del acelerador (branch servo-directo), viendo
 * el servo DESDE ATRAS (la vista real y consistente usada para
 * confirmar esto -- OJO: el sentido percibido se invierte si se mira
 * desde el lado opuesto, eso genero confusion en el primer intento de
 * validar este valor): subir el pulso logico (SERVO_PULSO_MIN hacia
 * SERVO_PULSO_MAX) YA gira en el sentido correcto sin necesidad de
 * invertir nada -- por eso queda en 0. Se probo con 1 (invertido) y
 * dio el sentido equivocado, asi que 0 es el valor correcto para este
 * montaje. Si se vuelve a remontar el servo (otra orientacion, u otro
 * mecanismo), reverificar este valor en banco antes de asumir que
 * sigue siendo el correcto.
 *
 * En 1, se invierte el pulso fisico que efectivamente recibe el PWM,
 * como si fuera el interruptor de "reverse" de un servo de RC -- se
 * espeja sobre el rango FIJO de fabrica del servo (500-2500us), NO sobre
 * SERVO_PULSO_MIN/MAX. A proposito: esos dos son justamente los
 * valores que se estan calibrando a mano en CALIB=2 en el
 * mecanismo nuevo -- si el espejo se hiciera sobre ellos, el pulso que
 * el operador manda a mano durante ESA calibracion dependeria de un
 * SERVO_PULSO_MIN/MAX todavia sin terminar de calibrar (circular). Con
 * el rango fijo, el mismo pulso logico siempre resulta en el mismo
 * pulso fisico sin importar en que estado este la calibracion del
 * acelerador -- para el operador, "pedir 900" siempre hace lo mismo.
 * Con esto, en los extremos SERVO_PULSO_MIN termina mandando
 * fisicamente el pulso que antes correspondia a SERVO_PULSO_MAX y
 * viceversa (siempre que ambos esten dentro de 500-2500), que es
 * exactamente el intercambio de sentido esperado. El resto del
 * firmware (PID, cada modo de CALIB, el barrido de
 * calibracion) sigue usando la misma convencion logica
 * (SERVO_PULSO_MIN=menos combustible, SERVO_PULSO_MAX=mas combustible)
 * sin cambios. Volver a 0 si se regresa al montaje biela-manivela, o
 * si se vuelve a montar el servo-directo con la orientacion original. */
#define SERVO_SENTIDO_INVERTIDO 0U
#define SERVO_PULSO_FISICO_MIN_US 500U
#define SERVO_PULSO_FISICO_MAX_US 2500U

/* ==================== ESTADO INTERNO ==================== */

static TIM_HandleTypeDef *s_htim = NULL;
static uint32_t s_canal = 0;
static uint16_t s_pulsoActualUs = 0;
static uint16_t s_pulsoFisicoActualUs = 0;
static uint32_t s_ultimoMovimientoMs = 0;
/* Avance permitido acumulado y todavia no aplicado, en milesimas de µs --
 * guarda los decimales que la division entera tiraba en cada vuelta. */
static uint32_t s_avanceMilesimasUs = 0;

/* ==================== API PÚBLICA ==================== */

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t canal)
{
    s_htim = htim;
    s_canal = canal;
    s_pulsoActualUs = 0;
    s_ultimoMovimientoMs = HAL_GetTick();
    s_avanceMilesimasUs = 0U;
}

uint16_t Servo_SetPulsoUs(uint16_t microsegundos)
{
    uint16_t minimo = CalibFlash_GetServoPulsoMinUs();
    uint16_t maximo = CalibFlash_GetServoPulsoMaxUs();

    uint16_t valorRecortado = microsegundos;
    if (valorRecortado < minimo) {
        valorRecortado = minimo;
    }
    if (valorRecortado > maximo) {
        valorRecortado = maximo;
    }

    /* s_pulsoActualUs (y el valor de retorno) se quedan en la
     * convencion LOGICA -- lo unico que se invierte es el valor
     * fisico que efectivamente recibe el registro del timer. Asi
     * Servo_MoverHacia() (que rampea contra s_pulsoActualUs) y
     * Servo_GetPulsoActualUs() (usado en logs como PID_TEST) siguen
     * siendo comparables directamente contra SERVO_PULSO_MIN/MAX. */
    uint16_t valorFisico = valorRecortado;
#if SERVO_SENTIDO_INVERTIDO
    valorFisico = (uint16_t)(SERVO_PULSO_FISICO_MIN_US + SERVO_PULSO_FISICO_MAX_US - valorRecortado);
#endif

    __HAL_TIM_SET_COMPARE(s_htim, s_canal, valorFisico);
    s_pulsoActualUs = valorRecortado;
    s_pulsoFisicoActualUs = valorFisico;

    return valorRecortado;
}

uint16_t Servo_MoverHacia(uint16_t destinoUs)
{
    uint32_t ahora = HAL_GetTick();
    uint32_t transcurridoMs = ahora - s_ultimoMovimientoMs;
    s_ultimoMovimientoMs = ahora;

    /* Velocidad EXACTA (2026-10-02, hallazgo B19): antes el paso se calculaba
     * como VEL * ms / 1000 en enteros y se perdian los decimales en cada
     * vuelta -- con vueltas de 1-3 ms el servo iba a ~1000 µs/s y con 4 ms a
     * 1250, es decir, la velocidad dependia de lo ocupado que estuviera el
     * loop. Ahora el sobrante se acumula para la vuelta siguiente. */
    if (transcurridoMs > 60000U) {
        transcurridoMs = 60000U; /* tope anti-desborde; de sobra para ir de MIN a MAX */
    }
    s_avanceMilesimasUs += SERVO_VELOCIDAD_MAX_US_S * transcurridoMs;
    int32_t pasoMaximo = (int32_t)(s_avanceMilesimasUs / 1000U);
    int32_t diferencia = (int32_t)destinoUs - (int32_t)s_pulsoActualUs;

    int32_t siguiente;
    if (diferencia > pasoMaximo) {
        siguiente = (int32_t)s_pulsoActualUs + pasoMaximo;
        s_avanceMilesimasUs -= (uint32_t)pasoMaximo * 1000U;
    } else if (diferencia < -pasoMaximo) {
        siguiente = (int32_t)s_pulsoActualUs - pasoMaximo;
        s_avanceMilesimasUs -= (uint32_t)pasoMaximo * 1000U;
    } else {
        /* Llego (o ya estaba): no se guarda avance para despues, si no el
         * proximo movimiento arrancaria con un salto acumulado. */
        siguiente = destinoUs;
        s_avanceMilesimasUs = 0U;
    }

    return Servo_SetPulsoUs((uint16_t)siguiente);
}

uint16_t Servo_GetPulsoActualUs(void)
{
    return s_pulsoActualUs;
}

uint16_t Servo_GetPulsoFisicoActualUs(void)
{
    return s_pulsoFisicoActualUs;
}
