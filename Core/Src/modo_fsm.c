/**
 ******************************************************************************
 * @file    modo_fsm.c
 * @brief   Ver modo_fsm.h
 ******************************************************************************
 */

#include "modo_fsm.h"

CalibFlash_ProtocoloStatus_t Modo_ValidarEntrada(CalibFlash_Modo_t actual,
                                                 uint8_t solicitado,
                                                 bool motorOperando,
                                                 float presionObjetivoRemoto,
                                                 bool remotoDisponible)
{
    /* MODO 1/2 no se arman con el motor detenido (decision 2026-09-22): si el
     * PID de presion entrara a controlar desde el primer instante que el motor
     * arranque, sin que el operador confirmara que la tuberia esta llena,
     * podria pedir RPM alta contra una lectura irreal. MODO 0, 3 y 4 quedan
     * exentos. */
    if (!motorOperando &&
        (solicitado == (uint8_t)CALIB_MODO_PRESION_LOCAL ||
         solicitado == (uint8_t)CALIB_MODO_REMOTO)) {
        return CALIB_STATUS_REJECTED_ENGINE_STOPPED;
    }

    if (solicitado == (uint8_t)CALIB_MODO_REMOTO) {
        /* No se arma MODO 2 sin PRESION_OBJETIVO_REMOTO (decision 2026-09-23). */
        if (presionObjetivoRemoto <= 0.0f) {
            return CALIB_STATUS_REJECTED_OBJETIVO_REMOTO_NO_CONFIGURADO;
        }
        /* MODO 2 solo desde MODO 1 (o ya en 2): su base es la
         * PRESION_OBJETIVO_LOCAL que MODO 1 dejo sostenida (decision 2026-09-30). */
        if (actual != CALIB_MODO_PRESION_LOCAL && actual != CALIB_MODO_REMOTO) {
            return CALIB_STATUS_APPLY_ERROR;
        }
        /* MODO 2 desde downlink O consola, pero solo con enlace LoRa y el
         * aspersor reportando (decision 2026-10-02): sin eso el watchdog de
         * main.c lo degradaria a MODO 1 en la vuelta siguiente. */
        if (!remotoDisponible) {
            return CALIB_STATUS_APPLY_ERROR;
        }
    }

    return CALIB_STATUS_OK;
}

float Modo_SetRpmAlEntrarManual(CalibFlash_Modo_t anterior, float rpmActual,
                                float rpmMin, float rpmMaxCarga)
{
    if (anterior != CALIB_MODO_PRESION_LOCAL && anterior != CALIB_MODO_REMOTO) {
        return 0.0f;
    }
    if (rpmActual <= rpmMin) {
        return 0.0f;
    }
    return (rpmActual < rpmMaxCarga) ? rpmActual : rpmMaxCarga;
}

Modo_Accion_t Modo_AccionPerdidaEnlace(CalibFlash_Modo_t modo, bool motorOperando,
                                       bool sensorPresionValido, float presionPsi,
                                       float presionObjetivoLocal)
{
    switch (modo) {
    case CALIB_MODO_REMOTO:
        return MODO_ACCION_A_MODO1_RECUPERABLE;

    case CALIB_MODO_MANUAL_BANCO:
        /* A MODO 1 solo si ya hay presion suficiente (el PID no tiene que
         * empujar hacia arriba); si no, ralenti. */
        if (motorOperando && sensorPresionValido && presionPsi >= presionObjetivoLocal) {
            return MODO_ACCION_A_MODO1;
        }
        return MODO_ACCION_A_MODO0;

    case CALIB_MODO_CALIBRACION:
        return MODO_ACCION_A_MODO0; /* la prueba CALIB en curso se aborta sola */

    default: /* MODO 0 y MODO 1 */
        return MODO_ACCION_NINGUNA;
    }
}
