/**
 ******************************************************************************
 * @file    modo_fsm.h
 * @brief   Reglas de la maquina de estados de MODO (hoja "Maquina de estados"
 *          de Documentacion.xlsx), en funciones PURAS: no tocan hardware ni
 *          flash, solo deciden. Asi quedan en un solo lugar, alineadas con la
 *          hoja y faciles de probar. Quien las llama (calibracion_flash.c para
 *          los downlinks/consola, main.c para los eventos) aplica el resultado.
 *
 *          MODO 5 (diagnostico) y MODO 6 (mantenimiento) NO existen todavia.
 ******************************************************************************
 */

#ifndef MODO_FSM_H
#define MODO_FSM_H

#include <stdbool.h>
#include <stdint.h>
#include "calibracion_flash.h"

/** Tabla A: reglas de ENTRADA a un MODO pedido por el operador (downlink o
 * consola). Devuelve CALIB_STATUS_OK si se puede intentar el cambio, o el
 * motivo del rechazo.
 *   - MODO 0, 3, 4: siempre aceptados (3 y 4 tambien con motor apagado).
 *   - MODO 1 y 2: rechazados con motor detenido.
 *   - MODO 1: desde cualquier modo (decision 2026-10-01, no se restringe).
 *   - MODO 2: requiere PRESION_OBJETIVO_REMOTO > 0, venir de MODO 1 (o ya
 *     estar en 2) y 'remotoDisponible' (enlace LoRa unido Y un PRESION_REMOTO
 *     recibido en los ultimos TIMEOUT_SIN_COMANDO_S). Vale igual si se pide
 *     por downlink o por la consola serial (decision 2026-10-02). */
CalibFlash_ProtocoloStatus_t Modo_ValidarEntrada(CalibFlash_Modo_t actual,
                                                 uint8_t solicitado,
                                                 bool motorOperando,
                                                 float presionObjetivoRemoto,
                                                 bool remotoDisponible);

/** Nota 1 de la hoja: SET_RPM inicial al ENTRAR a MODO 3. Viniendo de MODO 1
 * o 2 conserva el RPM actual (para no bajar a ralenti con la tuberia con
 * agua), acotado a rpmMaxCarga; si el RPM actual no supera rpmMin, o viene de
 * cualquier otro MODO, devuelve 0 (ralenti). */
float Modo_SetRpmAlEntrarManual(CalibFlash_Modo_t anterior, float rpmActual,
                                float rpmMin, float rpmMaxCarga);

/** Tabla B, evento "enlace LoRaWAN perdido": que hacer segun el MODO. */
typedef enum {
    MODO_ACCION_NINGUNA = 0,       /* sin efecto */
    MODO_ACCION_A_MODO0,           /* pasar a MODO 0 (ralenti) */
    MODO_ACCION_A_MODO1,           /* pasar a MODO 1 */
    MODO_ACCION_A_MODO1_RECUPERABLE /* MODO 2 -> MODO 1, con vuelta automatica a 2 */
} Modo_Accion_t;

Modo_Accion_t Modo_AccionPerdidaEnlace(CalibFlash_Modo_t modo, bool motorOperando,
                                       bool sensorPresionValido, float presionPsi,
                                       float presionObjetivoLocal);

#endif /* MODO_FSM_H */
