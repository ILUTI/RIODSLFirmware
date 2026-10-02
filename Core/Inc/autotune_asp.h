/**
 * @file    autotune_asp.h
 * @brief   Autosintonia del PID#3 (MODO=2, aspersor remoto, triple cascada),
 *          CALIB_AUTOTUNE_PID3 (=12). Reemplaza (2026-09-30) al viejo
 *          auto-escalon remoto de CALIB=12, que no estaba terminado.
 *
 * ⚠️ NO PROBADA CON MOTOBOMBA NI ASPERSOR REAL.
 *
 * Lo que se identifica NO es la bomba (eso ya es el PID#2) sino la relacion
 * presion LOCAL -> presion del ASPERSOR: ganancia K (~1 PSI por PSI menos lo
 * que pierde el tubo) y retardo L (dominado por la cadencia de reportes del
 * aspersor, 20 s fuera de su objetivo / 90 s en su objetivo).
 *
 * Dos formas de arrancar (decision del usuario 2026-09-30):
 *   - Desde MODO=4 (puesta en marcha, tuberia posiblemente vacia): primero
 *     sube a PRESION_OBJETIVO_LOCAL como MODO=1 (rampa de llenado al ritmo
 *     PRESION_OBJETIVO_LOCAL / TIEMPO_LLENADO_S; con la tuberia llena tarda poco).
 *   - Desde MODO=1 ya operando (re-ajuste rapido despues de mover el
 *     aspersor o cambiar la tuberia): se salta la subida.
 *
 * Secuencia:
 *   1. LLENADO   : presion local estable en PRESION_OBJETIVO_LOCAL (PID#2).
 *   2. BASE      : 3 reportes seguidos del aspersor dentro de 1 PSI -> R0.
 *   3. ESCALON   : objetivo local +5 PSI (-5 si no cabe bajo PRESION_MAX), al
 *                  ritmo de llenado; se espera a que el aspersor se estabilice.
 *                  K = cambio aspersor / cambio local; L = retardo entre que el
 *                  OBJETIVO local y la presion del aspersor cruzan la mitad de
 *                  su cambio (incluye lo que tarda el PID#2 en seguir su
 *                  objetivo, que es parte del lazo del PID#3).
 *   4. CALCULO   : SIMC con tau ~ 0 (la hidraulica es rapida frente a 20 s de
 *                  reporte) -> control INTEGRAL: Kp = 0, Ki = 1/(K*(tau_c+L)),
 *                  tau_c = max(2L, 40 s).
 *   5. VALIDACION: vuelve a la base y activa la triple cascada real con la
 *                  candidata (RAM) buscando PRESION_OBJETIVO_REMOTO; aprueba si
 *                  los ultimos 3 reportes quedan dentro de la banda sin oscilar.
 *   6. Aprobada -> guarda PID_ASP_KP/KI. Si no, tau_c x2 (hasta 3 intentos).
 *      Si ninguna pasa, deja las ganancias como estaban.
 */

#ifndef AUTOTUNE_ASP_H
#define AUTOTUNE_ASP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "autotune.h"
#include "calibracion_flash.h"

void AutotuneAsp_Iniciar(void);
void AutotuneAsp_Paso(bool motorOperando, CalibFlash_Modo_t modo);
void AutotuneAsp_Limpiar(void);
Autotune_Resultado_t AutotuneAsp_UltimoResultado(void);

/** true mientras la rutina corre (main.c activa la cascada de presion local). */
bool  AutotuneAsp_CascadaActiva(void);
/** Objetivo de presion LOCAL que debe perseguir el PID#2 ahora (RAM, ya con
 *  la rampa al ritmo de llenado). No toca PRESION_OBJETIVO_LOCAL. */
float AutotuneAsp_ObjetivoLocal(void);

#ifdef __cplusplus
}
#endif

#endif /* AUTOTUNE_ASP_H */
