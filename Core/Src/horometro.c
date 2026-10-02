/**
 * @file    horometro.c
 * @brief   Horómetro del motor -- ver horometro.h.
 */

#include "horometro.h"
#include "main.h"
#include <stdio.h>

/* ==================== UBICACIÓN EN FLASH ==================== */
/* Página 62, justo antes de la de calibración (63). El .ld declara 124K
 * para que el código nunca llegue aquí. */
#define HOROMETRO_FLASH_PAGE      62U
#define HOROMETRO_FLASH_ADDRESS   0x0801F000UL
#define HOROMETRO_FLASH_BYTES     2048U

/* Un renglón = un double word (lo mínimo que programa el STM32G4):
 * [segundos][~segundos]. El complemento distingue un renglón válido de uno
 * borrado (0xFFFFFFFF en las dos mitades) o a medio escribir. */
typedef struct {
    uint32_t segundos;
    uint32_t complemento;
} Renglon_t;

#define RENGLONES   (HOROMETRO_FLASH_BYTES / sizeof(Renglon_t))   /* 256 */

static const volatile Renglon_t *const s_pagina = (const volatile Renglon_t *)HOROMETRO_FLASH_ADDRESS;

/* ==================== ESTADO ==================== */

static uint32_t s_segundosGuardados = 0U;  /* último valor escrito en flash */
static uint32_t s_msSesion = 0U;           /* ms encendido aún no guardados */
static uint32_t s_ultimoTickMs = 0U;
static uint16_t s_siguienteRenglon = 0U;   /* primer renglón libre */
static bool     s_encendidoAntes = false;

/* ==================== FLASH ==================== */

static bool RenglonLibre(uint16_t i)
{
    return s_pagina[i].segundos == 0xFFFFFFFFUL && s_pagina[i].complemento == 0xFFFFFFFFUL;
}

static bool RenglonValido(uint16_t i)
{
    return s_pagina[i].complemento == ~s_pagina[i].segundos;
}

static bool BorrarPagina(void)
{
    FLASH_EraseInitTypeDef borrado = {0};
    uint32_t paginaConError = 0U;
    borrado.TypeErase = FLASH_TYPEERASE_PAGES;
    borrado.Banks = FLASH_BANK_1;
    borrado.Page = HOROMETRO_FLASH_PAGE;
    borrado.NbPages = 1U;
    return HAL_FLASHEx_Erase(&borrado, &paginaConError) == HAL_OK;
}

/* Escribe 'segundos' en el siguiente renglón libre; si la página está llena,
 * la borra primero (única borrada, cada 256 guardados). */
static bool Guardar(uint32_t segundos)
{
    HAL_FLASH_Unlock();
    bool ok = true;
    if (s_siguienteRenglon >= RENGLONES) {
        ok = BorrarPagina();
        s_siguienteRenglon = 0U;
    }
    if (ok) {
        uint64_t dato = ((uint64_t)(~segundos) << 32) | (uint64_t)segundos;
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                               HOROMETRO_FLASH_ADDRESS + (uint32_t)s_siguienteRenglon * sizeof(Renglon_t),
                               dato) == HAL_OK;
        s_siguienteRenglon++;
    }
    HAL_FLASH_Lock();
    if (ok) {
        s_segundosGuardados = segundos;
    } else {
        printf("HOROMETRO: error al escribir flash (renglon %u)\r\n", (unsigned)(s_siguienteRenglon - 1U));
    }
    return ok;
}

/* ==================== API ==================== */

void Horometro_Init(void)
{
    s_segundosGuardados = 0U;
    s_siguienteRenglon = 0U;
    for (uint16_t i = 0U; i < RENGLONES; i++) {
        if (RenglonLibre(i)) {
            s_siguienteRenglon = i;
            break;
        }
        if (RenglonValido(i)) {
            s_segundosGuardados = s_pagina[i].segundos;
        }
        s_siguienteRenglon = (uint16_t)(i + 1U); /* renglón usado (o dañado): seguir */
    }
    s_msSesion = 0U;
    s_ultimoTickMs = HAL_GetTick();
    s_encendidoAntes = false;
    printf("HOROMETRO: %lu.%lu h (renglon %u de %u)\r\n",
           (unsigned long)(s_segundosGuardados / 3600UL),
           (unsigned long)((s_segundosGuardados % 3600UL) / 360UL),
           (unsigned)s_siguienteRenglon, (unsigned)RENGLONES);
}

void Horometro_Update(bool motorEncendido)
{
    uint32_t ahora = HAL_GetTick();
    uint32_t dtMs = ahora - s_ultimoTickMs;
    s_ultimoTickMs = ahora;

    if (motorEncendido) {
        s_msSesion += dtMs;
        /* Respaldo cada 30 min de motor: si el TID pierde el voltaje con el
         * motor andando, lo máximo que se pierde es esto. */
        if (s_msSesion >= HOROMETRO_GUARDADO_PERIODICO_S * 1000UL) {
            uint32_t s = s_msSesion / 1000UL;
            if (Guardar(s_segundosGuardados + s)) {
                s_msSesion -= s * 1000UL;
            }
        }
    } else if (s_encendidoAntes && s_msSesion >= 1000UL) {
        /* Se apagó: guardar lo que llevaba la sesión (al segundo). */
        uint32_t s = s_msSesion / 1000UL;
        if (Guardar(s_segundosGuardados + s)) {
            s_msSesion -= s * 1000UL;
        }
    }
    s_encendidoAntes = motorEncendido;
}

bool Horometro_Fijar(uint32_t segundos)
{
    s_msSesion = 0U;
    return Guardar(segundos);
}

uint32_t Horometro_GetSegundos(void)
{
    return s_segundosGuardados + s_msSesion / 1000UL;
}
