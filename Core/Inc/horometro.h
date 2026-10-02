/**
 * @file    horometro.h
 * @brief   Horómetro del motor: horas con el motor ENCENDIDO, persistentes.
 *
 * Funciona como el cuentakilómetros de un carro, pero en horas de motor:
 *   - Solo sube mientras el motor está encendido (estado ENCENDIDO o ACTIVO
 *     de main.c); nunca baja ni se reinicia solo.
 *   - Se puede FIJAR por downlink (HOROMETRO_H, ID 17, horas enteras) al
 *     instalar el TID en un motor que ya trae horas, o para corregirlo.
 *   - Se guarda en su PROPIA página de flash (página 62, 0x0801F000), no en
 *     la de calibración, como un cuaderno: cada guardado escribe un renglón
 *     nuevo de 8 bytes sin borrar nada, y la página (256 renglones) solo se
 *     borra cuando se llena. Al arrancar se lee el último renglón válido.
 *   - Se guarda al APAGARSE el motor (el tiempo de esa sesión, completo) y,
 *     mientras sigue encendido, cada HOROMETRO_GUARDADO_PERIODICO_S (30 min)
 *     como respaldo: si el TID pierde el voltaje con el motor andando, lo
 *     máximo que se pierde son 30 min.
 *   - Vida de la página: ~48 renglones/día con el motor 24 h -> una borrada
 *     cada ~5 días -> 10 000 borradas = más de 100 años.
 *
 * Decision del usuario 2026-10-02. ⚠️ Compila; no probado en equipo.
 */

#ifndef HOROMETRO_H
#define HOROMETRO_H

#include <stdint.h>
#include <stdbool.h>

/* Guardado de respaldo mientras el motor sigue encendido. */
#define HOROMETRO_GUARDADO_PERIODICO_S   1800U

/** Lee el último valor guardado. Llamar una vez al arrancar. */
void Horometro_Init(void);

/** Llamar en cada vuelta del loop con el estado (ya con debounce) del
 *  motor: suma el tiempo encendido y guarda al apagarse / cada 30 min. */
void Horometro_Update(bool motorEncendido);

/** Fija el horómetro (segundos totales) y lo guarda de inmediato. */
bool Horometro_Fijar(uint32_t segundos);

/** Total en segundos, incluido lo que lleva la sesión en curso. */
uint32_t Horometro_GetSegundos(void);

#endif /* HOROMETRO_H */
