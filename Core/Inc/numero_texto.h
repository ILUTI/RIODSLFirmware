/**
 * @file    numero_texto.h
 * @brief   Lector minimo de numeros decimales desde texto (reemplazo de
 *          strtof()/atof() de la libreria de C).
 *
 * Agregado 2026-09-30 para ahorrar flash: strtof() (comando_serial.c) y
 * atof() (gps.c) eran los UNICOS usos del lector completo de la libreria
 * (_strtod_l + mprec + gethex + hexnan...), que soporta notacion cientifica,
 * hexadecimal, NaN/Inf, locales, etc. y ocupaba ~6-7 KB. Ninguno de los dos
 * usos necesita eso: el valor tecleado por serial y los minutos NMEA del GPS
 * son siempre "[signo]digitos[.digitos]".
 *
 * Acepta: espacios iniciales, signo +/- opcional, parte entera, punto y
 * decimales (ej. "12", "-0.047", "+3.5", ".5", "5.", "12.34567").
 * NO acepta exponente (1e3), hexadecimal ni NaN/Inf -- se detiene en el
 * primer caracter que no encaje, igual que strtof(). Sin digitos devuelve 0
 * (igual que strtof()/atof()).
 */

#ifndef NUMERO_TEXTO_H
#define NUMERO_TEXTO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @param texto  cadena a leer (NULL devuelve 0).
 * @param fin    si no es NULL, recibe un puntero al primer caracter no leido
 *               (a 'texto' si no habia ningun numero), igual que strtof().
 */
float NumeroTexto_LeerDecimal(const char *texto, const char **fin);

/**
 * Lee hasta 'maxDigitos' digitos decimales seguidos desde *p (sin espacios
 * ni signo) y avanza *p al primer caracter no leido. Reemplaza a sscanf("%u")
 * / atoi() (2026-10-01): asi ya no se enlaza el lector de la libreria de C.
 * @return false si en *p no habia ningun digito (*valor queda en 0).
 */
bool NumeroTexto_LeerUint(const char **p, uint8_t maxDigitos, uint32_t *valor);

/** Valor de un caracter hexadecimal (0-9, a-f, A-F), o -1 si no lo es. */
int NumeroTexto_HexDigito(char c);

#ifdef __cplusplus
}
#endif

#endif /* NUMERO_TEXTO_H */
