/**
 * @file    numero_texto.c
 * @brief   Lector minimo de decimales -- ver numero_texto.h.
 */

#include "numero_texto.h"
#include <stdint.h>
#include <stdbool.h>

/* Potencias de 10 exactas en float (hasta 1e10: todas representables sin
 * error, ver nota abajo). */
static const float s_pot10[] = {
    1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f, 100000.0f, 1000000.0f,
    10000000.0f, 100000000.0f, 1000000000.0f, 10000000000.0f
};
#define MAX_DIGITOS_DECIMALES   9U   /* entran en uint32 sin desbordar */

float NumeroTexto_LeerDecimal(const char *texto, const char **fin)
{
    if (fin != 0) *fin = texto;
    if (texto == 0) return 0.0f;

    const char *p = texto;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;

    bool negativo = false;
    if (*p == '+' || *p == '-') {
        negativo = (*p == '-');
        p++;
    }

    /* Parte entera acumulada en float (exacta hasta 16.7 millones, de sobra
     * para cualquier parametro de este proyecto). */
    float entero = 0.0f;
    bool hayDigitos = false;
    while (*p >= '0' && *p <= '9') {
        entero = entero * 10.0f + (float)(*p - '0');
        hayDigitos = true;
        p++;
    }

    /* Decimales acumulados como ENTERO y divididos una sola vez: asi el
     * resultado lleva a lo sumo dos redondeos (la division y la suma), en vez
     * de ir acumulando error digito por digito. Los digitos que pasen de 9 se
     * leen pero se ignoran (float no los podria representar igual). */
    uint32_t frac = 0U;
    uint8_t nFrac = 0U;
    if (*p == '.') {
        const char *trasPunto = p + 1;
        while (*trasPunto >= '0' && *trasPunto <= '9') {
            if (nFrac < MAX_DIGITOS_DECIMALES) {
                frac = frac * 10U + (uint32_t)(*trasPunto - '0');
                nFrac++;
            }
            hayDigitos = true;
            trasPunto++;
        }
        /* "5." y ".5" son validos; "." solo, no (ahi no se consume el punto). */
        if (hayDigitos) p = trasPunto;
    }

    if (!hayDigitos) {
        return 0.0f;             /* *fin ya apunta a 'texto', como strtof() */
    }
    if (fin != 0) *fin = p;

    float valor = entero;
    if (nFrac > 0U) {
        valor += (float)frac / s_pot10[nFrac];
    }
    return negativo ? -valor : valor;
}

bool NumeroTexto_LeerUint(const char **p, uint8_t maxDigitos, uint32_t *valor)
{
    const char *q = *p;
    uint32_t v = 0U;
    uint8_t n = 0U;
    while (n < maxDigitos && *q >= '0' && *q <= '9') {
        v = v * 10U + (uint32_t)(*q - '0');
        q++;
        n++;
    }
    *valor = v;
    *p = q;
    return n > 0U;
}

int NumeroTexto_HexDigito(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
