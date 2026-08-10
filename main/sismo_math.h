#pragma once

#include <stdint.h>

/* Devuelve x*x + y*y + z*z  (modulo al cuadrado del vector aceleracion) */
uint32_t asm_mag2(int32_t x, int32_t y, int32_t z);

/* Raiz cuadrada entera por el metodo digito a digito (sin flotantes) */
uint32_t asm_isqrt(uint32_t v);

/* Maximo de dos enteros sin signo (para ir guardando el pico) */
uint32_t asm_max_u32(uint32_t a, uint32_t b);