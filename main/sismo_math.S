/* =====================================================================
 *  sismo_math.S  -  ASSEMBLER RISC-V (RV32IMC) para el ESP32-C3
 *
 *  OJO: el ESP32-C3 NO es Xtensa como el ESP32 clasico. Su nucleo es un
 *  RISC-V de 32 bits, asi que el assembler es RISC-V.
 *
 *  Registros usados (nombres ABI):
 *      a0, a1, a2 = argumentos 1, 2 y 3   (y a0 = valor devuelto)
 *      t0, t1, t2 = temporales
 *      ra         = return address; "ret" es en realidad "jalr x0, ra, 0"
 * ===================================================================*/

    .section .text
    .align   2

/* ---------------------------------------------------------------------
 *  uint32_t asm_mag2(int32_t x, int32_t y, int32_t z);
 *
 *  Calcula  x*x + y*y + z*z.
 *  Es el cuadrado del modulo del vector aceleracion. Trabajamos con el
 *  cuadrado (y no con el modulo) porque asi se puede acumular la energia
 *  de la sacudida sin sacar raiz en cada muestra.
 *
 *  Rango: cada componente cabe en +-32767, asi que el peor caso es
 *  3 * 32767^2 = 3.221.078.787, que entra justo en un uint32.
 * -------------------------------------------------------------------*/
    .global asm_mag2
    .type   asm_mag2, @function
asm_mag2:
    mul  a0, a0, a0        /* a0 = x*x      (instruccion de la extension M) */
    mul  a1, a1, a1        /* a1 = y*y                                      */
    mul  a2, a2, a2        /* a2 = z*z                                      */
    add  a0, a0, a1        /* a0 = x*x + y*y                                */
    add  a0, a0, a2        /* a0 = x*x + y*y + z*z                          */
    ret                    /* vuelve a C con el resultado en a0             */
    .size asm_mag2, .-asm_mag2


/* ---------------------------------------------------------------------
 *  uint32_t asm_max_u32(uint32_t a, uint32_t b);
 *
 *  Devuelve el mayor de los dos, sin signo. Lo usamos para ir
 *  actualizando el pico de la sacudida muestra a muestra.
 * -------------------------------------------------------------------*/
    .global asm_max_u32
    .type   asm_max_u32, @function
asm_max_u32:
    bgeu a0, a1, 1f        /* si a0 >= a1 (unsigned) saltar adelante  */
    mv   a0, a1            /* si no, el resultado es a1               */
1:  ret
    .size asm_max_u32, .-asm_max_u32


/* ---------------------------------------------------------------------
 *  uint32_t asm_isqrt(uint32_t v);
 *
 *  Raiz cuadrada ENTERA, metodo "digito a digito" en base 4.
 *  Sin coma flotante, sin tablas, sin divisiones: solo sumas, restas,
 *  comparaciones y desplazamientos. Termina en 16 iteraciones como
 *  maximo, con tiempo de ejecucion practicamente constante.
 *
 *  Equivalente en C:
 *      uint32_t res = 0, bit = 1u << 30;
 *      while (bit > v)   bit >>= 2;
 *      while (bit) {
 *          if (v >= res + bit) { v -= res + bit; res = (res >> 1) + bit; }
 *          else                {                 res =  res >> 1;        }
 *          bit >>= 2;
 *      }
 *      return res;
 *
 *  Registros: a0 = v (se va consumiendo), t0 = res, t1 = bit, t2 = res+bit
 * -------------------------------------------------------------------*/
    .global asm_isqrt
    .type   asm_isqrt, @function
asm_isqrt:
    li   t0, 0                 /* res = 0                              */
    li   t1, 0x40000000        /* bit = 1 << 30 (potencia de 4 mas alta)*/

/* --- fase 1: bajar "bit" hasta que sea <= v --- */
1:  bgeu a0, t1, 2f            /* si v >= bit, ya esta                 */
    srli t1, t1, 2             /* bit >>= 2                            */
    bnez t1, 1b                /* mientras bit != 0, seguir bajando    */

/* --- fase 2: ir armando la raiz de a un bit por vez --- */
2:  beqz t1, 9f                /* v era 0 -> la raiz es 0              */
3:  add  t2, t0, t1            /* t2 = res + bit                       */
    bltu a0, t2, 5f            /* si v < res+bit, este bit no va       */
    sub  a0, a0, t2            /* v -= (res + bit)                     */
    srli t0, t0, 1             /* res >>= 1                            */
    add  t0, t0, t1            /* res += bit  -> este bit SI va        */
    j    6f
5:  srli t0, t0, 1             /* res >>= 1                            */
6:  srli t1, t1, 2             /* bit >>= 2 (pasamos al digito siguiente)*/
    bnez t1, 3b

9:  mv   a0, t0                /* resultado en a0                      */
    ret
    .size asm_isqrt, .-asm_isqrt