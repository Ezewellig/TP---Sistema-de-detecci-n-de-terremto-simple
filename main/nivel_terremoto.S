    .section .text
    .align   2

    .global asm_mag2
    .type   asm_mag2, @function
asm_mag2:
    mul  a0, a0, a0   # a0 = x*x
    mul  a1, a1, a1   #a1 = y*y
    mul  a2, a2, a2   #a2 = z*z
    add  a0, a0, a1   #a0 = x*x + y*y
    add  a0, a0, a2   #a0 = x*x + y*y + z*z
    ret               #vuelve a C con el resultado en a0
    .size asm_mag2, .-asm_mag2

    .global asm_max_u32
    .type   asm_max_u32, @function
asm_max_u32:
    bgeu a0, a1, 1f        #si a0 >= a1 saltar adelante
    mv   a0, a1            #si no, el resultado es a1
1:  ret
    .size asm_max_u32, .-asm_max_u32

    .global asm_isqrt
    .type   asm_isqrt, @function
asm_isqrt:
    li   t0, 0                 # res = 0
    li   t1, 0x40000000        # bit = 4^15

buscar_bit_inicial:
    bgeu a0, t1, comenzar      # si v >= bit, ya lo encontramos
    srli t1, t1, 2             # bit = bit / 4
    bnez t1, buscar_bit_inicial

comenzar:
    beqz t1, terminar          # v era 0 -> la raiz es 0

bucle:
    add  t2, t0, t1            # t2 = res + bit
    bltu a0, t2, no_entra      # si v < res+bit, este bit no va
    sub  a0, a0, t2            # v -= (res + bit)
    srli t0, t0, 1
    add  t0, t0, t1            # res = (res >> 1) + bit
    j    siguiente_digito
no_entra:
    srli t0, t0, 1             # res = res >> 1
siguiente_digito:
    srli t1, t1, 2             # bit = bit / 4
    bnez t1, bucle

terminar:
    mv   a0, t0
    ret
    .size asm_isqrt, .-asm_isqrt