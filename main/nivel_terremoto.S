.section .text
.align 2

.global sumar_cuadrados
.type sumar_cuadrados, @function
sumar_cuadrados:
    mul a0, a0, a0
    mul a1, a1, a1
    mul a2, a2, a2
    add a0, a0, a1
    add a0, a0, a2
    ret
    .size sumar_cuadrados, .-sumar_cuadrados

    .global elegir_mayor
    .type elegir_mayor, @function
elegir_mayor:
    bgeu a0, a1, 1f
    mv a0, a1
1:  ret
    .size elegir_mayor, .-elegir_mayor

    .global raiz_cuadrada
    .type raiz_cuadrada, @function
raiz_cuadrada:
    li t0, 0
    li t1, 0x40000000

buscar_inicio:
    bgeu a0, t1, empezar
    srli t1, t1, 2
    bnez t1, buscar_inicio

empezar:
    beqz t1, fin_raiz

repetir:
    add t2, t0, t1
    bltu a0, t2, no_suma
    sub a0, a0, t2
    srli t0, t0, 1
    add t0, t0, t1
    j siguiente
no_suma:
    srli t0, t0, 1
siguiente:
    srli t1, t1, 2
    bnez t1, repetir

fin_raiz:
    mv a0, t0
    ret
    .size raiz_cuadrada, .-raiz_cuadrada
