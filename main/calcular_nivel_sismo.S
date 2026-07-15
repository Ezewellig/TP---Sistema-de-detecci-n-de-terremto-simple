.section .text
.global calcular_nivel_sismo
.type calcular_nivel_sismo, @function

# int calcular_nivel_sismo(int golpes);
# Según la convención de llamadas de RISC-V:
# El argumento 'golpes' ingresa en el registro a0.
# El valor de retorno debe guardarse en el registro a0.

calcular_nivel_sismo:
    # Umbral para nivel MEDIO (Si hay menos de 4 golpes, es BAJO)
    li t0, 4
    blt a0, t0, sismo_bajo

    # Umbral para nivel ALTO (Si hay menos de 10 golpes, es MEDIO)
    li t1, 10
    blt a0, t1, sismo_medio

sismo_alto:
    li a0, 3          # Cargamos el valor 3 (Nivel Alto) en el registro de retorno
    ret               # Retornamos a la función en C

sismo_medio:
    li a0, 2          # Cargamos el valor 2 (Nivel Medio) en el registro de retorno
    ret               # Retornamos a la función en C

sismo_bajo:
    li a0, 1          # Cargamos el valor 1 (Nivel Bajo) en el registro de retorno
    ret               # Retornamos a la función en C