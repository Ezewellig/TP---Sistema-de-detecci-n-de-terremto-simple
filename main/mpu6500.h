#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#define PIN_DATOS 5
#define PIN_RELOJ 6
#define VELOCIDAD_BUS 400000
#define DIRECCION_MPU 0x68

#define REG_VELOCIDAD_LECTURAS  0x19
#define REG_FILTRO              0x1A
#define REG_GIROSCOPIO          0x1B
#define REG_RANGO_ACELERACION   0x1C
#define REG_FILTRO_ACELERACION  0x1D
#define REG_CONFIG_PIN_AVISO    0x37
#define REG_ACTIVAR_AVISO       0x38
#define REG_ESTADO_AVISO        0x3A
#define REG_PRIMER_DATO         0x3B
#define REG_ENCENDIDO           0x6B
#define REG_QUIEN_SOS           0x75

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lectura_t;

esp_err_t iniciar_mpu(void);
esp_err_t escribir_mpu(uint8_t registro, uint8_t valor);
esp_err_t leer_mpu(uint8_t registro, uint8_t *destino, size_t cantidad);
esp_err_t leer_aceleracion(lectura_t *lectura);
esp_err_t activar_aviso_mpu(bool prender);
