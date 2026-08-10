#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#define PIN_I2C_SDA 5       //SDA
#define PIN_I2C_SCL 6       //SCL
#define I2C_FREQ_HZ 400000  //modo rapido
#define MPU_ADDR_7B 0x68
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A    
#define REG_GYRO_CONFIG   0x1B    
#define REG_ACCEL_CONFIG  0x1C    
#define REG_ACCEL_CONFIG2 0x1D
#define REG_INT_PIN_CFG   0x37    
#define REG_INT_ENABLE    0x38    
#define REG_INT_STATUS    0x3A    
#define REG_ACCEL_XOUT_H  0x3B    
#define REG_PWR_MGMT_1    0x6B    
#define REG_WHO_AM_I      0x75

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
} accel_raw_t;

esp_err_t mpu_init(void);
esp_err_t mpu_write(uint8_t reg, uint8_t val);
esp_err_t mpu_read(uint8_t reg, uint8_t *dst, size_t len);
esp_err_t mpu_read_accel(accel_raw_t *out);
esp_err_t mpu_habilitar_int_dato_listo(bool on);