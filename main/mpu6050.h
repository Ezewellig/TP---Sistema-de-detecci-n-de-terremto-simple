/* =====================================================================
 *  mpu6050.h - driver "casero" del GY-521 / MPU-6050
 *
 *  No usa ninguna libreria de terceros: solamente el driver I2C que ya
 *  viene dentro de ESP-IDF. Todo lo que es "entender el chip" (mapa de
 *  registros, configuracion, armado de los enteros de 16 bits) esta
 *  escrito a mano aca.
 * ===================================================================*/
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/* ---------------------------------------------------------------
 *  Conexionado fisico (GPIO del ESP32-C3 SuperMini)
 * -------------------------------------------------------------*/
#define PIN_I2C_SDA     5       /* GY-521 SDA */
#define PIN_I2C_SCL     6       /* GY-521 SCL */
#define I2C_FREQ_HZ     400000  /* modo rapido; si hay ruido bajar a 100000 */

/* Direccion I2C de 7 bits.
 * El pin AD0 del modulo esta sin conectar y la placa GY-521 lo lleva a
 * masa con una resistencia -> direccion 0x68.
 * Si AD0 se pone a 3.3 V, la direccion pasa a 0x69.               */
#define MPU_ADDR_7B     0x68

/* ---------------------------------------------------------------
 *  Mapa de registros del MPU-6050 (los que usamos)
 * -------------------------------------------------------------*/
#define REG_SMPLRT_DIV      0x19    /* divisor de la frecuencia de muestreo */
#define REG_CONFIG          0x1A    /* filtro digital pasabajos (DLPF)      */
#define REG_GYRO_CONFIG     0x1B    /* fondo de escala del giroscopo        */
#define REG_ACCEL_CONFIG    0x1C    /* fondo de escala del acelerometro     */
#define REG_ACCEL_CONFIG2   0x1D    /* solo MPU-6500/9250: DLPF del acelerometro */
#define REG_INT_PIN_CFG     0x37    /* como se comporta el pin INT          */
#define REG_INT_ENABLE      0x38    /* que eventos generan interrupcion     */
#define REG_INT_STATUS      0x3A    /* que evento la genero (se autolimpia) */
#define REG_ACCEL_XOUT_H    0x3B    /* primer byte de los 6 del acelerometro*/
#define REG_PWR_MGMT_1      0x6B    /* reset / sleep / fuente de reloj      */
#define REG_WHO_AM_I        0x75    /* siempre devuelve 0x68                */

/* Lectura cruda del acelerometro (unidades: LSB, sin convertir) */
typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
} accel_raw_t;

/* ---------------------------------------------------------------
 *  API
 * -------------------------------------------------------------*/
esp_err_t mpu_init(void);                                 /* bus + sensor  */
esp_err_t mpu_write(uint8_t reg, uint8_t val);            /* 1 registro    */
esp_err_t mpu_read(uint8_t reg, uint8_t *dst, size_t len);/* n registros   */
esp_err_t mpu_read_accel(accel_raw_t *out);               /* 6 bytes       */
esp_err_t mpu_habilitar_int_dato_listo(bool on);