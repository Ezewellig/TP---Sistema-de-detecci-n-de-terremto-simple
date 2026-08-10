/* =====================================================================
 *  mpu6050.c - driver casero del GY-521 / MPU-6050
 * ===================================================================*/
#include "mpu6050.h"

#include "driver/i2c_master.h"   /* API nueva (ESP-IDF >= 5.2) */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "mpu6050";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

/* ---------------------------------------------------------------
 *  Acceso de bajo nivel
 *
 *  Escribir un registro  ->  [START][ADDR+W][reg][valor][STOP]
 *  Leer n registros      ->  [START][ADDR+W][reg][RESTART][ADDR+R][n bytes][STOP]
 *
 *  El MPU-6050 autoincrementa el puntero de registro, por eso con una
 *  sola transaccion podemos traer los 6 bytes del acelerometro.
 * -------------------------------------------------------------*/
esp_err_t mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100 /*ms*/);
}

esp_err_t mpu_read(uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, dst, len, 100 /*ms*/);
}

/* Los datos vienen en complemento a 2, big-endian (primero el byte alto). */
esp_err_t mpu_read_accel(accel_raw_t *out)
{
    uint8_t b[6];
    esp_err_t err = mpu_read(REG_ACCEL_XOUT_H, b, sizeof(b));
    if (err != ESP_OK) {
        return err;
    }
    out->ax = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    out->ay = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
    out->az = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
    return ESP_OK;
}

esp_err_t mpu_habilitar_int_dato_listo(bool on)
{
    return mpu_write(REG_INT_ENABLE, on ? 0x01 : 0x00);
}

/* ---------------------------------------------------------------
 *  Inicializacion completa
 * -------------------------------------------------------------*/
esp_err_t mpu_init(void)
{
    /* --- 1. bus I2C maestro --- */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .i2c_port                     = I2C_NUM_0,
        .scl_io_num                   = PIN_I2C_SCL,
        .sda_io_num                   = PIN_I2C_SDA,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,   /* el GY-521 ya trae 4k7,
                                                   esto es solo un respaldo */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU_ADDR_7B,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));

    /* --- 2. reset por software --- */
    ESP_ERROR_CHECK(mpu_write(REG_PWR_MGMT_1, 0x80));   /* bit7 = DEVICE_RESET */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* --- 3. verificacion de que hablamos con el chip correcto --- */
    uint8_t who = 0;
    ESP_ERROR_CHECK(mpu_read(REG_WHO_AM_I, &who, 1));

    const char *modelo;
    switch (who) {
        case 0x68: modelo = "MPU-6050"; break;
        case 0x70: modelo = "MPU-6500"; break;
        case 0x71: modelo = "MPU-9250"; break;
        case 0x73: modelo = "MPU-9255"; break;
        default:
            ESP_LOGE(TAG, "WHO_AM_I = 0x%02X: chip desconocido", who);
            return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Detectado: %s (WHO_AM_I = 0x%02X)", modelo, who);

    /* --- 4. configuracion ---
     * PWR_MGMT_1 = 0x01 -> sale de sleep y usa el PLL del giroscopo X
     *                      como reloj (mas estable que el oscilador RC). */
    ESP_ERROR_CHECK(mpu_write(REG_PWR_MGMT_1, 0x01));
    vTaskDelay(pdMS_TO_TICKS(20));

    /* CONFIG = 0x03 -> DLPF a 44 Hz. Filtra el ruido de alta frecuencia
     * y deja pasar la banda sismica util (0.1 - 20 Hz).
     * Con DLPF activo, la frecuencia base interna es 1 kHz.          */
    ESP_ERROR_CHECK(mpu_write(REG_CONFIG, 0x03));

    /* SMPLRT_DIV = 9 -> Fs = 1000 / (1 + 9) = 100 Hz                 */
    ESP_ERROR_CHECK(mpu_write(REG_SMPLRT_DIV, 9));

    /* Fondos de escala: giroscopo +-250 dps (no lo usamos),
     * acelerometro +-2 g  ->  16384 LSB por g  ->  0.061 mg por LSB.
     * Es el rango mas sensible, que es justo lo que queremos.        */
    ESP_ERROR_CHECK(mpu_write(REG_GYRO_CONFIG,  0x00));
    ESP_ERROR_CHECK(mpu_write(REG_ACCEL_CONFIG, 0x00));
    ESP_ERROR_CHECK(mpu_write(REG_ACCEL_CONFIG2, 0x03));

    /* INT_PIN_CFG = 0x00 -> INT activo en alto, push-pull, pulso de
     * 50 us que se limpia solo. Perfecto para un flanco de subida.   */
    ESP_ERROR_CHECK(mpu_write(REG_INT_PIN_CFG, 0x00));

    /* La interrupcion arranca deshabilitada: la prendemos recien
     * cuando la tarea que la atiende ya existe.                      */
    ESP_ERROR_CHECK(mpu_habilitar_int_dato_listo(false));

    ESP_LOGI(TAG, "%s configurado: +-2 g, DLPF 44 Hz, Fs = 100 Hz", modelo);
    return ESP_OK;
}