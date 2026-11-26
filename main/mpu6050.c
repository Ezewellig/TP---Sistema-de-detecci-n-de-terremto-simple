#include "mpu6050.h"

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *ETIQUETA ="mpu6050";

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t sensor;

esp_err_t escribir_mpu(uint8_t registro, uint8_t valor){
    uint8_t paquete[2]={ registro, valor};
    return i2c_master_transmit(sensor, paquete, sizeof(paquete), 100);
}

esp_err_t leer_mpu(uint8_t registro, uint8_t *destino, size_t cantidad){
    return i2c_master_transmit_receive(sensor, &registro, 1, destino, cantidad, 100);
}

esp_err_t leer_aceleracion(lectura_t *lectura){
    uint8_t datos[6];
    esp_err_t resultado=leer_mpu(REG_PRIMER_DATO, datos, sizeof(datos));
    if (resultado!=ESP_OK){
        return resultado;
    }
    lectura->x=(int16_t)(((uint16_t)datos[0]<<8) | datos[1]);
    lectura->y=(int16_t)(((uint16_t)datos[2]<<8) | datos[3]);
    lectura->z=(int16_t)(((uint16_t)datos[4]<<8) | datos[5]);
    return ESP_OK;
}

esp_err_t activar_aviso_mpu(bool prender){
    return escribir_mpu(REG_ACTIVAR_AVISO, prender ? 0x01 : 0x00);
}

esp_err_t iniciar_mpu(void){
    i2c_master_bus_config_t config_bus={
        .clk_source= I2C_CLK_SRC_DEFAULT,
        .i2c_port=I2C_NUM_0,
        .scl_io_num=PIN_RELOJ,
        .sda_io_num=PIN_DATOS,
        .glitch_ignore_cnt= 7,
        .flags.enable_internal_pullup=true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&config_bus, &bus));

    i2c_device_config_t config_sensor={
        .dev_addr_length=I2C_ADDR_BIT_LEN_7,
        .device_address=DIRECCION_MPU,
        .scl_speed_hz=VELOCIDAD_BUS,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &config_sensor, &sensor));

    ESP_ERROR_CHECK(escribir_mpu(REG_ENCENDIDO, 0x80));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(escribir_mpu(REG_ENCENDIDO, 0x01));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(escribir_mpu(REG_FILTRO, 0x03));  
    ESP_ERROR_CHECK(escribir_mpu(REG_VELOCIDAD_LECTURAS, 9));
    ESP_ERROR_CHECK(escribir_mpu(REG_RANGO_ACELERACION, 0x00));
    ESP_ERROR_CHECK(escribir_mpu(REG_FILTRO_ACELERACION, 0x03));
    ESP_ERROR_CHECK(escribir_mpu(REG_CONFIG_PIN_AVISO, 0x00));
    ESP_ERROR_CHECK(activar_aviso_mpu(false));

    ESP_LOGI(ETIQUETA, "MPU configurado: +-2 g");
    return ESP_OK;
}
