#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"         // NUEVA LIBRERIA PARA I2C
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_err.h"

// --- NUEVOS PINES SEGÚN PROTOBOARD ---
#define LED_VERDE 1
#define LED_AMARILLO 2
#define LED_ROJO 10
#define SENSOR_PIN 0            // Pin conectado a INT del MPU6050

// --- PINES I2C ---
#define I2C_MASTER_SCL_IO 7     // SCL de la protoboard
#define I2C_MASTER_SDA_IO 6     // SDA de la protoboard
#define I2C_MASTER_NUM 0
#define MPU6050_ADDR 0x69       // Dirección I2C del MPU6050 (con AD0 a GND)

// Definimos el maximo de eventos que podemos almacenar en el buffer de golpes
#define MAX_EVENTOS 100
// Ventana de tiempo de 10 segundos para contar los golpes
#define VENTANA_MS 10000

// Variables volatiles porque se modifican dentro de la interrupcion.
volatile uint64_t golpes[MAX_EVENTOS];
volatile int cantidad_golpes = 0;
volatile int golpesTotalesEvento = 0;

// Iniciamos el evento en falso y ponemos todo en 0
volatile bool eventoActivo = false;
volatile uint64_t inicioEvento = 0;
volatile int nivelMaximo = 0;

// Declaracion de la funcion en Assembler RISC-V
extern int calcular_nivel_sismo(int golpes);

// Declaracion de funciones
void actualizar_leds(int nivel);
void reportar_nivel(int nivel);
void actualizarVentana();

// --- FUNCIONES DE I2C Y SENSOR MPU6050 ---

// Inicializa el bus I2C
esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Escribe un byte en un registro del MPU6050
esp_err_t mpu_write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Configura el MPU6050 para Detección de Movimiento por Interrupción
void mpu6050_config_motion(void) {
    uint8_t UMBRAL_GOLPE = 2; // Lo dejamos en 2 para probar
    uint8_t DURACION_GOLPE = 1;

    // Intentamos despertar el sensor y verificamos si responde
    esp_err_t err = mpu_write_byte(0x6B, 0x00);         
    
    if (err != ESP_OK) {
        printf("\n🚨 ERROR CRITICO: El ESP32 no encuentra al MPU6050.\n");
        printf("🚨 Revisa los cables SDA, SCL, GND y VCC.\n\n");
    } else {
        printf("\n✅ ¡Comunicacion con MPU6050 exitosa!\n\n");
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    mpu_write_byte(0x1C, 0x01);         
    mpu_write_byte(0x1F, UMBRAL_GOLPE); 
    mpu_write_byte(0x20, DURACION_GOLPE); 
    mpu_write_byte(0x37, 0x00);         
    mpu_write_byte(0x38, 0x40);         
}

// --- INTERRUPCIÓN ---
// IRAM_ATTR guarda esta funcion en la memoria RAM para que sea mas rapida
void IRAM_ATTR ISR_sensor(void* arg) { 
    static uint64_t ultima_ISR = 0;
    uint64_t tiempo_Ahora = esp_timer_get_time() / 1000;

    // Filtro anti-rebote: Si el tiempo transcurrido es menor a 50ms, lo ignoramos
    if(tiempo_Ahora - ultima_ISR < 50)
        return;

    ultima_ISR = tiempo_Ahora;
    if(cantidad_golpes < MAX_EVENTOS) {
        golpes[cantidad_golpes] = tiempo_Ahora;
        cantidad_golpes++; 
        golpesTotalesEvento++; 
    }
}

// --- APP MAIN ---
void app_main(void)
{
    // Configuración de los pines de los leds como salida
    gpio_set_direction(LED_VERDE, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_AMARILLO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_ROJO, GPIO_MODE_OUTPUT);
    
    // Inicialización I2C y MPU6050
    printf("Inicializando I2C y MPU6050...\n");
    i2c_master_init();
    mpu6050_config_motion();

    // Configuración del pin del sensor (ahora es el INT del MPU6050)
    gpio_set_direction(SENSOR_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SENSOR_PIN, GPIO_PULLDOWN_ONLY);
    
    // IMPORTANTE: Como configuramos INT_PIN_CFG en activo ALTO (0x00), 
    // el sensor tira un pulso hacia arriba (POSEDGE) cuando detecta un golpe.
    gpio_set_intr_type(SENSOR_PIN, GPIO_INTR_POSEDGE);
    
    // Instalar el servicio de interrupciones y registrar la ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SENSOR_PIN, ISR_sensor, NULL);
    
    printf("\nSistema de vigilancia activo\n");
    printf("Esperando vibraciones...\n");
    actualizar_leds(0);

    // Bucle principal
    while(1)
    {
        actualizarVentana();
        
        if(cantidad_golpes > 0)
        { 
            // Si hay golpes dentro de la ventana de tiempo, entonces hay un evento activo
            if(!eventoActivo)
            { 
                eventoActivo = true;
                inicioEvento = golpes[0];
                printf("\nEVENTO DETECTADO\n");
            }
            
            // Llamamos la funcion en Assembler para calcular el nivel del sismo
            int nivel = calcular_nivel_sismo(cantidad_golpes);
            
            // Actualizamos nivel maximo y reportamos
            if(nivel > nivelMaximo)
            {
                nivelMaximo = nivel;
                reportar_nivel(nivel);
                actualizar_leds(nivel);
            }
         }
        else
        {
            // Si no hay golpes, el evento ha terminado
            if(eventoActivo)
            {
                eventoActivo = false;
                printf("\nFIN DEL EVENTO\n");
                printf("Duracion(ms): %llu\n", (esp_timer_get_time() / 1000) - inicioEvento);
                printf("Cantidad de golpes totales: %d\n", golpesTotalesEvento); 
                printf("Nivel maximo: ");
                reportar_nivel(nivelMaximo);
                
                // Reseteo para el siguiente evento
                nivelMaximo = 0;
                golpesTotalesEvento = 0;
                actualizar_leds(0);

                printf("\nSistema nuevamente en vigilancia.\n");
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}

// --- FUNCIONES SECUNDARIAS ---

void actualizarVentana()
{
    uint64_t tiempo_Ahora = esp_timer_get_time() / 1000;
    int i = 0;

    while(i < cantidad_golpes)
    {
       if(tiempo_Ahora - golpes[i] <= VENTANA_MS)
            break;
        i++;
    }

    if(i > 0)
    {
        for(int j = 0; j < cantidad_golpes - i; j++)
            golpes[j] = golpes[j + i];
        cantidad_golpes -= i;
    }
}

void reportar_nivel(int nivel)
{
    printf("\nNivel: ");
    switch(nivel)
    {
        case 1:
            printf("BAJO\n");
            break;
        case 2:
            printf("MEDIO\n");
            break;
        case 3:
            printf("ALTO\n");
            break;
    }
}

void actualizar_leds(int nivel)
{
    gpio_set_level(LED_VERDE, (nivel == 0 || nivel == 1) ? 1 : 0);
    gpio_set_level(LED_AMARILLO, (nivel == 2) ? 1 : 0);
    gpio_set_level(LED_ROJO, (nivel == 3) ? 1 : 0);
}