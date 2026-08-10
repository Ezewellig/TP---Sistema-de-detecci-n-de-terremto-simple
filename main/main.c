#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nivel_terremoto.h"
#include "mpu6500.h"

#define PIN_SENSOR 4

//ventana
#define VENTANA_MS 10000
#define VENTANA_US (VENTANA_MS * 1000LL)
#define MAX_GOLPES 64
#define REFRACTARIO_MS 120 //Si despues de un golpe se sigue detectando movimiento, subir este valor
#define UMBRAL_GOLPE_MG 40 //aceleracion minima para contar
#define MUESTRAS_CALIB 200
#define K_BASE 6

//clasificacion por CANTIDAD de golpes en la ventana
#define N_PARA_MEDIO 10 //  10 a 24 golpes -> MEDIO
#define N_PARA_ALTO 25 // >=25 golpes -> ALTO
// 1 a 9 golpes  -> BAJO

//LEDs: preparado para el futuro, apagado por ahora
#define USAR_LEDS           0
#define PIN_LED_VERDE       0
#define PIN_LED_AMARILLO    1
#define PIN_LED_ROJO        3

static const char *TAG = "sismo";

typedef enum {
    NIVEL_NINGUNO = 0,
    NIVEL_BAJO,
    NIVEL_MEDIO,
    NIVEL_ALTO
} nivel_t;

static const char *NOMBRE_NIVEL[] = { "-", "BAJO", "MEDIO", "ALTO" };

static TaskHandle_t s_tarea;

//cola FIFO circular de marcas de tiempo
static int64_t s_cola[MAX_GOLPES];
static int s_ini  = 0;
static int s_cant = 0;

//terremoto en curso
static bool s_evento_activo  = false;
static int64_t s_t_primer_golpe = 0;
static int64_t s_t_ultimo_golpe = 0;
static uint32_t s_golpes_totales = 0;
static nivel_t s_nivel_actual = NIVEL_NINGUNO;
static nivel_t s_nivel_maximo = NIVEL_NINGUNO;
static int64_t s_t_ultimo_contado = 0;
static uint32_t s_pico_lsb = 0;

//Ventana deslizante
static void ventana_reset(void)
{
    s_ini = 0;
    s_cant = 0;
}

static void ventana_agregar(int64_t t)
{
    if (s_cant == MAX_GOLPES) {
        s_ini = (s_ini + 1) % MAX_GOLPES;
        s_cant--;
    }
    s_cola[(s_ini + s_cant) % MAX_GOLPES] = t;
    s_cant++;
}

//Descarta lo que quedo fuera de la ventana y devuelve cuantos quedan
static int ventana_contar(int64_t ahora)
{
    while (s_cant > 0 && (ahora - s_cola[s_ini]) > VENTANA_US) {
        s_ini = (s_ini + 1) % MAX_GOLPES;
        s_cant--;
    }
    return s_cant;
}

//Clasificacion
static nivel_t clasificar(int golpes_en_ventana)
{
    if (golpes_en_ventana >= N_PARA_ALTO) return NIVEL_ALTO;
    if (golpes_en_ventana >= N_PARA_MEDIO) return NIVEL_MEDIO;
    if (golpes_en_ventana >= 1) return NIVEL_BAJO;
    return NIVEL_NINGUNO;
}

//int
static void IRAM_ATTR isr_sensor(void *arg)
{
    BaseType_t hay_tarea_mas_prioritaria = pdFALSE;
    vTaskNotifyGiveFromISR(s_tarea, &hay_tarea_mas_prioritaria);
    portYIELD_FROM_ISR(hay_tarea_mas_prioritaria);
}

//HACER LEDS
static void indicar_nivel(nivel_t n)
{
#if USAR_LEDS
    gpio_set_level(PIN_LED_VERDE, n == NIVEL_BAJO);
    gpio_set_level(PIN_LED_AMARILLO, n == NIVEL_MEDIO);
    gpio_set_level(PIN_LED_ROJO, n == NIVEL_ALTO);
#else
    (void)n;
#endif
}

static void init_leds(void)
{
#if USAR_LEDS
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_LED_VERDE) |
                        (1ULL << PIN_LED_AMARILLO) |
                        (1ULL << PIN_LED_ROJO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    indicar_nivel(NIVEL_NINGUNO);
#endif
}

//Informe de golpe
static void registrar_golpe(int64_t ahora, uint32_t intensidad_mg)
{
    if (s_t_ultimo_contado != 0 &&
        (ahora - s_t_ultimo_contado) < (REFRACTARIO_MS * 1000LL)) {
        return;
    }
    s_t_ultimo_contado = ahora;

    if (!s_evento_activo) {
        s_evento_activo  = true;
        s_t_primer_golpe = ahora;
        s_golpes_totales = 0;
        s_nivel_actual   = NIVEL_NINGUNO;
        s_nivel_maximo   = NIVEL_NINGUNO;
        s_pico_lsb = 0;
        ventana_reset();
        printf("\nInicio del terremoto\n");
    }

    s_golpes_totales++;
    s_t_ultimo_golpe = ahora;

    ventana_agregar(ahora);
    int n = ventana_contar(ahora);

    nivel_t nivel = clasificar(n);

    //Subió de nivel
    if (nivel > s_nivel_actual) {
        s_nivel_actual = nivel;
        if (nivel > s_nivel_maximo) {
            s_nivel_maximo = nivel;
        }
        indicar_nivel(nivel);
        printf("  NIVEL %s  (%d golpes en los ultimos %d s)\n",
               NOMBRE_NIVEL[nivel], n, VENTANA_MS / 1000);
    } else {
        printf("golpe #%" PRIu32 "  (%d en ventana)",
               s_golpes_totales, n);
        printf("  [%" PRIu32 " mg]", intensidad_mg);
        printf("\n");
    }

    (void)intensidad_mg;
}

//Pasan 10 segundos sin noticias para terminar
static void verificar_cierre(int64_t ahora)
{
    if (!s_evento_activo) {
        return;
    }
    if ((ahora - s_t_ultimo_golpe) <= VENTANA_US) {
        return;
    }

    int64_t duracion_ms = (s_t_ultimo_golpe - s_t_primer_golpe) / 1000;

    printf("FIN DEL TERREMOTO\n");
    printf("Duracion total: %" PRId64 " ms  (%.1f s)\n",
           duracion_ms, duracion_ms / 1000.0);
    printf("Golpes totales: %" PRIu32 "\n", s_golpes_totales);
    printf("Nivel maximo: %s\n", NOMBRE_NIVEL[s_nivel_maximo]);
    printf("Pico medido: %" PRIu32 " mg\n",
           (s_pico_lsb * 125u) >> 11);

    //Volver a vigilar
    s_evento_activo = false;
    s_golpes_totales = 0;
    s_nivel_actual = NIVEL_NINGUNO;
    s_nivel_maximo = NIVEL_NINGUNO;
    s_t_ultimo_contado = 0;
    ventana_reset();
    indicar_nivel(NIVEL_NINGUNO);

    printf("VIGILANDO MOVIMIENTOS\n");
}

static void tarea_sismo(void *arg)
{
    accel_raw_t m;
    int32_t base_x = 0, base_y = 0, base_z = 0;
    bool primera = true;
    uint32_t calib  = 0;
    ESP_LOGI(TAG, "Calibrando...");

    for (;;) {
        uint32_t avisos = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        int64_t  ahora  = esp_timer_get_time();

        if (avisos > 0) {
            //leemos y calculamos si supera el umbral
            if (mpu_read_accel(&m) == ESP_OK) {

                if (primera) {
                    base_x = (int32_t)m.ax << K_BASE;
                    base_y = (int32_t)m.ay << K_BASE;
                    base_z = (int32_t)m.az << K_BASE;
                    primera = false;
                }

                // filtro pasaaltos: le sacamos la gravedad
                base_x += (((int32_t)m.ax << K_BASE) - base_x) >> K_BASE;
                base_y += (((int32_t)m.ay << K_BASE) - base_y) >> K_BASE;
                base_z += (((int32_t)m.az << K_BASE) - base_z) >> K_BASE;

                int32_t hx = (int32_t)m.ax - (base_x >> K_BASE);
                int32_t hy = (int32_t)m.ay - (base_y >> K_BASE);
                int32_t hz = (int32_t)m.az - (base_z >> K_BASE);

                uint32_t mag2 = asm_mag2(hx, hy, hz);
                uint32_t mag  = asm_isqrt(mag2);
                uint32_t mg   = (mag * 125u) >> 11;

                if (calib < MUESTRAS_CALIB) {
                    calib++;
                    if (calib == MUESTRAS_CALIB) {
                        printf("\nVIGILANDO MOVIMIENTOS\n");
                        printf("Umbral de golpe: %d mg | ventana: %d s | "
                               "medio >= %d golpes | alto >= %d golpes\n\n",
                               UMBRAL_GOLPE_MG, VENTANA_MS / 1000,
                               N_PARA_MEDIO, N_PARA_ALTO);
                    }
                } else if (mg >= UMBRAL_GOLPE_MG) {
                    s_pico_lsb = asm_max_u32(s_pico_lsb, mag);  /* ASSEMBLER */
                    registrar_golpe(ahora, mg);
                }
            }
        }
        verificar_cierre(ahora);
    }
}

void app_main(void)
{
    printf(" SISTEMA DE DETECCION DE TERREMOTO\n");

    init_leds();

    if (mpu_init() != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar el sensor.");
        return;
    }

    xTaskCreate(tarea_sismo, "sismo", 4096, NULL, 10, &s_tarea);

    gpio_config_t cfg_int = {
        .pin_bit_mask = (1ULL << PIN_SENSOR),
        .mode = GPIO_MODE_INPUT,

        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg_int));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_SENSOR, isr_sensor, NULL));
    ESP_ERROR_CHECK(mpu_habilitar_int_dato_listo(true));
    printf("\nVIGILANDO MOVIMIENTOS\n");
    printf("Ventana: %d s | medio >= %d golpes | alto >= %d golpes\n\n",
           VENTANA_MS / 1000, N_PARA_MEDIO, N_PARA_ALTO);
    ESP_LOGI(TAG, "Interrupciones activas en GPIO%d", PIN_SENSOR);
}