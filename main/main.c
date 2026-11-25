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

#define PIN_AVISO 4

#define TIEMPO_VENTANA_MS 10000
#define TIEMPO_VENTANA_US (TIEMPO_VENTANA_MS * 1000LL)
#define MAXIMO_GOLPES 64
#define ESPERA_ENTRE_GOLPES_MS 120
#define MINIMO_GOLPE_MG 40
#define LECTURAS_INICIALES 200
#define SUAVIZADO 6

#define GOLPES_MEDIO 10
#define GOLPES_ALTO 25

static const char *ETIQUETA = "sismo";

typedef enum {
    NIVEL_NADA = 0,
    NIVEL_BAJO,
    NIVEL_MEDIO,
    NIVEL_ALTO
} nivel_sismo_t;

static const char *TEXTO_NIVEL[] = { "-", "BAJO", "MEDIO", "ALTO" };

static TaskHandle_t tarea_principal;

static int64_t lista_tiempos[MAXIMO_GOLPES];
static int primero  = 0;
static int cantidad = 0;

static bool hay_terremoto = false;
static int64_t hora_inicio = 0;
static int64_t hora_ultimo_golpe = 0;
static uint32_t total_golpes = 0;
static nivel_sismo_t nivel_actual = NIVEL_NADA;
static nivel_sismo_t nivel_maximo = NIVEL_NADA;
static int64_t hora_ultimo_contado = 0;
static uint32_t golpe_mas_fuerte = 0;

static void vaciar_lista(void)
{
    primero = 0;
    cantidad = 0;
}

static void agregar_golpe(int64_t hora)
{
    if (cantidad == MAXIMO_GOLPES) {
        primero = (primero + 1) % MAXIMO_GOLPES;
        cantidad--;
    }
    lista_tiempos[(primero + cantidad) % MAXIMO_GOLPES] = hora;
    cantidad++;
}

static int contar_golpes_recientes(int64_t ahora)
{
    while (cantidad > 0 && (ahora - lista_tiempos[primero]) > TIEMPO_VENTANA_US) {
        primero = (primero + 1) % MAXIMO_GOLPES;
        cantidad--;
    }
    return cantidad;
}

static nivel_sismo_t elegir_nivel(int golpes)
{
    if (golpes >= GOLPES_ALTO) return NIVEL_ALTO;
    if (golpes >= GOLPES_MEDIO) return NIVEL_MEDIO;
    if (golpes >= 1) return NIVEL_BAJO;
    return NIVEL_NADA;
}

static void IRAM_ATTR aviso_del_sensor(void *arg)
{
    BaseType_t cambiar_tarea = pdFALSE;
    vTaskNotifyGiveFromISR(tarea_principal, &cambiar_tarea);
    portYIELD_FROM_ISR(cambiar_tarea);
}

static void anotar_golpe(int64_t ahora, uint32_t fuerza_mg)
{
    if (hora_ultimo_contado != 0 &&
        (ahora - hora_ultimo_contado) < (ESPERA_ENTRE_GOLPES_MS * 1000LL)) {
        return;
    }
    hora_ultimo_contado = ahora;

    if (!hay_terremoto) {
        hay_terremoto = true;
        hora_inicio = ahora;
        total_golpes = 0;
        nivel_actual = NIVEL_NADA;
        nivel_maximo = NIVEL_NADA;
        golpe_mas_fuerte = 0;
        vaciar_lista();
        printf("\nInicio del terremoto\n");
    }

    total_golpes++;
    hora_ultimo_golpe = ahora;

    agregar_golpe(ahora);
    int recientes = contar_golpes_recientes(ahora);

    nivel_sismo_t nivel = elegir_nivel(recientes);

    if (nivel > nivel_actual) {
        nivel_actual = nivel;
        if (nivel > nivel_maximo) {
            nivel_maximo = nivel;
        }
        printf("  NIVEL %s  (%d golpes en los ultimos %d s)\n",
               TEXTO_NIVEL[nivel], recientes, TIEMPO_VENTANA_MS / 1000);
    } else {
        printf("golpe #%" PRIu32 "  (%d en ventana)",
               total_golpes, recientes);
        printf("  [%" PRIu32 " mg]", fuerza_mg);
        printf("\n");
    }
}

static void revisar_si_termino(int64_t ahora)
{
    if (!hay_terremoto) {
        return;
    }
    if ((ahora - hora_ultimo_golpe) <= TIEMPO_VENTANA_US) {
        return;
    }

    int64_t duracion_ms = (hora_ultimo_golpe - hora_inicio) / 1000;

    printf("FIN DEL TERREMOTO\n");
    printf("Duracion total: %" PRId64 " ms  (%.1f s)\n",
           duracion_ms, duracion_ms / 1000.0);
    printf("Golpes totales: %" PRIu32 "\n", total_golpes);
    printf("Nivel maximo: %s\n", TEXTO_NIVEL[nivel_maximo]);
    printf("Pico medido: %" PRIu32 " mg\n",
           (golpe_mas_fuerte * 125u) >> 11);

    hay_terremoto = false;
    total_golpes = 0;
    nivel_actual = NIVEL_NADA;
    nivel_maximo = NIVEL_NADA;
    hora_ultimo_contado = 0;
    vaciar_lista();

    printf("VIGILANDO MOVIMIENTOS\n");
    printf("Contador de eventos: %lu\n", (unsigned long)total_golpes);
}

static void vigilar_sismo(void *arg)
{
    lectura_t lectura;
    int32_t promedio_x = 0, promedio_y = 0, promedio_z = 0;
    bool es_la_primera = true;
    uint32_t contador_inicio = 0;
    ESP_LOGI(ETIQUETA, "Calibrando...");

    for (;;) {
        uint32_t avisos = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        int64_t  ahora  = esp_timer_get_time();

        if (avisos > 0) {
            if (leer_aceleracion(&lectura) == ESP_OK) {

                if (es_la_primera) {
                    promedio_x = (int32_t)lectura.x << SUAVIZADO;
                    promedio_y = (int32_t)lectura.y << SUAVIZADO;
                    promedio_z = (int32_t)lectura.z << SUAVIZADO;
                    es_la_primera = false;
                }

                promedio_x += (((int32_t)lectura.x << SUAVIZADO) - promedio_x) >> SUAVIZADO;
                promedio_y += (((int32_t)lectura.y << SUAVIZADO) - promedio_y) >> SUAVIZADO;
                promedio_z += (((int32_t)lectura.z << SUAVIZADO) - promedio_z) >> SUAVIZADO;

                int32_t mov_x = (int32_t)lectura.x - (promedio_x >> SUAVIZADO);
                int32_t mov_y = (int32_t)lectura.y - (promedio_y >> SUAVIZADO);
                int32_t mov_z = (int32_t)lectura.z - (promedio_z >> SUAVIZADO);

                uint32_t suma      = sumar_cuadrados(mov_x, mov_y, mov_z);
                uint32_t fuerza    = raiz_cuadrada(suma);
                uint32_t fuerza_mg = (fuerza * 125u) >> 11;

                if (contador_inicio < LECTURAS_INICIALES) {
                    contador_inicio++;
                    if (contador_inicio == LECTURAS_INICIALES) {
                        printf("\nVIGILANDO MOVIMIENTOS\n");
                        printf("\nContador de eventos: %lu\n", (unsigned long)total_golpes);
                        printf("Umbral de golpe: %d mg | ventana: %d s | "
                               "medio >= %d golpes | alto >= %d golpes\n\n",
                               MINIMO_GOLPE_MG, TIEMPO_VENTANA_MS / 1000,
                               GOLPES_MEDIO, GOLPES_ALTO);
                    }
                } else if (fuerza_mg >= MINIMO_GOLPE_MG) {
                    golpe_mas_fuerte = elegir_mayor(golpe_mas_fuerte, fuerza);
                    anotar_golpe(ahora, fuerza_mg);
                }
            }
        }
        revisar_si_termino(ahora);
    }
}

void app_main(void)
{
    printf(" SISTEMA DE DETECCION DE TERREMOTO\n");

    if (iniciar_mpu() != ESP_OK) {
        ESP_LOGE(ETIQUETA, "No se pudo inicializar el sensor.");
        return;
    }

    xTaskCreate(vigilar_sismo, "sismo", 4096, NULL, 10, &tarea_principal);

    gpio_config_t config_pin = {
        .pin_bit_mask = (1ULL << PIN_AVISO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&config_pin));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_AVISO, aviso_del_sensor, NULL));
    ESP_ERROR_CHECK(activar_aviso_mpu(true));
    printf("Ventana: %d s | medio >= %d golpes | alto >= %d golpes\n\n",
           TIEMPO_VENTANA_MS / 1000, GOLPES_MEDIO, GOLPES_ALTO);
    ESP_LOGI(ETIQUETA, "Interrupciones activas en GPIO%d", PIN_AVISO);
}
