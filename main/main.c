/* =====================================================================
 *  SISTEMA DE DETECCION DE TERREMOTO POR UMBRAL
 *  ESP32-C3 SuperMini
 *
 *  Implementa la consigna del Proyecto 26:
 *
 *   - Arranca en estado de VIGILANCIA y lo informa por terminal, con el
 *     contador de eventos en cero.
 *   - Espera interrupciones externas. El bucle principal NO hace polling:
 *     la tarea queda bloqueada hasta que la ISR la despierta.
 *   - Cuenta "golpes" dentro de una VENTANA DESLIZANTE de 10 segundos.
 *   - Clasifica en BAJO / MEDIO / ALTO segun CUANTOS golpes hay en esa
 *     ventana, e informa el nivel APENAS lo alcanza, sin esperar a que
 *     la ventana termine.
 *   - Si siguen llegando golpes y la frecuencia sube, el nivel ESCALA
 *     (bajo -> medio -> alto).
 *   - Cuando pasan 10 s sin ningun golpe nuevo, cierra el episodio e
 *     informa duracion total, golpes totales y nivel maximo alcanzado.
 *     Despues vuelve a vigilancia con el contador en cero.
 *
 *  Funciona con DOS sensores distintos, elegidos con USAR_HW139:
 *
 *    USAR_HW139 = 1 -> HW-139 (interruptor de vibracion, salida digital).
 *                      Cada pulso del sensor es un golpe. No usa I2C.
 *    USAR_HW139 = 0 -> MPU-6050 / MPU-9250 por I2C. La interrupcion
 *                      "dato listo" llega a 100 Hz y se considera golpe
 *                      cada vez que la aceleracion supera un umbral.
 *                      Ademas mide la intensidad real en mili-g.
 * ===================================================================*/
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sismo_math.h"

/* =====================================================================
 *  ELECCION DEL SENSOR
 * ===================================================================*/
#define USAR_HW139          0       /* 1 = HW-139 ; 0 = MPU por I2C */

#if !USAR_HW139
#include "mpu6050.h"
#endif

/* =====================================================================
 *  PARAMETROS AJUSTABLES
 * ===================================================================*/
#define PIN_SENSOR          4       /* HW-139 DO, o INT del MPU */

/* --- ventana deslizante --- */
#define VENTANA_MS          10000                   /* 10 segundos      */
#define VENTANA_US          (VENTANA_MS * 1000LL)
#define MAX_GOLPES          64      /* capacidad del buffer circular    */

/* --- que cuenta como un golpe --- */
#define REFRACTARIO_MS      120     /* antirrebote: despues de un golpe,
                                       ignoramos el sensor este tiempo.
                                       Sin esto, una sola sacudida se
                                       contaria como veinte golpes.     */
#if !USAR_HW139
#define UMBRAL_GOLPE_MG     40      /* aceleracion minima para contar   */
#define MUESTRAS_CALIB      200     /* 2 s aprendiendo la gravedad      */
#define K_BASE              6       /* filtro pasaaltos: tau = 0.64 s   */
#endif

/* --- clasificacion por CANTIDAD de golpes en la ventana --- */
#define N_PARA_MEDIO        4       /*  4..9 golpes  -> MEDIO */
#define N_PARA_ALTO         10      /* >=10 golpes   -> ALTO  */
                                    /*  1..3 golpes  -> BAJO  */

/* ---- LEDs: preparado para el futuro, apagado por ahora ---- */
#define USAR_LEDS           0
#define PIN_LED_VERDE       0
#define PIN_LED_AMARILLO    1
#define PIN_LED_ROJO        3

static const char *TAG = "sismo";

/* =====================================================================
 *  TIPOS Y ESTADO
 * ===================================================================*/
typedef enum {
    NIVEL_NINGUNO = 0,
    NIVEL_BAJO,
    NIVEL_MEDIO,
    NIVEL_ALTO
} nivel_t;

static const char *NOMBRE_NIVEL[] = { "-", "BAJO", "MEDIO", "ALTO" };

static TaskHandle_t s_tarea;

/* --- ventana deslizante: cola FIFO circular de marcas de tiempo --- */
static int64_t  s_cola[MAX_GOLPES];
static int      s_ini  = 0;     /* indice del golpe mas viejo */
static int      s_cant = 0;     /* cuantos hay guardados      */

/* --- episodio sismico en curso --- */
static bool     s_evento_activo   = false;
static int64_t  s_t_primer_golpe  = 0;
static int64_t  s_t_ultimo_golpe  = 0;
static uint32_t s_golpes_totales  = 0;
static nivel_t  s_nivel_actual    = NIVEL_NINGUNO;
static nivel_t  s_nivel_maximo    = NIVEL_NINGUNO;
static int64_t  s_t_ultimo_contado = 0;   /* para el refractario */
#if !USAR_HW139
static uint32_t s_pico_lsb        = 0;    /* dato extra de intensidad */
#endif

/* =====================================================================
 *  VENTANA DESLIZANTE
 *
 *  Guardamos la marca de tiempo de cada golpe en una cola circular.
 *  "Deslizante" significa que en todo momento la ventana son los
 *  ultimos 10 segundos contados hacia atras desde AHORA: por eso antes
 *  de contar, tiramos a la basura los golpes que ya quedaron viejos.
 * ===================================================================*/
static void ventana_reset(void)
{
    s_ini = 0;
    s_cant = 0;
}

static void ventana_agregar(int64_t t)
{
    if (s_cant == MAX_GOLPES) {          /* lleno: pisamos el mas viejo */
        s_ini = (s_ini + 1) % MAX_GOLPES;
        s_cant--;
    }
    s_cola[(s_ini + s_cant) % MAX_GOLPES] = t;
    s_cant++;
}

/* Descarta lo que quedo fuera de la ventana y devuelve cuantos quedan. */
static int ventana_contar(int64_t ahora)
{
    while (s_cant > 0 && (ahora - s_cola[s_ini]) > VENTANA_US) {
        s_ini = (s_ini + 1) % MAX_GOLPES;
        s_cant--;
    }
    return s_cant;
}

/* =====================================================================
 *  CLASIFICACION: depende de CUANTOS golpes hay en la ventana
 * ===================================================================*/
static nivel_t clasificar(int golpes_en_ventana)
{
    if (golpes_en_ventana >= N_PARA_ALTO)  return NIVEL_ALTO;
    if (golpes_en_ventana >= N_PARA_MEDIO) return NIVEL_MEDIO;
    if (golpes_en_ventana >= 1)            return NIVEL_BAJO;
    return NIVEL_NINGUNO;
}

/* =====================================================================
 *  INTERRUPCION
 *
 *  Lo unico que hace es despertar a la tarea. Nada de I2C, printf ni
 *  delays acá adentro.
 * ===================================================================*/
static void IRAM_ATTR isr_sensor(void *arg)
{
    BaseType_t hay_tarea_mas_prioritaria = pdFALSE;
    vTaskNotifyGiveFromISR(s_tarea, &hay_tarea_mas_prioritaria);
    portYIELD_FROM_ISR(hay_tarea_mas_prioritaria);
}

/* =====================================================================
 *  INDICADORES (consola hoy, LEDs manana)
 * ===================================================================*/
static void indicar_nivel(nivel_t n)
{
#if USAR_LEDS
    gpio_set_level(PIN_LED_VERDE,    n == NIVEL_BAJO);
    gpio_set_level(PIN_LED_AMARILLO, n == NIVEL_MEDIO);
    gpio_set_level(PIN_LED_ROJO,     n == NIVEL_ALTO);
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

/* =====================================================================
 *  REGISTRAR UN GOLPE
 *
 *  Se llama cada vez que el sensor dice "hubo vibracion". Actualiza la
 *  ventana, decide el nivel y lo informa SOLO si subio (escalado).
 * ===================================================================*/
static void registrar_golpe(int64_t ahora, uint32_t intensidad_mg)
{
    /* --- antirrebote: una sacudida = un golpe --- */
    if (s_t_ultimo_contado != 0 &&
        (ahora - s_t_ultimo_contado) < (REFRACTARIO_MS * 1000LL)) {
        return;
    }
    s_t_ultimo_contado = ahora;

    /* --- primer golpe: arranca el episodio --- */
    if (!s_evento_activo) {
        s_evento_activo  = true;
        s_t_primer_golpe = ahora;
        s_golpes_totales = 0;
        s_nivel_actual   = NIVEL_NINGUNO;
        s_nivel_maximo   = NIVEL_NINGUNO;
#if !USAR_HW139
        s_pico_lsb = 0;
#endif
        ventana_reset();
        printf("\n--- INICIO DE EVENTO SISMICO ---\n");
    }

    s_golpes_totales++;
    s_t_ultimo_golpe = ahora;

    ventana_agregar(ahora);
    int n = ventana_contar(ahora);

    nivel_t nivel = clasificar(n);

    /* --- reporte inmediato, solo cuando el nivel ESCALA --- */
    if (nivel > s_nivel_actual) {
        s_nivel_actual = nivel;
        if (nivel > s_nivel_maximo) {
            s_nivel_maximo = nivel;
        }
        indicar_nivel(nivel);
        printf("  >> NIVEL %s  (%d golpes en los ultimos %d s)\n",
               NOMBRE_NIVEL[nivel], n, VENTANA_MS / 1000);
    } else {
        printf("     golpe #%" PRIu32 "  (%d en ventana)",
               s_golpes_totales, n);
#if !USAR_HW139
        printf("  [%" PRIu32 " mg]", intensidad_mg);
#endif
        printf("\n");
    }

    (void)intensidad_mg;
}

/* =====================================================================
 *  CIERRE DEL EPISODIO
 *
 *  Si pasaron 10 s desde el ultimo golpe sin novedades, el episodio
 *  termino: informamos el resumen y volvemos a vigilancia.
 * ===================================================================*/
static void verificar_cierre(int64_t ahora)
{
    if (!s_evento_activo) {
        return;
    }
    if ((ahora - s_t_ultimo_golpe) <= VENTANA_US) {
        return;
    }

    int64_t duracion_ms = (s_t_ultimo_golpe - s_t_primer_golpe) / 1000;

    printf("\n=========================================\n");
    printf("      CIERRE DEL EVENTO SISMICO\n");
    printf("=========================================\n");
    printf(" Duracion total   : %" PRId64 " ms  (%.1f s)\n",
           duracion_ms, duracion_ms / 1000.0);
    printf(" Golpes totales   : %" PRIu32 "\n", s_golpes_totales);
    printf(" Nivel maximo     : %s\n", NOMBRE_NIVEL[s_nivel_maximo]);
#if !USAR_HW139
    printf(" Pico medido      : %" PRIu32 " mg\n",
           (s_pico_lsb * 125u) >> 11);
#endif
    printf("=========================================\n");

    /* --- vuelta a vigilancia, contador en cero --- */
    s_evento_activo    = false;
    s_golpes_totales   = 0;
    s_nivel_actual     = NIVEL_NINGUNO;
    s_nivel_maximo     = NIVEL_NINGUNO;
    s_t_ultimo_contado = 0;
    ventana_reset();
    indicar_nivel(NIVEL_NINGUNO);

    printf("Sensor activo. En vigilancia. Contador de eventos: 0\n\n");
}

/* =====================================================================
 *  TAREA PRINCIPAL
 * ===================================================================*/
static void tarea_sismo(void *arg)
{
#if !USAR_HW139
    accel_raw_t m;
    int32_t base_x = 0, base_y = 0, base_z = 0;
    bool    primera = true;
    uint32_t calib  = 0;

    ESP_LOGI(TAG, "Calibrando... mantene el sensor quieto.");
#endif

    for (;;) {
        /* Bloqueados hasta que la ISR avise. El timeout de 100 ms NO es
         * polling del sensor: solo sirve para poder cerrar el episodio
         * cuando dejan de llegar golpes.                              */
        uint32_t avisos = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        int64_t  ahora  = esp_timer_get_time();

        if (avisos > 0) {
#if USAR_HW139
            /* --- HW-139: cada interrupcion YA es un golpe --- */
            registrar_golpe(ahora, 0);
#else
            /* --- MPU: leemos y decidimos si supera el umbral --- */
            if (mpu_read_accel(&m) == ESP_OK) {

                if (primera) {
                    base_x = (int32_t)m.ax << K_BASE;
                    base_y = (int32_t)m.ay << K_BASE;
                    base_z = (int32_t)m.az << K_BASE;
                    primera = false;
                }

                /* filtro pasaaltos: le sacamos la gravedad */
                base_x += (((int32_t)m.ax << K_BASE) - base_x) >> K_BASE;
                base_y += (((int32_t)m.ay << K_BASE) - base_y) >> K_BASE;
                base_z += (((int32_t)m.az << K_BASE) - base_z) >> K_BASE;

                int32_t hx = (int32_t)m.ax - (base_x >> K_BASE);
                int32_t hy = (int32_t)m.ay - (base_y >> K_BASE);
                int32_t hz = (int32_t)m.az - (base_z >> K_BASE);

                /* modulo del vector, en ASSEMBLER */
                uint32_t mag2 = asm_mag2(hx, hy, hz);
                uint32_t mag  = asm_isqrt(mag2);
                uint32_t mg   = (mag * 125u) >> 11;

                if (calib < MUESTRAS_CALIB) {
                    calib++;
                    if (calib == MUESTRAS_CALIB) {
                        printf("\nSensor activo. En vigilancia. "
                               "Contador de eventos: 0\n");
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
#endif
        }

        /* Con golpe o sin golpe, siempre miramos si hay que cerrar. */
        verificar_cierre(ahora);
    }
}

/* =====================================================================
 *  app_main
 * ===================================================================*/
void app_main(void)
{
    printf("\n=========================================\n");
    printf(" SISTEMA DE DETECCION DE TERREMOTO\n");
    printf(" ESP32-C3 + %s\n", USAR_HW139 ? "HW-139" : "MPU-6050/9250");
    printf("=========================================\n");

    init_leds();

#if !USAR_HW139
    /* --- sensor I2C primero, con su INT todavia apagada --- */
    if (mpu_init() != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar el sensor. Revisa el cableado.");
        return;
    }
#endif

    /* --- la tarea que procesa los golpes --- */
    xTaskCreate(tarea_sismo, "sismo", 4096, NULL, 10, &s_tarea);

    /* --- el pin de interrupcion --- */
    gpio_config_t cfg_int = {
        .pin_bit_mask = (1ULL << PIN_SENSOR),
        .mode         = GPIO_MODE_INPUT,
#if USAR_HW139
        /* El HW-139 con LM393 tiene la salida en alto en reposo y la
         * baja al vibrar. Si tu modulo hace lo contrario, cambia a
         * GPIO_INTR_POSEDGE y pull-down.                              */
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
#else
        /* El MPU pulsa el INT en alto durante 50 us. */
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
#endif
    };
    ESP_ERROR_CHECK(gpio_config(&cfg_int));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_SENSOR, isr_sensor, NULL));

#if !USAR_HW139
    /* --- recien ahora habilitamos la INT del sensor --- */
    ESP_ERROR_CHECK(mpu_habilitar_int_dato_listo(true));
#else
    printf("\nSensor activo. En vigilancia. Contador de eventos: 0\n");
    printf("Ventana: %d s | medio >= %d golpes | alto >= %d golpes\n\n",
           VENTANA_MS / 1000, N_PARA_MEDIO, N_PARA_ALTO);
#endif

    ESP_LOGI(TAG, "Interrupciones activas en GPIO%d", PIN_SENSOR);
}