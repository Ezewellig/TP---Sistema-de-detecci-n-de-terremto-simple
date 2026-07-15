#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_err.h"
#define SENSOR_PIN 4
#define LED_VERDE 5
#define LED_AMARILLO 6
#define LED_ROJO 7
//definimos el maximo de eventos que podemos almacenar en el buffer de golpes
#define MAX_EVENTOS 100
//ventana de tiempo de 10 segundos para contar los golpes
#define VENTANA_MS 10000


//Variables volatiles porque se modifican dentro de la interrupcion.
volatile uint64_t golpes[MAX_EVENTOS];
volatile int cantidad_golpes = 0;
volatile int golpesTotalesEvento = 0;

//iniciamos el evento en falso y ponemos todo en 0
volatile bool eventoActivo = false; //preguntar si poner 1 verdaero y 0 falso o con bool
volatile uint64_t inicioEvento = 0;
volatile int nivelMaximo = 0;

//declaracion de la funcion en Assembler RISC-V
extern int calcular_nivel_sismo(int golpes);
//declaracion de la funcion para actualizar los leds
void actualizar_leds(int nivel);
void reportar_nivel(int nivel);
void actualizarVentana();
//interrupcion ISR
//IRAM_ATTR guarda esta funcion en la memoria RAM para que sea mas rapida y no se quede sin memoria
// y no en la memoria flash que es mas lenta y puede quedarse sin memoria
void IRAM_ATTR ISR_sensor(void* arg){ 
    static uint64_t ultima_ISR = 0;
    uint64_t tiempo_Ahora = esp_timer_get_time()/1000; //obtenemos el tiempo en milisegundos

    //si el tiempo transcurrido desde la ultima interrupcion es menor a 50ms,
    // entonces no hacemos nada para evitar saturar el buffer de golpes
    if(tiempo_Ahora-ultima_ISR<50)
        return;

    //guardamos el tiempo de la ultima interrupcion para no saturar el buffer de golpes
    ultima_ISR=tiempo_Ahora;
    if(cantidad_golpes<MAX_EVENTOS)
    {
        golpes[cantidad_golpes]=tiempo_Ahora;
        cantidad_golpes++; //sumamos un golpe al buffer de golpes
        golpesTotalesEvento++; //sumo el total de golpes del evento para reportarlo al final del mismo
    }
}
///en ESP-IDF no hay setup() ni loop() como en Arduino, 
//pero podemos usar estas funciones para organizar 
//nuestro codigo y luego llamarlas desde app_main()

void app_main(void)
{
    //configuracion de los pines de los leds como salida
    gpio_set_direction(LED_VERDE, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_AMARILLO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_ROJO, GPIO_MODE_OUTPUT);
    //configuracion del pin del sensor como entrada
    gpio_set_direction(SENSOR_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SENSOR_PIN, GPIO_PULLUP_ONLY);
    //configurar interrupcion para flanco descendente (FALLING) en el pin del sensor
    gpio_set_intr_type(SENSOR_PIN, GPIO_INTR_NEGEDGE);
    //instalar el servicio de interrupciones y registrar la ISR para el pin del sensor
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SENSOR_PIN, ISR_sensor, NULL);
    printf("\nSistema de vigilancia activo\n");
    printf("\nEsperando vibraciones...\n");
    actualizar_leds(0);

    
    //el equivalente al loop() de Arduino es un while(1) en ESP-IDF, que se ejecuta indefinidamente
    while(1)
    {
        actualizarVentana();
        if(cantidad_golpes>0)
        { 
            //si hay golpes dentro de la ventana de tiempo, entonces hay un evento activo
            if(!eventoActivo)
            { 
                //si no hay un evento activo, entonces iniciamos uno
                eventoActivo=true;
                inicioEvento=golpes[0];

                printf("\nEVENTO DETECTADO\n");

            }
            //llamamos la funcon en Assembler para calcular el nivel del sismo
            int nivel=calcular_nivel_sismo(cantidad_golpes);
            //si el nivel calculado es mayor al nivel maximo registrado, entonces actualizamos el nivel maximo y reportamos el nivel
            if(nivel>nivelMaximo)
            {
                //actualizamos el nivel maximo registrado
                nivelMaximo=nivel;
                //Reportamos el nivel del sismo
                reportar_nivel(nivel);
                //actualizamos los leds para mostrar el nivel del sismo
                actualizar_leds(nivel);}
         }
        else
        {
            //si no hay golpes dentro de la ventana de tiempo, entonces el evento ha terminado
            if(eventoActivo)
            {
                //si hay un evento activo, entonces lo finalizamos
                eventoActivo=false;
                //reportamos el fin del evento y la duracion del mismo
                printf("\nFIN DEL EVENTO\n");
                printf("\nDuracion(ms): %llu\n",(esp_timer_get_time()/1000)-inicioEvento);
                printf("\nCantidad de golpes totales: %d\n",golpesTotalesEvento); 
                printf("\nNivel maximo: ");
                reportar_nivel(nivelMaximo);
                //actualizamos el nivel maximo a 0 para el siguiente evento
                nivelMaximo=0;
                golpesTotalesEvento=0;
                //actualizamos los leds para mostrar que no hay evento activo
                actualizar_leds(0);

                printf("\nSistema nuevamente en vigilancia.\n");

            }

        }
        vTaskDelay(100 / portTICK_PERIOD_MS); //delay de 100ms para no saturar el procesador
    }
}



//actualizamos ventana de tiempo para contar los golpes y eliminar los que ya no estan dentro de la ventana
void actualizarVentana()
{
    uint64_t tiempo_Ahora = esp_timer_get_time()/1000;
    int i=0;

    while(i<cantidad_golpes)
    {
       if(tiempo_Ahora-golpes[i]<=VENTANA_MS)
            break;

        i++;
    }

    if(i>0)
    {
        //elimino los golpes que ya no estan dentro de la ventana de tiempo
        for(int j=0; j<cantidad_golpes - i; j++)
            golpes[j]=golpes[j+i];
        cantidad_golpes-=i;
    }

}

void reportar_nivel(int nivel)
{
    //reportamos el nivel del sismo por el puerto 
    printf("\nNivel: ");
    //usamos un switch para reportar el nivel del sismo
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
    //actualizamos los leds para mostrar el nivel del sismo
    //gpio_set_level recibe el pin y un estado (0 o 1) para encender o apagar el led
    gpio_set_level(LED_VERDE, (nivel==0||nivel==1) ? 1 : 0);
    gpio_set_level(LED_AMARILLO, (nivel==2) ? 1 : 0);
    gpio_set_level(LED_ROJO, (nivel==3) ? 1 : 0);
}

