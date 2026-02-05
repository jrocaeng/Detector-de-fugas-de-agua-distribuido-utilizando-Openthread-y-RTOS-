/*
 codigo fuente de un cliente de una red Thread que detecta agua y envia un mensaje UDP a un border router
*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_openthread.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "esp_ot_config.h"
#include "esp_vfs_eventfd.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "nvs_flash.h"
#include "openthread/cli.h"
#include "openthread/instance.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"
#include "openthread/udp.h"
#include "openthread/message.h"
#include "openthread/ip6.h"

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
#include "ot_led_strip.h"
#endif

#define WATER_SENSOR_GPIO 4  
#define LECTURAS_RAPIDAS 5  // Número de lecturas consecutivas para confirmar
#define DELAY_LECTURAS 100  // Delay en ms que he puesto entre lecturas rapidas

static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif != NULL);
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));

    return netif;
}

static void tarea_inicio_Openthread(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    // Función que inicializa la pila OpenThread
    ESP_ERROR_CHECK(esp_openthread_init(&config));

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
    ESP_ERROR_CHECK(esp_openthread_state_indicator_init(esp_openthread_get_instance()));
#endif

#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif
    
    esp_netif_t *openthread_netif;
    
    openthread_netif = init_openthread_netif(&config);  // inicia la interfaz de red de OpenThread
    esp_netif_set_default_netif(openthread_netif);

#if CONFIG_OPENTHREAD_AUTO_START
    otOperationalDatasetTlvs dataset;
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset); // obtiene el dataset activo de la red Thread colocada
    ESP_ERROR_CHECK(esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
#endif

    esp_openthread_launch_mainloop(); // lanzamos el bucle principal de OpenThread

    esp_openthread_netif_glue_deinit(); 
    esp_netif_destroy(openthread_netif);

    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL); 
}

static void tarea_alerta(void *arg)
{
    // Configuracion GPIO para el sensor de agua 
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WATER_SENSOR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    
    otInstance *instancia = NULL;

    // Esperamos a que la istancia OpenThread se inicialice para abrir el socket UDP
    while ((instancia = esp_openthread_get_instance()) == NULL) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    otUdpSocket socket;
    otError error = otUdpOpen(instancia, &socket, NULL, NULL); 
    
    if (error != OT_ERROR_NONE) {
        ESP_LOGE("UDP_SEND", "Error abriendo socket: %d", error);
        vTaskDelete(NULL);
        return;
    }

    static const char *BR_ADDR = "fd00:db8:a0:0:b422:1bc0:5b22:50ea"; // Dirección IPv6 del border router
    otIp6Address destAddr;
    otIp6AddressFromString(BR_ADDR, &destAddr);

    otMessageInfo msgInfo = {0}; 
    msgInfo.mPeerAddr = destAddr; // Direccion IPv6 del border router
    msgInfo.mPeerPort = 1234;

    bool fuga_confirmada_prev = false; // para avisar solo con el flanco de subida
    
    //he hecho polling por sencillez, pero para ganar en uso de energia intentaré cambiarlo a interrupciones para antes de la presentacion

    while (1) {
        
        int lecturas_agua = 0;
        
        // Tomar múltiples lecturas consecutivas
        for (int i = 0; i < LECTURAS_RAPIDAS; i++) {

            int estado_sensor = gpio_get_level(WATER_SENSOR_GPIO);
            if (estado_sensor == 0) {  // 0 = agua detectada, nivel bajo
                lecturas_agua++;
            }

            if (i < LECTURAS_RAPIDAS - 1) {
                vTaskDelay(pdMS_TO_TICKS(DELAY_LECTURAS));
            }
        }
        
        // filtro anti-falsos positivos 
        bool fuga_confirmada = (lecturas_agua == LECTURAS_RAPIDAS);
        
        // Solo enviamos en flanco de subida
        if (fuga_confirmada && !fuga_confirmada_prev) {
            ESP_LOGW("SENSOR", "¡FUGA CONFIRMADA! (%d/%d lecturas) Enviando alarma...", 
                     lecturas_agua, LECTURAS_RAPIDAS);
            
            const char *payload = "FUGA_DETECTADA"; //mensaje udp que enviamos
            otMessage *msg = otUdpNewMessage(instancia, NULL);

            if (msg == NULL) {
                ESP_LOGE("UDP_SEND", "Error creando mensaje");

            } else {
                otMessageAppend(msg, payload, strlen(payload)); 
                error = otUdpSend(instancia, &socket, msg, &msgInfo);

                if (error == OT_ERROR_NONE) {
                    ESP_LOGI("UDP_SEND", "Alarma enviada a %s", BR_ADDR);
                } else {
                    ESP_LOGE("UDP_SEND", "Error al enviar: %d", error);
                    otMessageFree(msg);
                }
            }

        } else if (!fuga_confirmada && fuga_confirmada_prev) {
            ESP_LOGI("SENSOR", "Sensor seco de nuevo");
        }
        
        fuga_confirmada_prev = fuga_confirmada;
        
        // Espera entre ciclos
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3, //
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));
    
    xTaskCreate(tarea_inicio_Openthread, "tarea_inicio_Openthread", 10240, xTaskGetCurrentTaskHandle(), 5, NULL);
    xTaskCreate(tarea_alerta, "tarea_alerta", 4096, NULL, 5, NULL);
}
