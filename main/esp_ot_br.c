/*
 Codigo en una ESP32-S3 que actua como Border Router Openthread, recibiendo alertas UDP de la placa ESP32-H2, activando el buzzer y la pantalla y mandando una alerta MQTT a Home asisstant
 */

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_openthread.h"
#include "esp_openthread_border_router.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif
#include "esp_ot_config.h"
#include "esp_ot_wifi_cmd.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hal/uart_types.h"
#include "openthread/error.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"

#include "openthread/udp.h"
#include "openthread/message.h"
#include "openthread/ip6.h"

// Includes para LCD
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "escalado_imagen.h"

// Includes para Buzzer RMT
#include "driver/rmt_tx.h"
#include "musical_score_encoder.h"

// Include para MQTT
#include "mqtt_client.h"

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
#include "ot_led_strip.h"
#endif

#if CONFIG_OPENTHREAD_BR_AUTO_START
#include "example_common_private.h"
#include "protocol_examples_common.h"
#endif

// Estructura para los mensajes UDP recibidos
typedef struct {
    char data[64];  
    uint16_t length;
    otIp6Address peer_addr;
    uint16_t peer_port;
} udp_msg_t;

static QueueHandle_t udp_msg_queue = NULL;

// mqtt configuración
#define MQTT_BROKER_URI "mqtt://192.168.1.17"  
#define MQTT_USERNAME "mqttuser"              
#define MQTT_PASSWORD "contraseña"             
#define MQTT_TOPIC_ALERT "homeassistant/sensor/water_leak/state"
#define MQTT_TOPIC_AVAILABLE "homeassistant/sensor/water_leak/availability"


static esp_mqtt_client_handle_t mqtt_client = NULL;

// configuración LCD
#define LCD_HOST       SPI2_HOST
#define PARALLEL_LINES 8

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_DATA0          36
#define EXAMPLE_PIN_NUM_PCLK           47
#define EXAMPLE_PIN_NUM_CS             41
#define EXAMPLE_PIN_NUM_DC             39
#define EXAMPLE_PIN_NUM_RST            42
#define EXAMPLE_PIN_NUM_BK_LIGHT       1  //para encender y apagar la luz de fondo

#define EXAMPLE_LCD_H_RES              320
#define EXAMPLE_LCD_V_RES              240
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

// Variables globales de LCD
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
static uint16_t *s_lines[2];
static bool inicializacion_lcd = false;

// configuración Buzzer RMT
#define RMT_BUZZER_RESOLUTION_HZ 1000000  
#define RMT_BUZZER_GPIO_NUM      19        


// Variables globales Buzzer RMT
static rmt_channel_handle_t buzzer_chan = NULL;
static rmt_encoder_handle_t score_encoder = NULL;

// Secuencia de frecuencias agudo-gravepara la alarma 
static const buzzer_musical_score_t frequency_sweep[] = {
    {600, 2500}, {100, 500}, {600, 2500}, {100, 500}, {600, 2500}, {100, 500}, {600, 2500}, {100, 500}, {600, 2500}, {100, 500}, {600, 2500}, {100, 500}, {600, 2500}, {100, 500}, {600, 2500}, {100, 500},
};
#if !CONFIG_OPENTHREAD_BR_AUTO_START && CONFIG_EXAMPLE_CONNECT_ETHERNET
#error Currently we do not support a manual way to connect ETH, \
       if you want to use ETH, please enable OPENTHREAD_BR_AUTO_START.
#endif

#define TAG "esp_ot_br"

// funciones MQTT

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    (void)event;  
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT conectado al broker");
        // Publicamos estado disponible al conectarnos (hay conexión)
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_AVAILABLE, "online", 0, 1, 1);
        // Reseteamos alerta a OFF al conectarse 
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ALERT, "OFF", 0, 1, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT desconectado del broker");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;
    default:
        break;
    }
}

static void init_mqtt(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials = {
            .username = MQTT_USERNAME,
            .authentication.password = MQTT_PASSWORD,
        },
        .session.last_will = {   // lo que publicamos cuando nos desconectamos
            .topic = MQTT_TOPIC_AVAILABLE, //
            .msg = "offline",
            .qos = 1,
            .retain = 1,
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg); // Inicializamos el cliente MQTT
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Error al inicializar cliente MQTT");
        return;
    }

    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL); 
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI(TAG, "Cliente MQTT iniciado");
}

static void mqtt_publish_alert(void)
{
    if (mqtt_client == NULL) {
        ESP_LOGW(TAG, "Cliente MQTT no inicializado");
        return;
    }

    // Publicamos alerta de fuga 
    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ALERT, "ON", 0, 1, 0);
    if (msg_id >= 0) {
        ESP_LOGI(TAG, "Alerta MQTT publicada (msg_id=%d)", msg_id);
    } else {
        ESP_LOGE(TAG, "Error al publicar alerta MQTT");
    }
}

static void mqtt_reset_alert(void)
{
    if (mqtt_client == NULL) {
        ESP_LOGW(TAG, "Cliente MQTT no inicializado");
        return;
    }

    // Reseteamos alerta a OFF
    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ALERT, "OFF", 0, 1, 0);
    if (msg_id >= 0) {
        ESP_LOGI(TAG, "Alerta MQTT reseteada a OFF (msg_id=%d)", msg_id);
    } else {
        ESP_LOGE(TAG, "Error al resetear alerta MQTT");
    }
}

#if CONFIG_OPENTHREAD_SUPPORT_HW_RESET_RCP
#define PIN_TO_RCP_RESET CONFIG_OPENTHREAD_HW_RESET_RCP_PIN

static void rcp_failure_hardware_reset_handler(void) // Reseteo hardware del RCP
{
    gpio_config_t reset_pin_config;
    memset(&reset_pin_config, 0, sizeof(reset_pin_config));

    reset_pin_config.intr_type = GPIO_INTR_DISABLE;
    reset_pin_config.pin_bit_mask = BIT(PIN_TO_RCP_RESET);
    reset_pin_config.mode = GPIO_MODE_OUTPUT;
    reset_pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    reset_pin_config.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&reset_pin_config);
    gpio_set_level(PIN_TO_RCP_RESET, 0);

    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_TO_RCP_RESET, 1);

    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_reset_pin(PIN_TO_RCP_RESET);
}
#endif

#if CONFIG_EXTERNAL_COEX_ENABLE
static void ot_br_external_coexist_init(void)
{
    esp_external_coex_gpio_set_t gpio_pin = ESP_OPENTHREAD_DEFAULT_EXTERNAL_COEX_CONFIG();
    esp_external_coex_set_work_mode(EXTERNAL_COEX_LEADER_ROLE);
    ESP_ERROR_CHECK(esp_enable_extern_coex_gpio_pin(CONFIG_EXTERNAL_COEX_WIRE_TYPE, gpio_pin));
}
#endif // CONFIG_EXTERNAL_COEX_ENABLE

static void inicio_Openthread(void *aContext)  // Tarea de inicio de Openthread
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *openthread_netif = esp_netif_new(&cfg);
    assert(openthread_netif != NULL);

    // Inicializamos OpenThread
    ESP_ERROR_CHECK(esp_openthread_init(&config)); // Inicialización de la pila OpenThread 
    ESP_ERROR_CHECK(esp_netif_attach(openthread_netif,
                                     esp_openthread_netif_glue_init(&config))); 

    esp_openthread_lock_acquire(portMAX_DELAY);

#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    // Match OT log level to ESP log
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif

#if CONFIG_OPENTHREAD_CLI
    // Reduce noise when CLI is active
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("OPENTHREAD", ESP_LOG_WARN);
    esp_log_level_set("*", ESP_LOG_INFO);

    esp_openthread_cli_init();
#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif
    esp_openthread_cli_create_task();
#endif // CONFIG_OPENTHREAD_CLI

    esp_openthread_lock_release();

    
    esp_openthread_launch_mainloop();

    
    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);
    esp_vfs_eventfd_unregister();

    vTaskDelete(NULL);
}

void ot_br_init(void *ctx)
{
#if CONFIG_OPENTHREAD_CLI_WIFI
    ESP_ERROR_CHECK(esp_ot_wifi_config_init());
#endif

#if CONFIG_OPENTHREAD_BR_AUTO_START
#if CONFIG_EXAMPLE_CONNECT_WIFI || CONFIG_EXAMPLE_CONNECT_ETHERNET
    bool wifi_or_ethernet_connected = false;
#else
#error No backbone netif!
#endif

#if CONFIG_EXAMPLE_CONNECT_WIFI
    char wifi_ssid[32] = "";
    char wifi_password[64] = "";

#if CONFIG_OPENTHREAD_CLI_WIFI
    if (esp_ot_wifi_config_get_ssid(wifi_ssid) == ESP_OK) {
        ESP_LOGI(TAG, "Using Wi-Fi config from NVS");
        esp_ot_wifi_config_get_password(wifi_password);
    } else
#endif
    {
        ESP_LOGI(TAG, "Using Wi-Fi config from Kconfig");
        strcpy(wifi_ssid, CONFIG_EXAMPLE_WIFI_SSID);
        strcpy(wifi_password, CONFIG_EXAMPLE_WIFI_PASSWORD);
    }

#if CONFIG_OPENTHREAD_CLI_WIFI
    if (esp_ot_wifi_connect(wifi_ssid, wifi_password) == ESP_OK) {
#else
    if (example_wifi_connect() == ESP_OK) {
#endif
        wifi_or_ethernet_connected = true;
    } else {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi, please try manually");
    }
#endif

#if CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(example_ethernet_connect());
    wifi_or_ethernet_connected = true;
#endif
#endif // CONFIG_OPENTHREAD_BR_AUTO_START

#if CONFIG_EXTERNAL_COEX_ENABLE
    ot_br_external_coexist_init();
#endif

    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp-ot-br"));

    esp_openthread_lock_acquire(portMAX_DELAY);

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
    ESP_ERROR_CHECK(esp_openthread_state_indicator_init(esp_openthread_get_instance()));
#endif

#if CONFIG_OPENTHREAD_BR_AUTO_START
    if (wifi_or_ethernet_connected) {
        esp_openthread_set_backbone_netif(get_example_netif()); //establecemos la interfaz de red backbone del border roter
        ESP_ERROR_CHECK(esp_openthread_border_router_init());

#if CONFIG_EXAMPLE_CONNECT_WIFI
#if CONFIG_OPENTHREAD_CLI_WIFI
        esp_ot_wifi_border_router_init_flag_set(true);
#endif
#endif

        otOperationalDatasetTlvs dataset;
        otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);  //obtenemos el dataset de la red Thread 

        ESP_ERROR_CHECK(
            esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
    } else {
        ESP_LOGE(TAG, "Auto-start mode failed, please start manually");
    }
#endif // CONFIG_OPENTHREAD_BR_AUTO_START

    esp_openthread_lock_release();
    vTaskDelete(NULL);
}



// Mostrar imagen en pantalla
static void mostrar_imagen(void)
{
    if (!inicializacion_lcd || panel_handle == NULL) return;
    
    int linea_envio = 0;
    int linea_calculo = 0;
// bucle que va enviando lineas a la pantalla de forma paralela 
    for (int y = 0; y < EXAMPLE_LCD_V_RES; y += PARALLEL_LINES) {

        relleno_buffer(s_lines[linea_calculo], y, PARALLEL_LINES, EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
        linea_envio = linea_calculo;
        linea_calculo = !linea_calculo;
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, 0 + EXAMPLE_LCD_H_RES, y + PARALLEL_LINES, s_lines[linea_envio]);
    }
}



static esp_err_t init_lcd(void)
{
    // Configuraciones GPIO y SPI para la pantalla LCD
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL));

    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_PCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_DATA0,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PARALLEL_LINES * EXAMPLE_LCD_H_RES * 2 + 8
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &lcd_io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    // Mantenemos backlight apagado hasta que se reciba una alerta
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL));

    ESP_ERROR_CHECK(decodificacion_pantalla());
    // Asignamos memoria para las líneas de la pantalla
    for (int i = 0; i < 2; i++) {

        s_lines[i] = heap_caps_malloc(EXAMPLE_LCD_H_RES * PARALLEL_LINES * sizeof(uint16_t), MALLOC_CAP_DMA);

        if (s_lines[i] == NULL) {
            ESP_LOGE(TAG, "Error al asignar memoria para LCD");
            return ESP_ERR_NO_MEM;
        }
    }

    inicializacion_lcd = true;
   
    ESP_LOGI(TAG, "LCD inicializado correctamente (backlight apagado)");
    return ESP_OK;
}


// Inicializar RMT para el buzzer
static esp_err_t init_buzzer(void)
{
    //ESP_LOGI(TAG, "Create RMT TX channel for buzzer");
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = RMT_BUZZER_GPIO_NUM,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_BUZZER_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &buzzer_chan));

    //ESP_LOGI(TAG, "Install musical score encoder");
    musical_score_encoder_config_t encoder_config = {
        .resolution = RMT_BUZZER_RESOLUTION_HZ
    };
    ESP_ERROR_CHECK(rmt_new_musical_score_encoder(&encoder_config, &score_encoder)); // creamos el codificador para la alarma

    
    ESP_ERROR_CHECK(rmt_enable(buzzer_chan));

    return ESP_OK;
}

// Función que toca la secuencia de alarma en el buzzer
static void alarma_buzzer(void)
{
    if (!buzzer_chan || !score_encoder) {
        return;
    }

    // Recorremos los pares de frecuencia y duración de la secuencia de la alarma
    for (size_t i = 0; i < sizeof(frequency_sweep) / sizeof(frequency_sweep[0]); i++) {
        
        rmt_transmit_config_t tx_config = {
            .loop_count = frequency_sweep[i].freq_hz * frequency_sweep[i].duration_ms / 1000,
        };
        ESP_ERROR_CHECK(rmt_transmit(buzzer_chan, score_encoder, &frequency_sweep[i], sizeof(buzzer_musical_score_t), &tx_config));
    }
    ESP_LOGI(TAG, "Secuencia del buzzer completada");
}

static void tarea_recepcion_udp(void *arg)
{
    otInstance *instance = NULL;

    // Esperar inicialización de OpenThread
    while ((instance = esp_openthread_get_instance()) == NULL) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    otUdpSocket socket;
    memset(&socket, 0, sizeof(socket));

    // Handler de recepción UDP
    void handler(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
    {
        udp_msg_t msg;
        
        // Obtenemos el contenido del mensaje UDP 
        uint16_t msg_len = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);
        if (msg_len > sizeof(msg.data) - 1) {
            msg_len = sizeof(msg.data) - 1;
        }
        
        msg.length = otMessageRead(aMessage, otMessageGetOffset(aMessage), msg.data, msg_len);  
        msg.data[msg.length] = '\0';

        msg.peer_addr = aMessageInfo->mPeerAddr;
        msg.peer_port = aMessageInfo->mPeerPort;
        
        xQueueSend(udp_msg_queue, &msg, portMAX_DELAY); // Enviamos el mensaje a la cola para su procesamiento
        
    }

    esp_openthread_lock_acquire(portMAX_DELAY);

    //creacion del socket UDP y bind al puerto 1234
    otError error = otUdpOpen(instance, &socket, handler, NULL); 
    if (error != OT_ERROR_NONE) {
        ESP_LOGE("UDP_RECV", "Error al abrir socket: %d", error);
        esp_openthread_lock_release();
        vTaskDelete(NULL);
    }

    otSockAddr listenAddr;
    memset(&listenAddr, 0, sizeof(listenAddr));
    listenAddr.mPort = 1234;

    error = otUdpBind(instance, &socket, &listenAddr, OT_NETIF_UNSPECIFIED);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE("UDP_RECV", "Error en bind UDP: %d", error);
        esp_openthread_lock_release();
        vTaskDelete(NULL);
    }
    esp_openthread_lock_release();
    
    ESP_LOGI("UDP_RECV", "Socket UDP escuchando en puerto 1234");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Tarea que toca el buzzer en paralelo 
static void tarea_buzzer(void *arg)
{
    if (buzzer_chan != NULL) {
        alarma_buzzer();  
    }
    vTaskDelete(NULL);  
}

// Tarea de muestra de mensajes UDP recibidos y activación de actuadores y mensaje MQTT
static void tarea_alertas(void *arg)
{
    udp_msg_t msg;
    
    while (1) {   //se activa al recibir mensajes en la cola
        if (xQueueReceive(udp_msg_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI("UDP_PROC", "Mensaje desde [%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]:%u -> %s",
                     msg.peer_addr.mFields.m8[0], msg.peer_addr.mFields.m8[1],msg.peer_addr.mFields.m8[2], msg.peer_addr.mFields.m8[3],
                     msg.peer_addr.mFields.m8[4], msg.peer_addr.mFields.m8[5],msg.peer_addr.mFields.m8[6], msg.peer_addr.mFields.m8[7],
                     msg.peer_addr.mFields.m8[8], msg.peer_addr.mFields.m8[9],msg.peer_addr.mFields.m8[10], msg.peer_addr.mFields.m8[11],
                     msg.peer_addr.mFields.m8[12], msg.peer_addr.mFields.m8[13],msg.peer_addr.mFields.m8[14], msg.peer_addr.mFields.m8[15],
                     msg.peer_port, msg.data);
            
            // Inicializar LCD si no está inicializado (lazy initialization)
            if (!inicializacion_lcd) {
                ESP_LOGI("UDP_PROC", "Inicializando LCD...");
                esp_err_t ret = init_lcd();
                if (ret != ESP_OK) {
                    ESP_LOGE("UDP_PROC", "Error al inicializar LCD: %s", esp_err_to_name(ret));
                    continue;
                }
            }
            
            
            if (inicializacion_lcd) {
                

                // Publicamos alerta a Home Assistant vía MQTT
                mqtt_publish_alert();

                // Dibujamos la imagen una sola vez  
                mostrar_imagen();

                // Encendemos backlight LCD para que se vea la imagen
                gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

                
                if (buzzer_chan == NULL) {
                    
                    esp_err_t ret = init_buzzer();
                    if (ret != ESP_OK) {
                        ESP_LOGE("UDP_PROC", "Error al inicializar buzzer: %s", esp_err_to_name(ret));
                    }
                }

                // Creaamos tarea para tocar el buzzer en paralelo con RTOS
                if (buzzer_chan != NULL) {
                    xTaskCreate(tarea_buzzer, "tarea_buzzer", 2048, NULL, 3, NULL);
                }

                //parpademos la pantalla durante 30 s con el backlight
                TickType_t start_time = xTaskGetTickCount();
                const TickType_t duracion = pdMS_TO_TICKS(30000);
                int level = EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL;  // Empezar apagando (ya estuvo encendida al inicio)

                while ((xTaskGetTickCount() - start_time) < duracion) {

                    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, level);
                    level = !level;
                    vTaskDelay(pdMS_TO_TICKS(500));

                }

                // Apagamos backlight LCD
                gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);
                
                // Reseteamos alerta MQTT a OFF
                mqtt_reset_alert();
                
                ESP_LOGI("UDP_PROC", "Backlight apagado - pantalla lista para próxima alarma");
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "=== BORDER ROUTER + UDP RECV INIT ===");
    ESP_LOGI(TAG, "========================================");
    
    // Configuramos y apagamos backlight LCD inmediatamente para evitar luz al arrancar
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    gpio_config(&bk_gpio_config);
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);
    ESP_LOGI(TAG, "Backlight LCD apagado al inicio");
    

    size_t max_eventfd = 3;

#if CONFIG_OPENTHREAD_RADIO_NATIVE || CONFIG_OPENTHREAD_RADIO_SPINEL_SPI
    max_eventfd++;  
#endif

#if CONFIG_OPENTHREAD_RADIO_TREL
    max_eventfd++;  
#endif

    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = max_eventfd,
    };

    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    
    vTaskDelay(pdMS_TO_TICKS(5000));  // Esperamos 5s para que WiFi se conecte para que no haya problemas
    init_mqtt();

    // Creamos cola para mensajes UDP
    udp_msg_queue = xQueueCreate(10, sizeof(udp_msg_t));  
    if (udp_msg_queue == NULL) {
        ESP_LOGE(TAG, "Error al crear cola UDP");
        return;
    }

#if CONFIG_OPENTHREAD_SUPPORT_HW_RESET_RCP
    esp_openthread_register_rcp_failure_handler(rcp_failure_hardware_reset_handler);
#endif

    xTaskCreate(inicio_Openthread, "inicio_Openthread", 7168,  
                xTaskGetCurrentTaskHandle(), 5, NULL);

    xTaskCreate(ot_br_init, "ot_br_init", 6144,  
                NULL, 4, NULL);

    xTaskCreate(tarea_recepcion_udp, "tarea_recepcion_udp", 3072,  
                NULL, 3, NULL);
    
    xTaskCreate(tarea_alertas, "tarea_alertas", 4096,  
                NULL, 2, NULL);
}