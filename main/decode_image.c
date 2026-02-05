

//codigo para decodificar la imagen

#include "decode_image.h"
#include "jpeg_decoder.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include <string.h>
#include "freertos/FreeRTOS.h"


extern const uint8_t image_jpg_start[] asm("_binary_image_jpg_start"); //
extern const uint8_t image_jpg_end[] asm("_binary_image_jpg_end");


const char *TAG_IMG = "ImageDec";

//descodifica la imagen y devuelve un puntero a los pixeles
esp_err_t decode_image(uint16_t **pixels)
{
    *pixels = NULL;
    esp_err_t ret = ESP_OK;

    // Asignamos memoria para los pixeles en PSRAM primero si se puede para bajar uso de espacio
    // Decodificamos a media resolución para reducir memoria (320 x 240 a 160 x 120)
    size_t contador_pixeles = (IMAGE_H / 2) * (IMAGE_W / 2);
    *pixels = (uint16_t *)heap_caps_calloc(contador_pixeles, sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (*pixels == NULL) {
        *pixels = (uint16_t *)heap_caps_calloc(contador_pixeles, sizeof(uint16_t), MALLOC_CAP_DEFAULT);
    }
    
    ESP_GOTO_ON_FALSE((*pixels), ESP_ERR_NO_MEM, err, TAG_IMG, "Error allocating memory for lines");

    //JPEG decode config
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)image_jpg_start,
        .indata_size = image_jpg_end - image_jpg_start,
        .outbuf = (uint8_t*)(*pixels),
        .outbuf_size = (IMAGE_W / 2) * (IMAGE_H / 2) * sizeof(uint16_t),
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_1_2,
        .flags = {
            .swap_color_bytes = 1,
        }
    };

    //decodificacion JPEG
    esp_jpeg_image_output_t outimg;
    esp_jpeg_decode(&jpeg_cfg, &outimg);

    return ret;
err:
    
    if (*pixels != NULL) {
        free(*pixels);
    }
    return ret;
}
