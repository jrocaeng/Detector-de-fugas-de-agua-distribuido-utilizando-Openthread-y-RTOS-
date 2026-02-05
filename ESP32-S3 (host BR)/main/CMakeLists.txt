set(srcs "esp_ot_br.c"
         "escalado_imagen.c"
         "decode_image.c"
         "musical_score_encoder.c")

idf_component_register(SRCS ${srcs}
                       PRIV_REQUIRES esp_psram openthread esp_coex esp_wifi nvs_flash esp_lcd driver esp_driver_rmt mqtt
                       INCLUDE_DIRS "."
                       EMBED_FILES image.jpg)
