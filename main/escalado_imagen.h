
#pragma once
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize the effect
 *
 * @return ESP_OK on success, an error from the jpeg decoder otherwise.
 */
esp_err_t decodificacion_pantalla(void);

/**
 * @brief Deinitialize effect and free decoded pixels buffer.
 */
void pretty_effect_deinit(void);

/**
 * @brief Blit scaled lines from decoded JPEG into destination buffer.
 *
 * @param dest Destination buffer for `linect` lines, each of width `out_w`.
 * @param start_y Starting Y in output space.
 * @param linect Number of lines to fill.
 * @param out_w Output width (e.g., 320)
 * @param out_h Output height (e.g., 240)
 */
void relleno_buffer(uint16_t *dest, int start_y, int linect, int out_w, int out_h);
