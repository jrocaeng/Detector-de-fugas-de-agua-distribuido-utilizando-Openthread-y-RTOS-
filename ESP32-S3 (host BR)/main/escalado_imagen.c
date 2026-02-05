//Este archivo viene de un codigo de ejemplo de expressif que hacia un efecto visual bonito pero al final lo he quitado y adaptado para que funcione estaticamenente porque no lo hacia bien 


#include <math.h>
#include <stdlib.h>
#include "escalado_imagen.h"
#include "sdkconfig.h"
#include "decode_image.h"

uint16_t *pixels;
static const int src_w = IMAGE_W / 2;  
static const int src_h = IMAGE_H / 2;


static inline uint16_t get_bgnd_pixel(int x, int y)
{
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= src_w) x = src_w - 1;
    if (y >= src_h) y = src_h - 1;
    return (uint16_t) * (pixels + (y * src_w) + x);
}


esp_err_t decodificacion_pantalla(void)
{
    return decode_image(&pixels);
}

void pretty_effect_deinit(void)
{
    if (pixels) {
        free(pixels);
        pixels = NULL;
    }
}

void relleno_buffer(uint16_t *dest, int start_y, int linect, int out_w, int out_h)
{
    // Reescalado utilizando Nearest Neighbor
    for (int y = start_y; y < start_y + linect; y++) {

        int src_y = (y * src_h) / out_h;

        for (int x = 0; x < out_w; x++) {
            
            int src_x = (x * src_w) / out_w;
            *dest++ = get_bgnd_pixel(src_x, src_y);
        }
    }
}
