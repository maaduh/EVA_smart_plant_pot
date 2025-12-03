#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include "driver/i2c.h"

#define SSD1306_I2C_ADDR   0x3C
#define SSD1306_WIDTH      128
#define SSD1306_HEIGHT     64

typedef enum {
    FACE_HAPPY,      // 😊 Tudo OK
    FACE_STRESSED,   // 😰 Alerta
    FACE_SAD         // 😢 Crítico
} face_type_t;

void ssd1306_init(i2c_port_t i2c_port);
void ssd1306_clear_screen(void);
void ssd1306_display_face(face_type_t face);
void ssd1306_display_text(const char* line1, const char* line2, const char* line3);

#endif // SSD1306_H
