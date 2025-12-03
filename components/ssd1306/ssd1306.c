#include "ssd1306.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "SSD1306";
static i2c_port_t i2c_num = I2C_NUM_0;

// Comandos SSD1306
#define SSD1306_CMD_DISPLAY_OFF     0xAE
#define SSD1306_CMD_DISPLAY_ON      0xAF
#define SSD1306_CMD_SET_CONTRAST    0x81
#define SSD1306_CMD_NORMAL_DISPLAY  0xA6
#define SSD1306_CMD_SET_MUX_RATIO   0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_CMD_SET_START_LINE  0x40
#define SSD1306_CMD_SET_CHARGE_PUMP 0x8D
#define SSD1306_CMD_SET_MEMORY_MODE 0x20
#define SSD1306_CMD_SET_SEGMENT_REMAP 0xA1
#define SSD1306_CMD_SET_COM_SCAN_DEC 0xC8
#define SSD1306_CMD_SET_COM_PINS    0xDA
#define SSD1306_CMD_SET_PRECHARGE   0xD9
#define SSD1306_CMD_SET_VCOM_DETECT 0xDB
#define SSD1306_CMD_DEACTIVATE_SCROLL 0x2E

static uint8_t display_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static esp_err_t ssd1306_write_command(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd};
    return i2c_master_write_to_device(i2c_num, SSD1306_I2C_ADDR, data, 2, pdMS_TO_TICKS(100));
}

static esp_err_t ssd1306_write_data(uint8_t* data, size_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = 0x40;  // Data mode
    memcpy(buffer + 1, data, len);
    return i2c_master_write_to_device(i2c_num, SSD1306_I2C_ADDR, buffer, len + 1, pdMS_TO_TICKS(100));
}

void ssd1306_init(i2c_port_t port) {
    i2c_num = port;
    
    ssd1306_write_command(SSD1306_CMD_DISPLAY_OFF);
    ssd1306_write_command(SSD1306_CMD_SET_MUX_RATIO);
    ssd1306_write_command(0x3F);  // 64 lines
    ssd1306_write_command(SSD1306_CMD_SET_DISPLAY_OFFSET);
    ssd1306_write_command(0x00);
    ssd1306_write_command(SSD1306_CMD_SET_START_LINE | 0x00);
    ssd1306_write_command(SSD1306_CMD_SET_CHARGE_PUMP);
    ssd1306_write_command(0x14);  // Enable
    ssd1306_write_command(SSD1306_CMD_SET_MEMORY_MODE);
    ssd1306_write_command(0x00);  // Horizontal
    ssd1306_write_command(SSD1306_CMD_SET_SEGMENT_REMAP);
    ssd1306_write_command(SSD1306_CMD_SET_COM_SCAN_DEC);
    ssd1306_write_command(SSD1306_CMD_SET_COM_PINS);
    ssd1306_write_command(0x12);
    ssd1306_write_command(SSD1306_CMD_SET_CONTRAST);
    ssd1306_write_command(0x7F);
    ssd1306_write_command(SSD1306_CMD_SET_PRECHARGE);
    ssd1306_write_command(0xF1);
    ssd1306_write_command(SSD1306_CMD_SET_VCOM_DETECT);
    ssd1306_write_command(0x40);
    ssd1306_write_command(SSD1306_CMD_NORMAL_DISPLAY);
    ssd1306_write_command(SSD1306_CMD_DEACTIVATE_SCROLL);
    ssd1306_write_command(SSD1306_CMD_DISPLAY_ON);
    
    ssd1306_clear_screen();
    ESP_LOGI(TAG, "SSD1306 inicializado");
}

void ssd1306_clear_screen(void) {
    memset(display_buffer, 0, sizeof(display_buffer));
    
    for (int i = 0; i < 8; i++) {
        ssd1306_write_command(0xB0 + i);  // Page
        ssd1306_write_command(0x00);       // Lower column
        ssd1306_write_command(0x10);       // Upper column
        ssd1306_write_data(&display_buffer[i * 128], 128);
    }
}

// Desenha um pixel
static void set_pixel(int x, int y, bool on) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    if (on) {
        display_buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    } else {
        display_buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

// Desenha círculo (olhos e boca)
static void draw_circle(int cx, int cy, int radius, bool filled) {
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x*x + y*y <= radius*radius) {
                if (filled || (x*x + y*y >= (radius-2)*(radius-2))) {
                    set_pixel(cx + x, cy + y, true);
                }
            }
        }
    }
}

// Desenha linha
static void draw_line(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (1) {
        set_pixel(x0, y0, true);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void ssd1306_display_face(face_type_t face) {
    ssd1306_clear_screen();
    
    int cx = 64;  // Centro X
    int cy = 32;  // Centro Y
    
    // Desenha rosto (círculo grande)
    draw_circle(cx, cy, 28, false);
    
    // Olhos
    draw_circle(cx - 10, cy - 8, 4, true);
    draw_circle(cx + 10, cy - 8, 4, true);
    
    switch (face) {
        case FACE_HAPPY:  // 😊 Boca sorrindo (arco para CIMA)
            for (int x = -12; x <= 12; x++) {
                int y = -(x * x) / 20;  // Parábola negativa = curva para cima
                set_pixel(cx + x, cy + 18 + y, true);
                set_pixel(cx + x, cy + 19 + y, true);
            }
            break;
            
        case FACE_STRESSED:  // 😰 Boca reta
            draw_line(cx - 12, cy + 12, cx + 12, cy + 12);
            draw_line(cx - 12, cy + 13, cx + 12, cy + 13);
            // Gota de suor
            draw_circle(cx + 20, cy - 15, 3, true);
            break;
            
        case FACE_SAD:  // 😢 Boca triste (arco para BAIXO)
            for (int x = -12; x <= 12; x++) {
                int y = (x * x) / 20;  // Parábola positiva = curva para baixo
                set_pixel(cx + x, cy + 10 + y, true);
                set_pixel(cx + x, cy + 11 + y, true);
            }
            // Lágrima
            draw_line(cx - 10, cy - 4, cx - 10, cy + 5);
            draw_circle(cx - 10, cy + 6, 2, true);
            break;
    }
    
    // Atualiza display
    for (int i = 0; i < 8; i++) {
        ssd1306_write_command(0xB0 + i);
        ssd1306_write_command(0x00);
        ssd1306_write_command(0x10);
        ssd1306_write_data(&display_buffer[i * 128], 128);
    }
}

void ssd1306_display_text(const char* line1, const char* line2, const char* line3) {
    // Implementação simples - apenas limpa a tela por enquanto
    // Você pode adicionar uma fonte bitmap aqui se quiser texto
    ssd1306_clear_screen();
}
