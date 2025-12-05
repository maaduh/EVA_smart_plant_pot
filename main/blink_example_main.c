/* Smart Plant Ivy - Monitoramento Inteligente de Plantas
 * 
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                          DIAGRAMA DE CONEXÕES                            ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 * 
 * ESP32 PINOUT:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ 3.3V  ──→  VCC de todos os sensores (AHT10, BH1750, VL53L0X)          │
 * │ GND   ──→  GND de todos os sensores + LED RGB (pino comum)            │
 * │ GPIO21──→  SDA (barramento I2C - conecta nos 3 sensores)              │
 * │ GPIO22──→  SCL (barramento I2C - conecta nos 3 sensores)              │
 * │ GPIO27──→  LED RGB - Pino R (Vermelho)                                │
 * │ GPIO25──→  LED RGB - Pino G (Verde)                                   │
 * │ GPIO26──→  LED RGB - Pino B (Azul)                                    │
 * └─────────────────────────────────────────────────────────────────────────┘
 * 
 * SENSOR AHT10 (Temperatura/Umidade):
 * ┌──────────┬───────────┐
 * │ AHT10    │ ESP32     │
 * ├──────────┼───────────┤
 * │ VCC/VIN  │ 3.3V      │
 * │ GND      │ GND       │
 * │ SDA      │ GPIO 21   │
 * │ SCL      │ GPIO 22   │
 * └──────────┴───────────┘
 * Endereço I2C: 0x38
 * 
 * SENSOR BH1750 (Luminosidade):
 * ┌──────────┬───────────┐
 * │ BH1750   │ ESP32     │
 * ├──────────┼───────────┤
 * │ VCC      │ 3.3V      │
 * │ GND      │ GND       │
 * │ SDA      │ GPIO 21   │
 * │ SCL      │ GPIO 22   │
 * └──────────┴───────────┘
 * Endereço I2C: 0x23
 * 
 * SENSOR VL53L0X (Distância/Nível Água):
 * ┌──────────┬───────────┐
 * │ VL53L0X  │ ESP32     │
 * ├──────────┼───────────┤
 * │ VIN      │ 3.3V      │
 * │ GND      │ GND       │
 * │ SDA      │ GPIO 21   │
 * │ SCL      │ GPIO 22   │  
 * └──────────┴───────────┘
 * Endereço I2C: 0x29
 * 
 * LED RGB (Common Anode - 4 pinos):
 * ┌──────────┬───────────┐
 * │ LED RGB  │ ESP32     │
 * ├──────────┼───────────┤
 * │ R (Red)  │ GPIO 27   │ 
 * │ G (Green)│ GPIO 25   │ 
 * │ B (Blue) │ GPIO 26   │ 
 * │ I (comum)│ GND       │ 
 * └──────────┴───────────┘
 * 
 * NOTAS:
 * • Todos os sensores I2C compartilham os mesmos pinos SDA/SCL
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "vl53l0x.h"
#include "ssd1306.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "wifi.h"
#include "mqtt.h"

SemaphoreHandle_t conexaoWifiSemaphore;
SemaphoreHandle_t conexaoMQTTSemaphore;

// ===== CONFIGURAÇÃO I2C =====
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000

// ===== ENDEREÇOS I2C DOS SENSORES =====
#define AHT10_ADDR           0x38
#define BH1750_ADDR          0x23
#define VL53L0X_ADDR         0x29

// ===== COMANDOS AHT10 =====
#define AHT10_CMD_INIT       0xE1
#define AHT10_CMD_TRIGGER    0xAC
#define AHT10_CMD_SOFTRESET  0xBA

// ===== COMANDOS BH1750 =====
#define BH1750_POWER_ON      0x01
#define BH1750_CONTINUOUS_H  0x10  // 1 lx resolution

// ===== CONFIGURAÇÃO RGB LED (Common Anode) =====
#define LED_RED_GPIO         26
#define LED_GREEN_GPIO       25
#define LED_BLUE_GPIO        27

#define LEDC_TIMER           LEDC_TIMER_0
#define LEDC_MODE            LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES        LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY       5000

#define LEDC_CHANNEL_RED     LEDC_CHANNEL_0
#define LEDC_CHANNEL_GREEN   LEDC_CHANNEL_1
#define LEDC_CHANNEL_BLUE    LEDC_CHANNEL_2

// ===== LIMIARES DE ALERTA =====
#define TEMP_MIN             15.0f  // °C
#define TEMP_MAX             30.0f  // °C
#define HUMIDITY_MIN         40.0f  // % (sensor próximo à terra: <40% = solo seco, precisa regar!)
#define LUX_MIN              200.0f // lux (precisa de luz)
#define WATER_LEVEL_CRITICAL 150    // mm (distância - quanto maior, menos água)

// ===== CORES RGB (Common Anode: 0=aceso, 255=apagado) =====
#define RGB_OFF         255, 255, 255
#define RGB_GREEN       0, 255, 0
#define RGB_YELLOW      0, 255, 255
#define RGB_RED         255, 0, 255
#define RGB_BLUE        255, 255, 0
#define RGB_WHITE       0, 0, 0

static const char *TAG = "SmartPlant";

// ===== PROTÓTIPOS DE FUNÇÕES =====
static void rgb_set_color(uint8_t r, uint8_t g, uint8_t b);

// ===== ESTRUTURA DE DADOS DOS SENSORES =====
typedef struct {
    float temperature;
    float humidity;
    float lux;
    uint16_t water_distance_mm;
    bool temp_ok;
    bool humidity_ok;
    bool light_ok;
    bool water_ok;
    uint8_t alert_count;
} plant_status_t;

// ===== FUNÇÕES RGB LED =====
static void rgb_led_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t channels[3] = {
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_RED,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = LED_RED_GPIO,
            .duty           = 255, 
            .hpoint         = 0
        },
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_GREEN,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = LED_GREEN_GPIO,
            .duty           = 255,
            .hpoint         = 0
        },
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_BLUE,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = LED_BLUE_GPIO,
            .duty           = 255,
            .hpoint         = 0
        }
    };

    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }
    
    ESP_LOGI(TAG, "RGB LED configurado (Common Anode)");
}

static void rgb_set_color(uint8_t r, uint8_t g, uint8_t b) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED, r);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED);
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN, g);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN);
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, b);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE);
}

// ===== FUNÇÕES AHT10 =====
static esp_err_t aht10_init(void) {
    vTaskDelay(pdMS_TO_TICKS(40));
    
    // Envia comando de calibração/inicialização
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT10_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AHT10_CMD_INIT, true);
    i2c_master_write_byte(cmd, 0x08, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    vTaskDelay(pdMS_TO_TICKS(10));
    return ret;
}

static esp_err_t aht10_read(float *temperature, float *humidity) {
    // Trigger measurement
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT10_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AHT10_CMD_TRIGGER, true);
    i2c_master_write_byte(cmd, 0x33, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(80));
    
    // Read data
    uint8_t data[6];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT10_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return ret;
    
    uint32_t humidity_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t temperature_raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    
    *humidity = (humidity_raw * 100.0f) / 1048576.0f;
    *temperature = ((temperature_raw * 200.0f) / 1048576.0f) - 50.0f;
    
    return ESP_OK;
}

// ===== FUNÇÕES BH1750 =====
static esp_err_t bh1750_init(void) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BH1750_POWER_ON, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Start continuous high-res measurement
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BH1750_CONTINUOUS_H, true);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t bh1750_read(float *lux) {
    uint8_t data[2];
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return ret;
    
    uint16_t raw = (data[0] << 8) | data[1];
    *lux = raw / 1.2f;
    
    return ESP_OK;
}

// ===== FUNÇÃO DE ATUALIZAÇÃO DE STATUS =====
static void update_plant_status(plant_status_t *status) {
    status->temp_ok = (status->temperature >= TEMP_MIN && status->temperature <= TEMP_MAX);
    status->humidity_ok = (status->humidity >= HUMIDITY_MIN);
    status->light_ok = (status->lux >= LUX_MIN);
    status->water_ok = (status->water_distance_mm < WATER_LEVEL_CRITICAL);
    
    status->alert_count = 0;
    if (!status->temp_ok) status->alert_count++;
    if (!status->humidity_ok) status->alert_count++;
    if (!status->light_ok) status->alert_count++;
    if (!status->water_ok) status->alert_count++;
    
    if (!status->water_ok) {
        rgb_set_color(RGB_RED);
        ESP_LOGW(TAG, "⚠️  ALERTA: Nível de água baixo!");
    } else if (status->alert_count == 0) {
        rgb_set_color(RGB_GREEN);
    } else if (status->alert_count == 1) {
        rgb_set_color(RGB_YELLOW);
    } else {
        rgb_set_color(RGB_RED);
    }
    
    if (!status->water_ok || status->alert_count >= 3) {
        ssd1306_display_face(FACE_SAD);
    } else if (status->alert_count >= 1) {
        ssd1306_display_face(FACE_STRESSED);
    } else {
        ssd1306_display_face(FACE_HAPPY);
    }
}

// ===== FUNÇÃO DE SCAN I2C =====
static void scan_i2c_bus(void)
{
    ESP_LOGI(TAG, "\n🔍 Escaneando barramento I2C...");
    ESP_LOGI(TAG, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    for (int base = 0; base < 128; base += 16) {
        printf("%02x: ", base);
        for (int offset = 0; offset < 16; offset++) {
            const uint8_t address = base + offset;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            const esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("?? ");
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
    ESP_LOGI(TAG, " Scan completo\n");
}

void app_main(void)
{
    esp_err_t ret_nvs = nvs_flash_init();
    if (ret_nvs == ESP_ERR_NVS_NO_FREE_PAGES || ret_nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret_nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret_nvs);

    // Cria os semáforos
    conexaoWifiSemaphore = xSemaphoreCreateBinary();
    conexaoMQTTSemaphore = xSemaphoreCreateBinary();

    // Inicia Wi-Fi e MQTT
    wifi_start();
    // Nota: O mqtt_start vai tentar conectar, mas só conseguirá quando o Wi-Fi subir.
    // O código do mqtt.c gerencia isso internamente ou podemos esperar o wifi antes.
    mqtt_start();
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║           SMART PLANT IVY - Sistema Iniciando                 ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, " Configuração do Sistema:");
    ESP_LOGI(TAG, "   • AHT10  (0x38): Temperatura e Umidade do Ar");
    ESP_LOGI(TAG, "   • BH1750 (0x23): Luminosidade Ambiente");
    ESP_LOGI(TAG, "   • VL53L0X(0x29): Nível de Água (medição de distância)");
    ESP_LOGI(TAG, "   • RGB LED      : Alertas Visuais de Status");
    ESP_LOGI(TAG, "");
    
    rgb_led_init();
    rgb_set_color(RGB_BLUE);
    
    // ===== CONFIGURAÇÃO I2C GLOBAL =====
    ESP_LOGI(TAG, " Configurando barramento I2C...");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Falha na configuração I2C: %s", esp_err_to_name(ret));
        rgb_set_color(RGB_RED);
        return;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Falha ao instalar driver I2C: %s", esp_err_to_name(ret));
        rgb_set_color(RGB_RED);
        return;
    }
    ESP_LOGI(TAG, "    I2C configurado (GPIO21=SDA, GPIO22=SCL, 100kHz)");
    ESP_LOGI(TAG, "");
    
    scan_i2c_bus();
    
    // Inicializa AHT10
    ESP_LOGI(TAG, "  Inicializando AHT10 (Temperatura/Umidade)...");
    if (aht10_init() == ESP_OK) {
        ESP_LOGI(TAG, "    AHT10 pronto!");
    } else {
        ESP_LOGE(TAG, "    Falha ao inicializar AHT10");
    }
    
    // Inicializa BH1750
    ESP_LOGI(TAG, "  Inicializando BH1750 (Luminosidade)...");
    if (bh1750_init() == ESP_OK) {
        ESP_LOGI(TAG, "    BH1750 pronto!");
    } else {
        ESP_LOGE(TAG, "    Falha ao inicializar BH1750");
    }
    
    // Inicializa VL53L0X
    ESP_LOGI(TAG, " Inicializando VL53L0X (Nível de Água)...");
    vl53l0x_t *tof = vl53l0x_config((int8_t)I2C_MASTER_NUM,
                                    I2C_MASTER_SCL_IO,
                                    I2C_MASTER_SDA_IO,
                                    -1,
                                    VL53L0X_ADDR,
                                    true);
    if (!tof) {
        ESP_LOGE(TAG, "    Falha ao configurar VL53L0X driver");
        tof = NULL;
    } else {
        vl53l0x_setTimeout(tof, 500);
        
        esp_task_wdt_delete(xTaskGetCurrentTaskHandle());
        
        const char *init_err = vl53l0x_init(tof);
        if (init_err != NULL) {
            ESP_LOGE(TAG, "    VL53L0X init falhou: %s", init_err);
            vl53l0x_end(tof);
            tof = NULL;
        } else {
            ESP_LOGI(TAG, "    VL53L0X pronto!");
            
            const char *budget_err = vl53l0x_setMeasurementTimingBudget(tof, 33000);
            if (budget_err != NULL) {
                ESP_LOGW(TAG, "     Timing budget warning: %s", budget_err);
            }
        }
        
        esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, " Todos os sensores inicializados!");
    
    ESP_LOGI(TAG, " Inicializando display OLED...");
    ssd1306_init(I2C_MASTER_NUM);
    ssd1306_display_face(FACE_HAPPY);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, " Iniciando monitoramento contínuo...");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "");
    
    plant_status_t status = {0};
    uint32_t cycle_count = 0;
    uint8_t vl53l0x_timeout_count = 0;
    
    while (1) {
        cycle_count++;
        
        esp_task_wdt_reset();
        
        // Lê AHT10
        if (aht10_read(&status.temperature, &status.humidity) != ESP_OK) {
            ESP_LOGW(TAG, "  Erro ao ler AHT10");
        }
        
        // Lê BH1750
        if (bh1750_read(&status.lux) != ESP_OK) {
            ESP_LOGW(TAG, "  Erro ao ler BH1750");
        }
        
        if (tof != NULL) {
            status.water_distance_mm = vl53l0x_readRangeSingleMillimeters(tof);
            if (vl53l0x_timeoutOccurred(tof)) {
                ESP_LOGW(TAG, "  Timeout VL53L0X");
                status.water_distance_mm = 999;
                
                vl53l0x_timeout_count++;
                if (vl53l0x_timeout_count >= 3) {
                    ESP_LOGW(TAG, "🔄 Tentando reiniciar VL53L0X após %d timeouts...", vl53l0x_timeout_count);
                    
                    vl53l0x_end(tof);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    
                    tof = vl53l0x_config(I2C_MASTER_NUM, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, -1, 0x29, 0);
                    if (tof != NULL) {
                        const char* init_err = vl53l0x_init(tof);
                        if (init_err == NULL) {
                            vl53l0x_setTimeout(tof, 1500);
                            vl53l0x_setMeasurementTimingBudget(tof, 33000);
                            vl53l0x_timeout_count = 0;
                            ESP_LOGI(TAG, " VL53L0X reiniciado com sucesso!");
                        } else {
                            ESP_LOGE(TAG, " Falha ao reiniciar VL53L0X: %s", init_err);
                            vl53l0x_end(tof);
                            tof = NULL;
                        }
                    } else {
                        ESP_LOGE(TAG, " Falha ao configurar VL53L0X");
                    }
                }
            } else {
                vl53l0x_timeout_count = 0;
            }
        } else {
            status.water_distance_mm = 100;
        }
        
        update_plant_status(&status);
        
        if (cycle_count % 10 == 0) {
            if(xSemaphoreTake(conexaoMQTTSemaphore, pdMS_TO_TICKS(100))) {
                
                // Cria o objeto JSON
                cJSON *root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "temperatura", status.temperature);
                cJSON_AddNumberToObject(root, "umidade", status.humidity);
                cJSON_AddNumberToObject(root, "luminosidade", status.lux);
                cJSON_AddNumberToObject(root, "nivel_agua_mm", status.water_distance_mm);
                
                // Adiciona status booleanos (0 ou 1) para facilitar gráficos
                cJSON_AddBoolToObject(root, "alerta_agua", !status.water_ok);
                
                // Converte o JSON para string
                char *json_string = cJSON_PrintUnformatted(root);
                
                // Envia para o tópico de telemetria do ThingsBoard
                mqtt_envia_mensagem("v1/devices/me/telemetry", json_string);
                
                ESP_LOGI(TAG, "📡 Dados enviados para o ThingsBoard!");

                // Limpa a memória (MUITO IMPORTANTE para não travar o ESP32)
                free(json_string);
                cJSON_Delete(root);
                
                // Devolve o semáforo para indicar que a conexão ainda existe/está livre
                xSemaphoreGive(conexaoMQTTSemaphore);
            } else {
                ESP_LOGW(TAG, "📡 MQTT desconectado - Tentando reconectar ou aguardando...");
            }
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, " Status da Planta #%lu:", cycle_count / 10);
            ESP_LOGI(TAG, "     Temperatura: %.1f°C %s (%.1f~%.1f°C)", 
                     status.temperature,
                     status.temp_ok ? "✅" : "⚠️",
                     TEMP_MIN, TEMP_MAX);
            ESP_LOGI(TAG, "    Umidade Ar : %.1f%% %s (>%.0f%% = solo úmido)", 
                     status.humidity,
                     status.humidity_ok ? "✅" : "⚠️",
                     HUMIDITY_MIN);
            ESP_LOGI(TAG, "     Luminosidade: %.0f lux %s (>%.0flux)", 
                     status.lux,
                     status.light_ok ? "✅" : "⚠️",
                     LUX_MIN);
            ESP_LOGI(TAG, "    Nível Água: %dmm %s (<%dmm=OK)", 
                     status.water_distance_mm,
                     status.water_ok ? "✅" : "🚨",
                     WATER_LEVEL_CRITICAL);
            
            if (status.alert_count == 0) {
                ESP_LOGI(TAG, "    Status: TUDO OK - Planta saudável!");
            } else if (status.alert_count == 1) {
                ESP_LOGW(TAG, "    Status: ATENÇÃO - 1 parâmetro fora do ideal");
            } else {
                ESP_LOGE(TAG, "    Status: CRÍTICO - %d parâmetros precisam de atenção!", 
                         status.alert_count);
            }
            ESP_LOGI(TAG, "");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
