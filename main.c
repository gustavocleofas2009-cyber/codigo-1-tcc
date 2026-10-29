#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_camera.h"

#include "driver/uart.h"


#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

// =====================================================
// CONFIGURACOES
// =====================================================

static const char *TAG = "AUTO_CAR";

#define LIMIAR             0.50f

#define UART_PORT          UART_NUM_2
#define UART_TX_PIN        14
#define UART_BAUD_RATE     115200

static uint8_t *rgb888_buf = nullptr;

// =====================================================
// COMUNICACAO COM A SEGUNDA ESP32
// TX GPIO14 da ESP32-CAM -> RX da segunda ESP32
// GND da ESP32-CAM -> GND da segunda ESP32
// =====================================================

static void init_comunicacao()
{
    uart_config_t uart_config = {};

    uart_config.baud_rate = UART_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 1024, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART inicializada");
}

static void enviar_comando(const char *comando)
{
    uart_write_bytes(UART_PORT, comando, strlen(comando));
    uart_write_bytes(UART_PORT, "\n", 1);

    ESP_LOGI(TAG, "Comando enviado: %s", comando);
}

static void parar_carrinho()
{
    ESP_LOGW(TAG, "PARADA solicitada pela IA");
    enviar_comando("STOP");
}

// =====================================================
// ROTULOS QUE DEVEM PARAR O CARRINHO
// =====================================================

static bool rotulo_de_parada(const char *rotulo)
{
    if (rotulo == nullptr) {
        return false;
    }

    char texto[100];
    size_t tamanho = strlen(rotulo);

    if (tamanho >= sizeof(texto)) {
        tamanho = sizeof(texto) - 1;
    }

    for (size_t i = 0; i < tamanho; i++) {
        texto[i] = (char)tolower((unsigned char)rotulo[i]);
    }
    texto[tamanho] = '\0';

    return strstr(texto, "objeto") != nullptr ||
           strstr(texto, "pessoa") != nullptr ||
           strstr(texto, "obstaculo") != nullptr ||
           strstr(texto, "obstacle") != nullptr;
}

// =====================================================
// CAMERA
// =====================================================

static esp_err_t init_camera()
{
    camera_config_t config = {};

    config.pin_pwdn = 32;
    config.pin_reset = -1;
    config.pin_xclk = 0;

    config.pin_sccb_sda = 26;
    config.pin_sccb_scl = 27;

    config.pin_d7 = 35;
    config.pin_d6 = 34;
    config.pin_d5 = 39;
    config.pin_d4 = 36;
    config.pin_d3 = 21;
    config.pin_d2 = 19;
    config.pin_d1 = 18;
    config.pin_d0 = 5;

    config.pin_vsync = 25;
    config.pin_href = 23;
    config.pin_pclk = 22;

    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;

    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar camera: 0x%x", (unsigned int)err);
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();

    if (sensor != nullptr && sensor->id.PID == OV3660_PID) {
        sensor->set_vflip(sensor, 1);
        sensor->set_brightness(sensor, 1);
    }

    ESP_LOGI(TAG, "Camera inicializada");
    return ESP_OK;
}

// =====================================================
// RGB565 -> RGB888 E REDIMENSIONAMENTO PARA O MODELO
// =====================================================

static void prepare_frame(camera_fb_t *fb)
{
    const int largura_modelo = EI_CLASSIFIER_INPUT_WIDTH;
    const int altura_modelo = EI_CLASSIFIER_INPUT_HEIGHT;

    const int largura_camera = fb->width;
    const int altura_camera = fb->height;

    const uint16_t *pixels = reinterpret_cast<const uint16_t *>(fb->buf);

    for (int y = 0; y < altura_modelo; y++) {
        int origem_y = y * altura_camera / altura_modelo;

        for (int x = 0; x < largura_modelo; x++) {
            int origem_x = x * largura_camera / largura_modelo;

            uint16_t pixel = pixels[origem_y * largura_camera + origem_x];
            pixel = (pixel >> 8) | (pixel << 8);   // ordem dos bytes da camera

            uint8_t r = (uint8_t)(((pixel >> 11) & 0x1F) * 255 / 31);
            uint8_t g = (uint8_t)(((pixel >> 5) & 0x3F) * 255 / 63);
            uint8_t b = (uint8_t)((pixel & 0x1F) * 255 / 31);

            size_t indice = (size_t)(y * largura_modelo + x) * 3;

            rgb888_buf[indice] = r;
            rgb888_buf[indice + 1] = g;
            rgb888_buf[indice + 2] = b;
        }
    }
}

// =====================================================
// LEITURA DOS PIXELS PELO EDGE IMPULSE
// =====================================================

static int get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t total_pixels =
        (size_t)EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;

    if (offset + length > total_pixels) {
        return -1;
    }

    for (size_t i = 0; i < length; i++) {
        size_t indice_rgb = (offset + i) * 3;

        uint8_t r = rgb888_buf[indice_rgb];
        uint8_t g = rgb888_buf[indice_rgb + 1];
        uint8_t b = rgb888_buf[indice_rgb + 2];

        out_ptr[i] = (float)((r << 16) | (g << 8) | b);
    }

    return 0;
}

// =====================================================
// PROGRAMA PRINCIPAL
// =====================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando carrinho autonomo");

    init_comunicacao();

    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera indisponivel");
        enviar_comando("STOP");

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    size_t tamanho_buffer =
        (size_t)EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;

    rgb888_buf = (uint8_t *)heap_caps_malloc(
        tamanho_buffer, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (rgb888_buf == nullptr) {
        ESP_LOGE(TAG, "Falha ao alocar buffer RGB");
        enviar_comando("STOP");

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Buffer RGB alocado: %u bytes", (unsigned)tamanho_buffer);

    while (true) {

        camera_fb_t *fb = esp_camera_fb_get();

        if (fb == nullptr) {
            ESP_LOGE(TAG, "Falha ao capturar imagem");
            enviar_comando("STOP");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ESP_LOGI(TAG, "Frame: %u x %u | formato: %d | bytes: %u",
                 (unsigned)fb->width, (unsigned)fb->height,
                 (int)fb->format, (unsigned)fb->len);

        prepare_frame(fb);
        esp_camera_fb_return(fb);

        signal_t signal = {};
        signal.total_length =
            (size_t)EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &get_data;

        ei_impulse_result_t result = {};

        EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

        if (err != EI_IMPULSE_OK) {
            ESP_LOGE(TAG, "Erro na IA: %d", err);
            enviar_comando("STOP");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ESP_LOGI(TAG, "DSP: %d ms | inferencia: %d ms",
                 (int)result.timing.dsp, (int)result.timing.classification);

        bool parar = false;

#if EI_CLASSIFIER_OBJECT_DETECTION == 1

        for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];

            if (bb.value <= 0.0f) {
                continue;
            }

            ESP_LOGI(TAG, "Deteccao: %s | confianca: %.2f",
                     bb.label, bb.value);

            if (bb.value >= LIMIAR && rotulo_de_parada(bb.label)) {
                parar = true;
                ESP_LOGW(TAG, "Parada: %s", bb.label);
            }
        }

#else

        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            const char *rotulo = result.classification[i].label;
            float confianca = result.classification[i].value;

            ESP_LOGI(TAG, "Classe: %s | confianca: %.2f", rotulo, confianca);

            if (confianca >= LIMIAR && rotulo_de_parada(rotulo)) {
                parar = true;
            }
        }

#endif

        if (parar) {
            parar_carrinho();
        } else {
            enviar_comando("CLEAR");
            ESP_LOGI(TAG, "Nenhum objeto de parada detectado");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}