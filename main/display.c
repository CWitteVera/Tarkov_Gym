#include "stream_uart.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "StreamUART";

// Protocol header for frame transmission
#define STREAM_FRAME_START 0xFFD8     // JPEG SOI marker
#define STREAM_FRAME_END 0xFFD9       // JPEG EOI marker
#define STREAM_PROTOCOL_MAGIC 0x4A4D  // "JM" - JPEG Magic

#define UART_NUM UART_NUM_0
#define UART_BAUD_RATE 921600

typedef struct {
    uint16_t magic;      // STREAM_PROTOCOL_MAGIC
    uint32_t frame_num;
    uint32_t frame_size;
} stream_header_t;

static uint32_t frame_counter = 0;

esp_err_t stream_uart_init(void)
{
    ESP_LOGI(TAG, "UART streaming initialized at %d baud", UART_BAUD_RATE);
    return ESP_OK;
}

esp_err_t stream_uart_send_frame(camera_fb_t *fb)
{
    if (!fb || fb->len == 0) {
        ESP_LOGW(TAG, "Invalid frame buffer");
        return ESP_ERR_INVALID_ARG;
    }

    // Create stream header
    stream_header_t header = {
        .magic = STREAM_PROTOCOL_MAGIC,
        .frame_num = frame_counter++,
        .frame_size = fb->len,
    };

    // Send header (8 bytes)
    int written = uart_write_bytes(UART_NUM, (const char *)&header, sizeof(stream_header_t));
    if (written != sizeof(stream_header_t)) {
        ESP_LOGW(TAG, "Failed to send header: wrote %d bytes", written);
        return ESP_FAIL;
    }

    // Send JPEG data
    written = uart_write_bytes(UART_NUM, (const char *)fb->buf, fb->len);
    if (written != fb->len) {
        ESP_LOGW(TAG, "Failed to send frame: wrote %d/%d bytes", written, fb->len);
        return ESP_FAIL;
    }

    return ESP_OK;
}

void stream_uart_send_status(const char *message)
{
    ESP_LOGI(TAG, "Status: %s", message);
}
