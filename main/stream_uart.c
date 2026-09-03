#include "stream_uart.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "driver/usb_serial_jtag_vfs.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "StreamUART";

// Protocol header for frame transmission
#define STREAM_PROTOCOL_MAGIC 0x4D4A4754u  // "TGJM" in little-endian byte order
#define STREAM_PROTOCOL_MAGIC2 0xA55AA55Au
#define STREAM_STATUS_FRAME_NUM 0xFFFFFFFFu

typedef struct {
    uint32_t magic;      // STREAM_PROTOCOL_MAGIC
    uint32_t magic2;     // STREAM_PROTOCOL_MAGIC2
    uint32_t frame_num;
    uint32_t frame_size;
} __attribute__((packed)) stream_header_t;

static uint32_t frame_counter = 0;

esp_err_t stream_uart_init(void)
{
    // Keep binary stream intact: avoid CRLF/CR translation on console writes.
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);

    // Use unbuffered stdout so frame chunks are pushed immediately over USB serial.
    setvbuf(stdout, NULL, _IONBF, 0);
    return ESP_OK;
}

esp_err_t stream_uart_send_frame(camera_fb_t *fb)
{
    if (!fb || fb->len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Create stream header
    stream_header_t header = {
        .magic = STREAM_PROTOCOL_MAGIC,
        .magic2 = STREAM_PROTOCOL_MAGIC2,
        .frame_num = frame_counter++,
        .frame_size = fb->len,
    };

    // Header layout: 4-byte magic + 4-byte magic2 + 4-byte frame number + 4-byte frame size = 16 bytes.
    size_t written = fwrite(&header, 1, sizeof(stream_header_t), stdout);
    if (written != sizeof(stream_header_t)) {
        return ESP_FAIL;
    }

    // Send JPEG data
    written = fwrite(fb->buf, 1, fb->len, stdout);
    if (written != fb->len) {
        return ESP_FAIL;
    }
    fflush(stdout);

    return ESP_OK;
}

void stream_uart_send_status(const char *message)
{
    if (message && message[0] != '\0') {
        stream_header_t header = {
            .magic = STREAM_PROTOCOL_MAGIC,
            .magic2 = STREAM_PROTOCOL_MAGIC2,
            .frame_num = STREAM_STATUS_FRAME_NUM,
            .frame_size = (uint32_t)strlen(message),
        };

        size_t written = fwrite(&header, 1, sizeof(stream_header_t), stdout);
        if (written == sizeof(stream_header_t)) {
            (void)fwrite(message, 1, header.frame_size, stdout);
            fflush(stdout);
        }
    }

    (void)TAG;
}
