#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "arm_math.h"

#include "driver/i2s.h"
#include "lwip/sockets.h"

 #include <string.h>
 #include <sys/param.h>
 #include "lwip/sockets.h"
 #include "esp_log.h"
 #include "esp_err.h"
 #include "esp_wifi.h"
 #include "nvs_flash.h"

//#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#define HOST_IP_ADDR "fe80::201b:9254:1705:de19%2"

// float32_t inputSignal[FFT_SIZE];
// float32_t outputSignal[FFT_SIZE];

#define FFT_SIZE 1024
float32_t inputSignal[FFT_SIZE];
float32_t outputSignal[FFT_SIZE];

#define I2S_NUM (0) // I2S port number, can be 0 or 1
#define I2S_BCK_PIN  (26)  // Bit Clock Pin
#define I2S_WS_PIN   (25)  // Word Select (LRCK) Pin
#define I2S_DO_PIN   (-1)  // Data Out Pin, not used for input
#define I2S_DI_PIN   (22)  // Data In Pin (microphone data input)

const char *payload = "Hello, Server!";

void I2S_init() {
       i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,           // Master mode, receive mode
        .sample_rate = 44100,                           // Set sample rate, e.g., 44.1kHz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,    // 16-bit resolution
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,   // Mono microphone
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,// I2S MSB format
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,       // Interrupt level
        .dma_buf_count = 8,                             // Number of DMA buffers
        .dma_buf_len = 64,                              // Buffer length
        .use_apll = false,                              // APLL not used
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_PIN,                      // Bit Clock pin
        .ws_io_num = I2S_WS_PIN,                        // Word Select pin
        .data_out_num = I2S_DO_PIN,                     // Data Out pin (not used)
        .data_in_num = I2S_DI_PIN,                      // Data In pin
    };

    // Install and configure the I2S driver
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM, &pin_config));
}



// Task 1: Prints "Task 1 running"
void task1(void *pvParameter) {
    while (1) {
        printf("Task 1 running\n");
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1000 ms (1 second)
    }
}

static const char *TAG = "TCP_SERVER";

// Task 2: Prints "Task 2 running"
void task_client(void *pvParameter) {
   char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = AF_INET; // 0 
    int ip_protocol = IPPROTO_TC; // 0 

    int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        break;
    }

     while (1) {
            int err = send(sock, payload, strlen(payload), 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

     }

    close(sock);
    vTaskDelete(NULL);
}

// Main function: Creates the FreeRTOS tasks
void app_main(void) {
    printf("ESP32 FreeRTOS Example\n");

    I2S_init();
    xTaskCreate(task_client, "tcp_client_task", 4096, NULL, 5, NULL);

    int32_t i2s_read_buff[1024];

    while (1) {
        // Read audio data from microphone via I2S
        size_t bytes_read;
        // i2s_read(I2S_NUM, &i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, portMAX_DELAY);
        i2s_read(I2S_NUM, (void *)i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, portMAX_DELAY);

    }

}
