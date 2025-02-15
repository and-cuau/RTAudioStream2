#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "arm_math.h"
#include "esp_dsp.h"

// #include <string.h>
// #include <sys/param.h>
// #include "lwip/sockets.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "esp_wifi.h"
// #include "nvs_flash.h"

// float32_t inputSignal[FFT_SIZE];
// float32_t outputSignal[FFT_SIZE];

#define FFT_SIZE 1024
float32_t inputSignal[FFT_SIZE];
float32_t outputSignal[FFT_SIZE];

void process_audio() {
    dsps_fft2r_fc32(input, FFT_SIZE); 
}

// Task 1: Prints "Task 1 running"
void task_server(void *pvParameter) {
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    uint8_t audio_buffer[BUFFER_SIZE];

    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    listen(server_sock, 1);
    ESP_LOGI(TAG, "TCP server listening on port %d", PORT);

    client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &addr_len);

    while (1) {
        int len = recv(client_sock, audio_buffer, sizeof(audio_buffer), 0);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1000 ms (1 second)

        if (len <= 0) {
            ESP_LOGE(TAG, "Failed to receive data or client disconnected");
            break;
        }

        i2s_write(I2S_NUM, audio_buffer, len, NULL, portMAX_DELAY);
    }

    close(client_sock);
    close(server_sock);
}

static const char *TAG = "TCP_SERVER";

// Task 2: Prints "Task 2 running"
void task_stream(void *pvParameter) {
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char rx_buffer[128];

    if (server_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
    }

    while (1) {
        printf("Task 2 running\n");
        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay for 2000 ms (2 seconds)
    }
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
        i2s_read(I2S_NUM, &i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, portMAX_DELAY);

    }

    // Create Task 1
    // xTaskCreate(&task_server, "Task1", 2048, NULL, 1, NULL);

    // Create Task 2
    // xTaskCreate(&task_stream, "Task2", 2048, NULL, 1, NULL);
}
