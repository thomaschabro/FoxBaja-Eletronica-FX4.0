#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/queue.h"

#define LORA_SLAVE 2
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024
#define TASK_MEMORY 1024 * 2
#define RGB_LED 8
#define DELAY 500
static const char *TAG = "LORA MASTER";
static QueueHandle_t uart_queue;

static void uart_task(void *pvParameters)
{   
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    while (1)
    {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
        {
            bzero(data, BUF_SIZE);

            switch (event.type)
            {
            case UART_DATA:
                uart_read_bytes(UART_NUM, data, event.size, pdMS_TO_TICKS(100));                

                ESP_LOGI(TAG, "Data received: %s", data);             

                break;

            default:
                break;
            }
        }
    }
}

static void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM, &uart_config);

    uart_set_pin(UART_NUM, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
   
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 5, &uart_queue, 0);
    xTaskCreate(uart_task, "uart_task", TASK_MEMORY, NULL, 5, NULL);

    ESP_LOGI(TAG, "init uart completed!");
}

static void init_lora_896(void)
{
    char *data = "AT\r\n";
    uart_write_bytes(UART_NUM, (const char *)data, strlen(data));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data = "AT+ADDRESS=1\r\n";
    uart_write_bytes(UART_NUM, (const char *)data, strlen(data));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data = "AT+ADDRESS?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data, strlen(data));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data = "AT+NETWORKID=10\r\n";
    uart_write_bytes(UART_NUM, (const char *)data, strlen(data));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data = "AT+NETWORKID?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data, strlen(data));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data = "AT+CPIN=AABCF002EEDCFA90FABC0002EEDCAAF0\r\n";
    uart_write_bytes(UART_NUM, (const char *)data, strlen(data));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data = "AT+CPIN?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data, strlen(data));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    ESP_LOGI(TAG, "init_lora_896 completed!");
}

static void lora_send(int address, const char *data)
{
    char command_data[300] = "";
    char command[] = "AT+SEND=";
    char separator[] = ",";
    char end_of_line[] = "\r\n";
    // "AT+SEND=2,11,000|255|000"
    snprintf(command_data, 200, "%s%d%s%d%s%s%s", command, address, separator, strlen(data), separator, data, end_of_line);
    ESP_LOGI(TAG, "command_data : %s", command_data);

    uart_write_bytes(UART_NUM, (const char *)command_data, strlen(command_data));
    vTaskDelay(pdMS_TO_TICKS(DELAY * 2));
}

void app_main(void)
{
    init_uart();
    init_lora_896();    

    uint8_t count = 0;
    while (true)
    {
        count++;
        count = count > 2 ? 0 : count;

        switch (count)
        {
        case 0:
            vTaskDelay(pdMS_TO_TICKS(500));
            lora_send(LORA_SLAVE, "255|000|000");
            break;
        case 1:
            lora_send(LORA_SLAVE, "000|255|000");
            break;
        case 2:
            lora_send(LORA_SLAVE, "000|000|255");
            break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
