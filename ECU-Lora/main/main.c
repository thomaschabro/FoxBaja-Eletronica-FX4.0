/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a slave node in a TWAI network. The slave
 * node is responsible for sending data messages to the master. The example will
 * execute multiple iterations, with each iteration the slave node will do the
 * following:
 * 1) Start the TWAI driver
 * 2) Listen for ping messages from master, and send ping response
 * 3) Listen for start command from master
 * 4) Send data messages to master and listen for stop command
 * 5) Send stop response to master
 * 6) Stop the TWAI driver
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "string.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define DATA_PERIOD_MS                  50
#define ITER_DELAY_MS                   1000
#define RX_TASK_PRIO                    8       //Receiving task priority
#define TX_TASK_PRIO                    9       //Sending task priority
#define CTRL_TSK_PRIO                   10      //Control task priority
#define TX_GPIO_NUM                     GPIO_NUM_5
#define RX_GPIO_NUM                     GPIO_NUM_4
#define EXAMPLE_TAG                     "TWAI Slave"

#define ADC_PIN                         34

#define ID_MASTER_STOP_CMD              0x0A0
#define ID_MASTER_START_CMD             0x0A1
#define ID_MASTER_PING                  0x0A2
#define ID_SLAVE_STOP_RESP              0x0B0
#define ID_SLAVE_DATA                   0x0B1
#define ID_SLAVE_PING_RESP              0x0B2
#define ID_SLAVE2_STOP_RESP             0x0C0
#define ID_SLAVE2_DATA                  0x0C1
#define ID_SLAVE2_PING_RESP             0x0C2

typedef enum {
    TX_SEND_PING_RESP,
    TX_SEND_STOP_RESP,
    TX_TASK_EXIT,
    TX_SEND_DATA,
} tx_task_action_t;

typedef enum {
    RX_RECEIVE_PING,
    RX_RECEIVE_START_CMD,
    RX_RECEIVE_STOP_CMD,
    RX_TASK_EXIT,
    RX_RECEIVE_DATA,
} rx_task_action_t;

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


static const twai_message_t ping_resp = {.identifier = ID_SLAVE2_PING_RESP, .data_length_code = 0,
                                        .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};
static const twai_message_t stop_resp = {.identifier = ID_SLAVE2_STOP_RESP, .data_length_code = 0,
                                        .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};
//Data bytes of data message will be initialized in the transmit task
static twai_message_t data_message = {.identifier = ID_SLAVE2_DATA, .data_length_code = 4,
                                     .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};

static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t stop_data_sem;
static SemaphoreHandle_t done_sem;

int potencia1;
int potencia3 = 15;
int velocidade;

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    while (1) {
        ESP_LOGI(EXAMPLE_TAG, "Waiting for rx task action");
        rx_task_action_t action;
        xQueueReceive(rx_task_queue, &action, portMAX_DELAY);
        if (action == RX_RECEIVE_PING) {
            //Listen for pings from master
            twai_message_t rx_msg;
            while (1) {
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_MASTER_PING) {
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_RECEIVE_START_CMD) {
            //Listen for start command from master
            twai_message_t rx_msg;
            while (1) {
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_MASTER_START_CMD) {
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_RECEIVE_STOP_CMD) {
            //Listen for stop command from master
            twai_message_t rx_msg;
            while (1) {
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_MASTER_STOP_CMD) {
                    xSemaphoreGive(stop_data_sem);
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_RECEIVE_DATA) {
            //Receive data messages from slave
            while (1) {
                twai_message_t rx_msg;
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_SLAVE_DATA) {

                    uint32_t data1 = 0;
                    for (int i = 0; i < 4; i++) {
                        data1 |= (rx_msg.data[i] << (i * 8));
                    }
                    potencia1 = (int) data1;
                    ESP_LOGI(EXAMPLE_TAG, "Received data1 value %d", potencia1);

                    // Ler segunda parte do dado
                    uint32_t data2 = 0;
                    for (int i = 4; i < rx_msg.data_length_code; i++) {
                        data2 |= (rx_msg.data[i] << ((i-4) * 8));
                    }
                    velocidade = (int) data2;
                    ESP_LOGI(EXAMPLE_TAG, "Received velo value %d", velocidade);
                    ESP_LOGI(EXAMPLE_TAG, " = ");

                // ESP_LOGI(EXAMPLE_TAG, "Received data value %"PRIu32, data);
                } else if (rx_msg.identifier == ID_SLAVE2_DATA) {
                    uint32_t data = 0;
                    for (int i = 0; i < rx_msg.data_length_code; i++) {
                        data |= (rx_msg.data[i] << (i * 8));
                    }
                    // potencia3 = (int) data;
                    // ESP_LOGI(EXAMPLE_TAG, "Received data value %"PRIu32, data);
                } else {
                    ESP_LOGI(EXAMPLE_TAG, "Received unexpected message");
                }
            }
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == RX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}


// char* message;
static void twai_transmit_task(void *arg)
{
    ESP_LOGI(EXAMPLE_TAG, "Transmit task started");
    while (1) {

        tx_task_action_t action;
        xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

        if (action == TX_SEND_PING_RESP) {
            //Transmit ping response to master
            twai_transmit(&ping_resp, portMAX_DELAY);
            ESP_LOGI(EXAMPLE_TAG, "Transmitted ping response");
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == TX_SEND_STOP_RESP) {
            //Transmit stop response to master
            twai_transmit(&stop_resp, portMAX_DELAY);
            ESP_LOGI(EXAMPLE_TAG, "Transmitted stop response");
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == TX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_control_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_task_action_t tx_action;
    rx_task_action_t rx_action;

    for (;;) {
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(EXAMPLE_TAG, "Driver started");

        // Listen of pings from master   
        ESP_LOGI(EXAMPLE_TAG, "Waiting for ping from master");
        rx_action = RX_RECEIVE_PING;
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        ESP_LOGI(EXAMPLE_TAG, "Received ping from master");

        // // Send ping response
        tx_action = TX_SEND_PING_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        /* 
        HANDSHAKE FEITO COM SUCESSO
        AGORA ESPERA POR MSG QUE PODE COMECAR
        */

        //Listen for start command
        rx_action = RX_RECEIVE_START_CMD;
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Start receiving data messages
        ESP_LOGI(EXAMPLE_TAG, "Sending uart messages");
        rx_action = RX_RECEIVE_DATA;
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Send stop response
        tx_action = TX_SEND_STOP_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Wait for bus to become free
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        while (status_info.msgs_to_tx > 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            twai_get_status_info(&status_info);
        }

        ESP_ERROR_CHECK(twai_stop());
        ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
        vTaskDelay(pdMS_TO_TICKS(ITER_DELAY_MS));
    }

    //Stop TX and RX tasks
    tx_action = TX_TASK_EXIT;
    rx_action = RX_TASK_EXIT;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

    //Delete Control task
    xSemaphoreGive(done_sem);
    vTaskDelete(NULL);
}

void app_main(void)
{

    // //Add short delay to allow master it to initialize first
    // for (int i = 3; i > 0; i--) {
    //     printf("Slave starting in %d\n", i);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    // const uart_port_t uart_num = UART_NUM_0;
    // uart_config_t uart_config = {
    //     .baud_rate = 115200,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .rx_flow_ctrl_thresh = 122,
    //     .source_clk = UART_SCLK_DEFAULT,
    // };
    // // Configure UART parameters
    // ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // const int uart_buffer_size = (1024 * 2);
    // QueueHandle_t uart_queue;
    // ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size, 
    //                                     uart_buffer_size, 10, &uart_queue, 0));


    //Create semaphores and tasks
    tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
    rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
    // uart_queue = xQueueCreate(10, sizeof(char));
    ctrl_task_sem = xSemaphoreCreateBinary();
    stop_data_sem  = xSemaphoreCreateBinary();
    done_sem  = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);

    //Install TWAI driver, trigger tasks to start
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");

    xSemaphoreGive(ctrl_task_sem);              //Start Control task
    xSemaphoreTake(done_sem, portMAX_DELAY);    //Wait for tasks to complete

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(stop_data_sem);
    vSemaphoreDelete(done_sem);
    vQueueDelete(tx_task_queue);
    vQueueDelete(rx_task_queue);
}