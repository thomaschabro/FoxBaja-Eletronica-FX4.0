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
#include "freertos/timers.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/adc.h"
#include "driver/gpio.h"

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

// Para o Timer
#define TIMER_DIVIDER             16  // Hardware timer clock divider
#define TIMER_SCALE               (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#define VEL_PIN                         GPIO_NUM_27

typedef enum {
    TX_SEND_PING_RESP,
    TX_SEND_DATA,
    TX_SEND_STOP_RESP,
    TX_TASK_EXIT,
} tx_task_action_t;

typedef enum {
    RX_RECEIVE_PING,
    RX_RECEIVE_START_CMD,
    RX_RECEIVE_STOP_CMD,
    RX_TASK_EXIT,
} rx_task_action_t;

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


static const twai_message_t ping_resp = {.identifier = ID_SLAVE_PING_RESP, .data_length_code = 0,
                                        .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};
static const twai_message_t stop_resp = {.identifier = ID_SLAVE_STOP_RESP, .data_length_code = 0,
                                        .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};
//Data bytes of data message will be initialized in the transmit task
static twai_message_t data_message = {.identifier = ID_SLAVE_DATA, .data_length_code = 8, 
                                        .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};


static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static QueueHandle_t vel_task_queue;

static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t stop_data_sem;
static SemaphoreHandle_t done_sem;

static TimerHandle_t xTimers;

uint32_t potencia1;
uint32_t velocidade;

int interval = 100;  // ms 
int TimerId = 1;
int raio = 45 * 1e-3; // 45mm
/* ------------------------------ Funções Timer ----------------------------- */
void vtimer_callback( TimerHandle_t pxTimer ) {

    xQueueSendFromISR(vel_task_queue, &potencia1, NULL);   

}

esp_err_t set_timer(void) {

    xTimers = xTimerCreate("timer", pdMS_TO_TICKS(interval), pdTRUE, (void *)TimerId, vtimer_callback);
    
    if (xTimers == NULL) {
        

    } else {
        
        if (xTimerStart(xTimers, 0) != pdPASS) {
        }
    
    }
    return ESP_OK;
}


/* --------------------------- Tasks and Functions -------------------------- */

static void read_rotation(void *arg) {

    gpio_set_direction(VEL_PIN, GPIO_MODE_INPUT);

    uint32_t n = 0;

    while (1) {

        n += gpio_get_level(VEL_PIN);
        int x;
        // if (xQueueReceive(vel_task_queue, &x, portMAX_DELAY) == pdTRUE) {

            

        // }
        ESP_LOGI(EXAMPLE_TAG, "n = %d", n);
    
    }

}


static void twai_receive_task(void *arg)
{
    while (1) {
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
        } else if (action == RX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg)
{
    while (1) {
        tx_task_action_t action;
        xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

        if (action == TX_SEND_PING_RESP) {
            //Transmit ping response to master
            twai_transmit(&ping_resp, portMAX_DELAY);
            ESP_LOGI(EXAMPLE_TAG, "Transmitted ping response");
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == TX_SEND_DATA) {
            //Transmit data messages until stop command is received
            ESP_LOGI(EXAMPLE_TAG, "Start transmitting data");
            while (1) {
                //FreeRTOS tick count used to simulate sensor data -> DEVEMOS MUDAR ESSA PARTE PARA PEGAR DADOS DO SENSOR
                // uint32_t sensor_data = xTaskGetTickCount();
                potencia1 = adc1_get_raw(ADC1_CHANNEL_6);

                // Criando a "data" da mensagem com os valores de potencia1 e potencia2
                for (int i = 0; i < 4; i++) {
                    data_message.data[i] = (potencia1 >> (i * 8)) & 0xFF;
                }
                for (int i = 4; i < 8; i++) {
                    data_message.data[i] = (velocidade >> ((i - 4) * 8)) & 0xFF;
                }

                twai_transmit(&data_message, portMAX_DELAY);
                ESP_LOGI(EXAMPLE_TAG, "Transmitted data1 value %"PRIu32, potencia1);
                ESP_LOGI(EXAMPLE_TAG, "Transmitted velo value %"PRIu32, velocidade);
                ESP_LOGI(EXAMPLE_TAG, " = ");
                vTaskDelay(pdMS_TO_TICKS(DATA_PERIOD_MS));
                if (xSemaphoreTake(stop_data_sem, 0) == pdTRUE) {
                    break;
                }
            }
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

        //Listen of pings from master   
        rx_action = RX_RECEIVE_PING;
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Send ping response
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

        //Start sending data messages and listen for stop command
        tx_action = TX_SEND_DATA;
        rx_action = RX_RECEIVE_STOP_CMD;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
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
    //Add short delay to allow master it to initialize first
    for (int i = 3; i > 0; i--) {
        printf("Slave starting in %d\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    set_timer();    

    //Create semaphores and tasks
    tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
    rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
    vel_task_queue = xQueueCreate(1, sizeof(int));
    ctrl_task_sem = xSemaphoreCreateBinary();
    stop_data_sem  = xSemaphoreCreateBinary();
    done_sem  = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(read_rotation, "read_rotation", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);

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