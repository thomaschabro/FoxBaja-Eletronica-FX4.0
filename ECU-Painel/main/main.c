/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a master node in a TWAI network. The master
 * node is responsible for initiating and stopping the transfer of data messages.
 * The example will execute multiple iterations, with each iteration the master
 * node will do the following:
 * 1) Start the TWAI driver
 * 2) Repeatedly send ping messages until a ping response from slave is received
 * 3) Send start command to slave and receive data messages from slave
 * 4) Send stop command to slave and wait for stop response from slave
 * 5) Stop the TWAI driver
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
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "unistd.h"
#include "i2c-lcd.h"
#include "driver/uart.h"



/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define PING_PERIOD_MS          250
#define NO_OF_ITERS             3
#define ITER_DELAY_MS           1000
#define RX_TASK_PRIO            8
#define TX_TASK_PRIO            9
#define CTRL_TSK_PRIO           10
#define TX_GPIO_NUM             GPIO_NUM_5
#define RX_GPIO_NUM             GPIO_NUM_4
#define EXAMPLE_TAG             "TWAI Master"

#define ID_MASTER_STOP_CMD      0x0A0
#define ID_MASTER_START_CMD     0x0A1
#define ID_MASTER_PING          0x0A2
#define ID_SLAVE_STOP_RESP      0x0B0
#define ID_SLAVE_DATA           0x0B1
#define ID_SLAVE_PING_RESP      0x0B2
#define ID_SLAVE2_STOP_RESP     0x0C0
#define ID_SLAVE2_DATA          0x0C1
#define ID_SLAVE2_PING_RESP     0x0C2
#define ID_ECU_LORA             0x0D1

// Para o Display LCD
#define I2C_MASTER_SCL_IO           GPIO_NUM_22    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0              /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000         /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUFF_DISABLE  0              /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUFF_DISABLE  0              /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000           /*!< I2C timeout ms */
#define SLAVE_ADDRESS_LCD           0x27          /*!< ESP32 slave address for LCD */

    // Para o Timer
    #define TIMER_DIVIDER             16  // Hardware timer clock divider
    #define TIMER_SCALE               (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

static const char *TAG = "ECU-Painel";

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;


typedef enum {
    TX_SEND_PINGS,
    TX_SEND_START_CMD,
    TX_SEND_STOP_CMD,
    TX_TASK_EXIT,
} tx_task_action_t;

typedef enum {
    RX_RECEIVE_PING_RESP,
    RX_RECEIVE_DATA,
    RX_RECEIVE_STOP_RESP,
    RX_TASK_EXIT,
} rx_task_action_t;

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

static const twai_message_t ping_message = {.identifier = ID_MASTER_PING, .data_length_code = 0,
                                           .ss = 1, .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};
static const twai_message_t start_message = {.identifier = ID_MASTER_START_CMD, .data_length_code = 0,
                                            .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};
static const twai_message_t stop_message = {.identifier = ID_MASTER_STOP_CMD, .data_length_code = 0,
                                           .data = {0, 0 , 0 , 0 ,0 ,0 ,0 ,0}};

static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static QueueHandle_t display_task_queue;

static SemaphoreHandle_t stop_ping_sem;
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t done_sem;

static TimerHandle_t xTimers;

int interval = 500; 
int TimerId = 1;
int potencia1 = 0;
int velocidade = 0;
int potencia3 = 0;

/* ------------------------------ Funções LCD ----------------------------- */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM; 

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUFF_DISABLE, I2C_MASTER_TX_BUFF_DISABLE, 0);
}

/* ------------------------------ Funções Timer ----------------------------- */

void vtimer_callback( TimerHandle_t pxTimer ) {

    xQueueSendFromISR(display_task_queue, &potencia1, NULL);   

}

esp_err_t set_timer(void) {

    xTimers = xTimerCreate("timer", pdMS_TO_TICKS(interval), pdTRUE, (void *)TimerId, vtimer_callback);
    
    if (xTimers == NULL) {
        
        ESP_LOGI(TAG, "Erro ao criar o timer");

    } else {
        
        if (xTimerStart(xTimers, 0) != pdPASS) {
            ESP_LOGI(TAG, "Erro ao iniciar o timer");
        }
    
    }



    return ESP_OK;
}

/* --------------------------- Tasks and Functions -------------------------- */

static void display_update_task(void *arg) {
    uint8_t data[20];

    for (;;) {
        
        // Verifica se chegou algo na queue
        int rec;
        if (xQueueReceive(display_task_queue, &rec, portMAX_DELAY) == pdTRUE) {

            // Limpando
            sprintf((char *)data, "Potencia1: %s", "       ");
            lcd_put_cur(1, 0);
            lcd_send_string((char *)data);
            lcd_put_cur(2, 0);
            lcd_send_string((char *)data);
            lcd_put_cur(3, 0);
            lcd_send_string((char *)data);

            sprintf((char *)data, "Potencia1: %d", potencia1);
            lcd_put_cur(1, 0);
            lcd_send_string((char *)data);

            sprintf((char *)data, "Velocidade: %d", velocidade);
            lcd_put_cur(2, 0);
            lcd_send_string((char *)data);

            sprintf((char *)data, "Potencia3: %d", potencia3);
            lcd_put_cur(3, 0);
            lcd_send_string((char *)data); 

        }
    
    }
}

static void twai_receive_task(void *arg)
{
    int contador_ping = 0;

    while (1) {
        rx_task_action_t action;
        xQueueReceive(rx_task_queue, &action, portMAX_DELAY);

        if (action == RX_RECEIVE_PING_RESP) {
            //Listen for ping response from slave
            while (1) {
                twai_message_t rx_msg;
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_SLAVE_PING_RESP) {
                    contador_ping++;
                }
                else if (rx_msg.identifier == ID_SLAVE2_PING_RESP) {
                    contador_ping++;
                }
                if (contador_ping >= 0) {
                    ESP_LOGI(EXAMPLE_TAG, "Recebeu os pings");
                    xSemaphoreGive(stop_ping_sem);
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
                // Printa o valor do contador_ping
                ESP_LOGI(EXAMPLE_TAG, "CONTADOR DE PING: %d", contador_ping);
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
                    float temperatura = ((float)data)/10;
                    // ESP_LOGI(EXAMPLE_TAG, "temperatura f%", temperatura);
                    printf("Temperatura: %.1f\n", temperatura);
                } 
                else if (rx_msg.identifier == ID_ECU_LORA)
                {
                    ESP_LOGI(EXAMPLE_TAG, "Recebeu mensagem do ECU-LORA");
                }
                else {
                    ESP_LOGI(EXAMPLE_TAG, "Received unexpected message");
                }
            }
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == RX_RECEIVE_STOP_RESP) {
            //Listen for stop response from slave
            while (1) {
                twai_message_t rx_msg;
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_SLAVE_STOP_RESP || rx_msg.identifier == ID_SLAVE2_STOP_RESP) {
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

        if (action == TX_SEND_PINGS) {
            //Repeatedly transmit pings
            ESP_LOGI(EXAMPLE_TAG, "Transmitting ping");
            while (xSemaphoreTake(stop_ping_sem, 0) != pdTRUE) {
                twai_transmit(&ping_message, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(PING_PERIOD_MS));
                ESP_LOGI(EXAMPLE_TAG, "PING ENVIADO");
            }
        } else if (action == TX_SEND_START_CMD) {
            //Transmit start command to slave
            twai_transmit(&start_message, portMAX_DELAY);
            ESP_LOGI(EXAMPLE_TAG, "Transmitted start command");
        } else if (action == TX_SEND_STOP_CMD) {
            //Transmit stop command to slave
            twai_transmit(&stop_message, portMAX_DELAY);
            ESP_LOGI(EXAMPLE_TAG, "Transmitted stop command");
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
    
    twai_reconfigure_alerts(TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF, NULL);

    for (int iter = 0; iter < NO_OF_ITERS; iter++) {
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(EXAMPLE_TAG, "Driver started");

        //Start transmitting pings, and listen for ping response
        // tx_action = TX_SEND_PINGS;
        // rx_action = RX_RECEIVE_PING_RESP;
        // xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        // xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

        //Send Start command to slave, and receive data messages
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        tx_action = TX_SEND_START_CMD;
        rx_action = RX_RECEIVE_DATA;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);


        //Send Stop command to slave when enough data messages have been received. Wait for stop response
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        tx_action = TX_SEND_STOP_CMD;
        rx_action = RX_RECEIVE_STOP_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
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
    // Inializando o I2C/LCD
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C inicializado com sucesso!");

    lcd_init();
    lcd_put_cur(0,0);
    lcd_send_string("ECU-Painel");    

    // Criando timer para atualizar o LCD
    set_timer();    

    //Create tasks, queues, and semaphores
    display_task_queue = xQueueCreate(1, sizeof(int));
    rx_task_queue      = xQueueCreate(1, sizeof(rx_task_action_t));
    tx_task_queue      = xQueueCreate(1, sizeof(tx_task_action_t));
    ctrl_task_sem      = xSemaphoreCreateBinary();
    stop_ping_sem      = xSemaphoreCreateBinary();
    done_sem           = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(display_update_task, "Atualiza o painel", 4096, NULL, RX_TASK_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, 1);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, 1);
    xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, CTRL_TSK_PRIO, NULL, 1);

    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");

    xSemaphoreGive(ctrl_task_sem);              //Start control task
    xSemaphoreTake(done_sem, portMAX_DELAY);    //Wait for completion

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vQueueDelete(rx_task_queue);
    vQueueDelete(tx_task_queue);
    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(stop_ping_sem);
    vSemaphoreDelete(done_sem);
}