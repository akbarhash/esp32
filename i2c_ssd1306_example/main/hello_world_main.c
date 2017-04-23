/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "xi2c.h"
#include "fonts.h"
#include "ssd1306.h"

#define BLINK_GPIO 2

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a BH1750 light sensor(GY-30 module) for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP32 chip.
 *
 * Pin assignment:
 *
 * - slave :
 *    GPIO25 is assigned as the data signal of i2c slave port
 *    GPIO26 is assigned as the clock signal of i2c slave port
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect GPIO18 with GPIO25
 * - connect GPIO19 with GPIO26
 * - connect sda/scl of sensor with GPIO18/GPIO19
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 * - i2c master(ESP32) will write data to i2c slave(ESP32).
 * - i2c master(ESP32) will read data from i2c slave(ESP32).
 */

#define DELAY_TIME_BETWEEN_ITEMS_MS   1234 /*!< delay time between different test items */

#define I2C_EXAMPLE_MASTER_SCL_IO    22    /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO    21    /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

xSemaphoreHandle print_mux;

static void i2c_test_task(void)
{
    /*
    int i = 0;
    int ret;
    uint32_t task_idx = (uint32_t) arg;
    uint8_t* data = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_wr = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_rd = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t sensor_data_h, sensor_data_l;

    while (1) {
        ret = i2c_example_master_sensor_test( I2C_EXAMPLE_MASTER_NUM, &sensor_data_h, &sensor_data_l);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("*******************\n");
        printf("TASK[%d]  MASTER READ SENSOR( BH1750 )\n", task_idx);
        printf("*******************\n");
        if (ret == ESP_OK) {
            printf("data_h: %02x\n", sensor_data_h);
            printf("data_l: %02x\n", sensor_data_l);
            printf("sensor val: %f\n", ( sensor_data_h << 8 | sensor_data_l ) / 1.2);
        } else {
            printf("No ack, sensor not connected...skip...\n");
        }
        xSemaphoreGive(print_mux);
        vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);

        //---------------------------------------------------
        for (i = 0; i < DATA_LENGTH; i++) {
            data[i] = i;
        }
        size_t d_size = i2c_slave_write_buffer(I2C_EXAMPLE_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
        if (d_size == 0) {
            printf("i2c slave tx buffer full\n");
            ret = i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM, data_rd, DATA_LENGTH);
        } else {
            ret = i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM, data_rd, RW_TEST_LENGTH);
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("*******************\n");
        printf("TASK[%d]  MASTER READ FROM SLAVE\n", task_idx);
        printf("*******************\n");
        printf("====TASK[%d] Slave buffer data ====\n", task_idx);
        disp_buf(data, d_size);X_Error
        if (ret == ESP_OK) {
            printf("====TASK[%d] Master read ====\n", task_idx);
            disp_buf(data_rd, d_size);
        } else {
            printf("Master read slave error, IO not connected...\n");
        }
        xSemaphoreGive(print_mux);
        vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
        //---------------------------------------------------
        int size;
        for (i = 0; i < DATA_LENGTH; i++) {
            data_wr[i] = i + 10;
        }
        //we need to fill the slave buffer so that master can read later
        ret = i2c_example_master_write_slave( I2C_EXAMPLE_MASTER_NUM, data_wr, RW_TEST_LENGTH);
        if (ret == ESP_OK) {
            size = i2c_slave_read_buffer( I2C_EXAMPLE_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("*******************\n");
        printf("TASK[%d]  MASTER WRITE TO SLAVE\n", task_idx);
        printf("*******************\n");
        printf("----TASK[%d] Master write ----\n", task_idx);
        disp_buf(data_wr, RW_TEST_LENGTH);
        if (ret == ESP_OK) {
            printf("----TASK[%d] Slave read: [%d] bytes ----\n", task_idx, size);
            disp_buf(data, size);
        } else {
            printf("TASK[%d] Master write slave error, IO not connected....\n", task_idx);
        }
        xSemaphoreGive(print_mux);
        vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
    }*/
    SSD1306_GotoXY(40, 4);
    SSD1306_Puts("ESP32", &Font_7x10, SSD1306_COLOR_WHITE);
    
    /* Go to location X = 20, Y = 25 */
    SSD1306_GotoXY(8, 25);
    SSD1306_Puts("I2C SSD1306 OLED", &Font_7x10, SSD1306_COLOR_WHITE);
    
    /* Go to location X = 15, Y = 45 */
    SSD1306_GotoXY(20, 45);
    SSD1306_Puts("Akbar Hashim", &Font_7x10, SSD1306_COLOR_WHITE);
    
    /* Update screen, send changes to LCD */
    SSD1306_UpdateScreen();

    while (1) {
        /* Invert pixels */
        SSD1306_ToggleInvert();
        
        /* Update screen */
        SSD1306_UpdateScreen();
        
        /* Make a little delay */
        vTaskDelay(50);
    }

}

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

void app_main()
{
    print_mux = xSemaphoreCreateMutex();
    i2c_example_master_init();
    SSD1306_Init();

    xTaskCreate(&blink_task, "blink_task", 512, NULL, 5, NULL);
    xTaskCreate(i2c_test_task, "i2c_test_task", 1024, NULL, 10, NULL);
}
