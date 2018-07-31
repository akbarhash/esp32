#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"

#include "esp_system.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "bme280.h"
#include "bme280_defs.h"

#define BME280_FLOAT_ENABLE

#define I2C_EXAMPLE_MASTER_SCL_IO 22        /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO 21        /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define ACK_CHECK_EN 0x1                    /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                   /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                         /*!< I2C ack value */
#define NACK_VAL 0x1                        /*!< I2C nack value */

xSemaphoreHandle print_mux;

/*
* @brief i2c master initialization
 */
void i2cInit()
{
    uint8_t i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
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

static esp_err_t XI2CWrite(i2c_port_t i2c_num, uint8_t i2c_add, uint8_t *data_wr, size_t size)
{

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_add << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t XI2CRead(i2c_port_t i2c_num, uint8_t i2c_add, uint8_t *data_rd, size_t size)
{
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_add << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1)
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t X_WriteMulti(i2c_port_t i2c_num, uint8_t i2c_add, uint8_t index, uint32_t count, uint8_t *data_wr)
{
    uint8_t XI2CBuffer[20];
    XI2CBuffer[0] = index;
    memcpy(&XI2CBuffer[1], data_wr, count);
    esp_err_t ret = XI2CWrite(i2c_num, i2c_add, XI2CBuffer, count + 1);
    return ret;
}

esp_err_t X_ReadMulti(i2c_port_t i2c_num, uint8_t i2c_add, uint8_t index, uint32_t count, uint8_t *data_rd)
{
    esp_err_t ret;
    ret = XI2CWrite(i2c_num, i2c_add, &index, 0x1);
    if (ret == ESP_FAIL)
        return ret;
    ret = XI2CRead(i2c_num, i2c_add, data_rd, count);
    return ret;
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    esp_err_t ret = X_ReadMulti(I2C_EXAMPLE_MASTER_NUM, dev_id, reg_addr, len, reg_data);
    vTaskDelay( 10 / portTICK_PERIOD_MS);
    return ret;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    esp_err_t ret = X_WriteMulti(I2C_EXAMPLE_MASTER_NUM, dev_id, reg_addr, len, reg_data);
    vTaskDelay( 10 / portTICK_PERIOD_MS);
    return ret;
}

void user_delay_ms(uint32_t period)
{
    vTaskDelay( period / portTICK_PERIOD_MS);
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
    printf("%0.2f, %0.2f, %0.2f\r\n", comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
    printf("%ld, %ld, %ld\r\n", comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;
    dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, dev);
    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

    printf("Temperature, Pressure, Humidity\r\n");
    while (1)
    {
        /* Delay while the sensor completes a measurement */
        dev->delay_ms(70);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        print_sensor_data(&comp_data);
    }

    return rslt;
}

int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);

    printf("Temperature, Pressure, Humidity\r\n");
    /* Continuously stream sensor data */
    while (1)
    {
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        /* Wait for the measurement to complete and print data @25Hz */
        dev->delay_ms(40);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        print_sensor_data(&comp_data);
    }
    return rslt;
}

void app_main()
{
    print_mux = xSemaphoreCreateMutex();

    i2cInit();
    struct bme280_dev dev;
    int8_t rslt;

    dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev);
    if(!rslt)
        stream_sensor_data_normal_mode(&dev);
}
