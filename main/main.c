/****************** INCLUDE ******************/
#include "sdkconfig.h"
#include <stdio.h>
#include "stdint.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_cpu.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
//#include "driver/i2c.h"

/***************** CUSTOM INCLUDE *****************/
#include "encoder.h"
#include "eeprom.h"
#include "lcd_i80_bus.h"
#include "ili9488_i80.h"
//#include "bmp180.h"
// #include "image.h"
/***************** GLOBAL VARIABLES ****************/
uint8_t cnt = 0;

/******************* QUEUE HANDLE *****************/
QueueHandle_t queue_data_print;
typedef struct
{
    bool led_state;
    uint16_t led_pwm;
    uint16_t press;
    float temp;

} queue_t;
/******************* CONFIG TIMER *********************/
static void timer_init_(void)
{
    timer_config_t config_tim;
    config_tim.divider = 80;
    config_tim.counter_dir = TIMER_COUNT_UP;
    config_tim.counter_en = TIMER_START;
    config_tim.alarm_en = TIMER_ALARM_DIS;
    config_tim.auto_reload = TIMER_AUTORELOAD_DIS;
    // config_tim.intr_type =  TIMER_INTR_LEVEL;
    timer_init(TIMER_GROUP_0, TIMER_0, &config_tim);
}
/******************* CONFIG I2C *********************/
static void i2c_init_(uint8_t i2c_sda, uint8_t i2c_scl)
{
    i2c_config_t config_i2c;
    config_i2c.mode = I2C_MODE_MASTER;
    config_i2c.sda_io_num = i2c_sda;
    config_i2c.scl_io_num = i2c_scl;
    config_i2c.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config_i2c.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config_i2c.master.clk_speed = 400 * 1000; // kHz
    config_i2c.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &config_i2c);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}
/******************* MAIN TASK *****************************/
void main_task(void *pvParameters)
{
    while (1)
    {
        encoder();

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
/******************* PRINT TASK *****************************/
void print_task(void *pvParameters)
{
    uint16_t cnt = 1;
    while (1)
    {
        if (cnt == 1)
        {
            lcd_fill_screen(LCD_RED);
        }
        if (cnt == 2)
        {
            lcd_fill_screen(LCD_GREEN);
        }
        if (cnt == 3)
        {
            lcd_fill_screen(LCD_BLUE);
        }
        cnt++;
        if (cnt == 4)
        {
            cnt = 1;
        }
        // lcd_fill_rect(0, 0, 300, 300, LCD_BLUE);
        // lcd_fill_screen(LCD_BLUE);

        ESP_LOGW("Encoder", "wh=%u", cnt);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
/********************* APP_MAIN ******************************/
void app_main(void)
{
    timer_init_();
    i2c_init_(21, 22);
    encoder_init(34, 39, 36);
    eeprom_init(I2C_NUM_0);
    lcd_i80_bus_init(8);
    ili9488_i80_init(320, 480);
    lcd_fill_screen(LCD_BLACK);

    /******************* TASK CREATE *****************************/
    xTaskCreate(main_task, "main_task", 3000, NULL, 1, NULL);
    xTaskCreate(print_task, "print_task", 2000, NULL, 1, NULL);

    /******************* QUEUE CREATE *****************************/
    queue_data_print = xQueueCreate(1, sizeof(queue_t));
}
