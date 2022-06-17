#include "sdkconfig.h"
#include <stdio.h>
#include "stdint.h"
#include "string.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
//#include "driver/ledc.h"
//#include "driver/i2c.h"
//#include "driver/spi_master.h"
//#include "bmp180.h"
#include "encoder.h"
#include "eeprom.h"
#include "esp_cpu.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/i2s.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lcd_i80_bus.h"
#include "ili9488_i80.h"
// #include "image.h"

//#include "lvgl.h"
/* #include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_netif.h" */

//#include "esp_event.h"
// #include "esp_attr.h"
/******************* GPIO *****************************/
#define ENC_SW GPIO_NUM_36
#define ENC_SW_SEL GPIO_SEL_36

#define ENC_DT GPIO_NUM_39
#define ENC_DT_SEL GPIO_SEL_39

#define ENC_CLK GPIO_NUM_34
#define ENC_CLK_SEL GPIO_SEL_34

#define I2C_SCL GPIO_NUM_22
#define I2C_SDA GPIO_NUM_21

/******************* WI-FI *********************/
#define STA_SSID "ESP32"
#define STA_PASSWORD "taras1988"

/******************* SD CARD *********************/
#define TAG_SD "SD Card"
#define SDMMC_MOUNT "/sdmmc"

/******************* LCD ***********************/
#define DB0 0
#define DB1 4
#define DB2 25
#define DB3 26
#define DB4 27
#define DB5 14
#define DB6 12
#define DB7 13
#define DC 2
#define WR 15

// static esp_lcd_panel_io_handle_t io_handle = NULL;
// static esp_lcd_i80_bus_handle_t i80_bus = NULL;
// static esp_lcd_panel_handle_t panel_handle = NULL;

void set_xy(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/******************* ������� ********************/
/* void client_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("%.*s\n", evt->data_len, (char *)evt->data);
        break;
    }
    if (evt->event_id == HTTP_EVENT_ON_DATA)
        printf("%.*s\n", evt->data_len, (char *)evt->data);
} */
/******************* ����������� �������� *****************************/
QueueHandle_t queue_data_print;
// QueueHandle_t queue_rtc_print;
typedef struct
{
    bool led_state;
    uint16_t led_pwm;
    uint16_t press;
    float temp;
} queue_t;
/* void scann()
{

    // configure and run the scan process in blocking mode
    wifi_scan_config_t scan_config = {
        .ssid = 0,
        .bssid = 0,
        .channel = 0,
        .show_hidden = true};

    printf("Start scanning...");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
    printf(" completed!\n");

    // get the list of APs found in the last scan
    uint8_t ap_num;
    wifi_ap_record_t ap_records[20];
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

    // print the list
    printf("Found %d access points:\n", ap_num);

    printf("               SSID              | Channel | RSSI |   MAC \n\n");
    // printf("----------------------------------------------------------------\n");
    for (int i = 0; i < ap_num; i++)
        printf("%32s | %7d | %4d   %2x:%2x:%2x:%2x:%2x:%2x   \n", ap_records[i].ssid, ap_records[i].primary, ap_records[i].rssi, *ap_records[i].bssid, *ap_records[i].bssid + 1, *(ap_records[i].bssid) + 2, *(ap_records[i].bssid) + 3, *(ap_records[i].bssid) + 4, *(ap_records[i].bssid) + 5);
    //  printf("----------------------------------------------------------------\n");

} */
/******************* ������������ GPIO *********************/
// static void gpio_init_(void)
// {
//     /****** INPUT *******/
//     gpio_config_t gpio_conf_in;
//     gpio_conf_in.pin_bit_mask = ENC_SW_SEL;
//     gpio_conf_in.mode = GPIO_MODE_INPUT;
//     gpio_conf_in.pull_up_en = GPIO_PULLUP_DISABLE;
//     gpio_conf_in.pull_down_en = GPIO_PULLUP_DISABLE;
//     gpio_conf_in.intr_type = GPIO_INTR_ANYEDGE; // ����������
//     gpio_config(&gpio_conf_in);

//     /****** OUTPUT *******/
//     gpio_config_t gpio_conf_out;
//     gpio_conf_out.pin_bit_mask = (LED1_PIN_SEL | LED2_PIN_SEL);
//     gpio_conf_out.mode = GPIO_MODE_OUTPUT;
//     gpio_conf_out.pull_up_en = GPIO_PULLUP_DISABLE;
//     gpio_conf_out.pull_down_en = GPIO_PULLDOWN_DISABLE;
//     gpio_conf_out.intr_type = GPIO_INTR_DISABLE;
//     gpio_config(&gpio_conf_out);
//     //     gpio_set_level(LED1_PIN, 0);
//     //     gpio_set_level(LED2_PIN, 0);
// }
/******************* ������������ TIMER *********************/
static void timer_init_(void)
{
    timer_config_t config_tim;
    config_tim.divider = 80;                         // ������������ �������� 1 ���
    config_tim.counter_dir = TIMER_COUNT_UP;         // ������ ����
    config_tim.counter_en = TIMER_START;             // ������� ��������
    config_tim.alarm_en = TIMER_ALARM_DIS;           // ������� ���������
    config_tim.auto_reload = TIMER_AUTORELOAD_DIS;   // ���������� ������������ ���������
                                                     // config_tim.intr_type =  TIMER_INTR_LEVEL;
    timer_init(TIMER_GROUP_0, TIMER_0, &config_tim); // �������������
}
/******************* Config LEDC *********************/
/* static void ledc_init_(void)
{
    ledc_timer_config_t ledc_tim_config;                 // ������������ �������
    ledc_tim_config.duty_resolution = LEDC_TIMER_12_BIT; // �����������
    ledc_tim_config.freq_hz = 1000;                      // ������� ���
    ledc_tim_config.speed_mode = LEDC_HIGH_SPEED_MODE;   // ���������������� �����
    ledc_tim_config.timer_num = LEDC_TIMER_0;            // ����� �������
    ledc_timer_config(&ledc_tim_config);                 // ������������� �������

    ledc_channel_config_t ledc_config;
    ledc_config.channel = LEDC_CHANNEL_0;          // ����� ������ LEDC
    ledc_config.duty = 0;                          // ��������� ����������.0 �� 2^duty_resolution-1
    ledc_config.gpio_num = LED2_PIN;               // ����� ����
    ledc_config.intr_type = LEDC_INTR_DISABLE;     // ���������� ���������
    ledc_config.speed_mode = LEDC_HIGH_SPEED_MODE; //���������������� �����
    ledc_config.timer_sel = LEDC_TIMER_0;          // ����� �������
    ledc_config.hpoint = 0;
    ledc_channel_config(&ledc_config); // ������������� ������ �������
} */
/******************* Config I2C *********************/
static void i2c_init_(void)
{
    i2c_config_t config_i2c;
    config_i2c.mode = I2C_MODE_MASTER;
    config_i2c.sda_io_num = I2C_SDA;
    config_i2c.scl_io_num = I2C_SCL;
    config_i2c.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config_i2c.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config_i2c.master.clk_speed = 400 * 1000; // kHz
    config_i2c.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &config_i2c);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}
/******************* Config SDIO *********************/
// static void sdmmc_init(void)
// {
//     esp_err_t ret;
//     sdmmc_card_t *sd_card;
//     FILE *fp;
//     FILE *f_dump;

//     sdmmc_host_t host_config = SDMMC_HOST_DEFAULT();
//     sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
//     sdmmc_host_set_card_clk(SDMMC_HOST_SLOT_1, SDMMC_FREQ_HIGHSPEED);
//     ret = sdmmc_host_pullup_en(1, 4);
//     ESP_ERROR_CHECK(ret);
//     esp_vfs_fat_mount_config_t mount_config = {
//         .allocation_unit_size = 16 * 512, // Size sector, bytes (min 512)
//         .format_if_mount_failed = true,
//         .max_files = 5};

//     ret = esp_vfs_fat_sdmmc_mount(SDMMC_MOUNT, &host_config, &slot_config, &mount_config, &sd_card);
//     ESP_ERROR_CHECK(ret);

//     /* fp = fopen(SDMMC_MOUNT "/pizda.bin", "wb");
//     if (fp == NULL)
//     {
//         ESP_LOGE(TAG_SD, "File open error");
//         return;
//     }
//     fwrite(&kiska, sizeof(kiska), 1, fp);
//     fclose(fp); */

//     fp = fopen(SDMMC_MOUNT "/kiska.bin", "rb");
//     if (fp == NULL)
//     {
//         ESP_LOGE(TAG_SD, "File open error");
//         return;
//     }
//     fread(&sd_buff, 1, sizeof(sd_buff), fp);
//     fclose(fp);

//     /* for (uint32_t addr = 0x0; addr < 320000; addr += 0x10000)
//     {
//         esp_flash_read_encrypted(esp_flash_default_chip, addr, &dump, 0x10000);
//         f_dump = fopen(SDMMC_MOUNT "/core_dump.bin", "ab");
//         fwrite(&dump, sizeof(dump), 1, f_dump);
//     } */

//     /* for (uint32_t addr = 0x0; addr < 10; addr++)
//     {

//         fp = fopen(SDMMC_MOUNT "/Core_Dump.bin", "ab");
//     }
//     fclose(fp); */

//     /* for (uint8_t i = 0; i < 10; i++)
//     {
//         sd_buff[i] = i;
//     } */

//     /*  for (uint8_t i = 0; i < 10; i++)
//      {
//          printf("sd_buff[%u] = %c\n", i, sd_buff[i]);
//      }
//      for (uint8_t i = 0; i < 10; i++)
//      {
//          printf("sd_buff[%u] = 0x%02x\n", i, (uint8_t)sd_buff[i]);
//      } */
//     /*  fp = fopen(SDMMC_MOUNT "/write.txt", "w");
//     fwrite(&sd_buff, sizeof(sd_buff), 1, fp);
//     fclose(fp);
//     printf("Write w"); */

//     /* fp = fopen(SDMMC_MOUNT "/wrt.bin", "wb");
//     fwrite(&sd_buff, sizeof(sd_buff), 1, fp);
//     fclose(fp); */
//     /* fp = fopen(SDMMC_MOUNT "/kiska.txt", "rb");
//     fread(sd_buff, sizeof(uint16_t), 10, fp);
//     fclose(fp);
//     for (uint8_t i = 0; i < 10; i++)
//     {
//         printf("[%u] = %c\n", i, sd_buff[i]);
//     }

//     fp = fopen(SDMMC_MOUNT "/kiska.txt", "r");
//     fread(sd_buff, sizeof(uint16_t), 10, fp);
//     fclose(fp);
//     for (uint8_t i = 0; i < 10; i++)
//     {
//         printf("[%u] = %x\n", i, sd_buff[i]);
//     } */
// }
/******************* MAIN_TASK *****************************/
void main_task(void *pvParameters)
{

    /* queue_t data_send;
    uint32_t cnt_cpu_cycle = 0;
    uint32_t cnt_cpu_cycle_run = 0;
    volatile uint32_t *p_addr = 0x3ff44004;

    gpio_set_direction(16, GPIO_MODE_OUTPUT);
    gpio_set_level(16, 0); */

    // uint32_t read_buff[10] = {0};

    // esp_flash_init(esp_flash_default_chip);// ������ �� ���� � 0x001d0000

    /* esp_flash_read_id(esp_flash_default_chip, &chip);
    printf("Chip ID: %u\n", chip);

    esp_flash_get_size(esp_flash_default_chip, &chip);
    printf("Chip size: %uMB\n", chip / (1024 * 1024)); */
    // esp_partition_mmap()

    while (1)
    {
        encoder();
        /* uint8_t dat[2];
        lcd_send_cmd(0x55);
        dat[0] = 0x1d;
        dat[1] = 0x2d;
        lcd_send_data8n(dat, 2);

        lcd_send_cmd(0x66);
        lcd_send_data16(0x1A2B, 10);
        ESP_LOGI("LCD", "Data sended"); */

        /* ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, set_duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0); */

        // timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &tim_sec);
        // data_send.led_state = 0;
        // data_send.led_pwm = 0;
        // data_send.temp = temp;
        // data_send.press = press;
        // xQueueSend(queue_data_print, &data_send, 0x0);
        // xQueueSend(queue_rtc_print, &rtc, 0x0);
        // st7735_fill_screen(TFT_BLACK);
        // st7735_image(0, 0, 160, 128, sd_buff);

        /* ESP_LOGI("Read Temp", "Current temp = %.1f", temp);
        ESP_LOGW("Read Press", "Current press = %u", press);
        ESP_LOGE("Read Temp", "Current temp = %.1f", temp); */
        // ESP_LOGE(__FUNCTION__, "Low temp");
        // set_xy(0, 0, 10, 100);
        //  esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 59, 59, rgb);

        /**************** Send to I2S *******************/
        /* ESP_LOGE("I2S_FIFO_WR_REG", "[0x%08X]", READ_PERI_REG(DR_REG_I2S_BASE));
        ESP_LOGE("I2S_FIFO_RD_REG", "[0x%08X]", READ_PERI_REG(DR_REG_I2S_BASE + 0x004));
        ESP_LOGW("I2S_SAMPESP_LOGE("dt", "[0x%08X]", dt);LE_RATE_CONF_REG_0", "[0x%08X]\n", READ_PERI_REG(I2S_SAMPLE_RATE_CONF_REG(0))); */

        /* SET_PERI_REG_MASK(I2S_CONF1_REG(0), I2S_TX_STOP_EN); // Stop WS signal, if FIFO empty
        CLEAR_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_START);  // TX Stop (нужно)

        SET_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_RESET);   // TX Reset (нужно)
        CLEAR_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_RESET); // TX Reset (нужно)
 */
        //
        /***************************** Send 16 bit to I2S ******************************/
        /* uint32_t dat = 0;
        uint16_t dt = 0x0;

        dt = 0x1020;
        SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_TX_BITS_MOD, 16, I2S_TX_BITS_MOD_S); // For trans 16 bit
        for (uint8_t i = 0; i < 10; i++)
        {
            dat = (uint32_t)(dt << 8) | dt;
            while (!(READ_PERI_REG(I2S_STATE_REG(0)) & I2S_TX_IDLE))
            {
            }
            WRITE_PERI_REG(DR_REG_I2S_BASE, dat); // Write to FIFO_RD_REG
            dt++;
        }
        CLEAR_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_START); // TX Stop (нужно)
        SET_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_RESET);   // TX Reset (нужно)
        CLEAR_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_RESET); // TX Reset (нужно) */

        /***************************** Send 8 bit to I2S ******************************/
        /* uint8_t cmd = 0x60;
         dat = 0;
        SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_TX_BITS_MOD, 32, I2S_TX_BITS_MOD_S); // For trans 8 bit  [23:16]
        SET_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_START);                                       // Start
        for (uint8_t i = 0; i < 10; i++)
        {

            dat = (uint32_t)cmd << 16;
            while (!(READ_PERI_REG(I2S_STATE_REG(0)) & I2S_TX_IDLE))
            {
            }
            WRITE_PERI_REG(DR_REG_I2S_BASE, dat); // Write to FIFO_RD_REG
            cmd++;
        }
        CLEAR_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_START); // TX Stop (нужно)
        SET_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_RESET);   // TX Reset (нужно)
        CLEAR_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_RESET); // TX Reset (нужно)
 */
        // esp_lcd_panel_io_tx_color(io_handle, LCD_CMD_RAMWR, color, 1000);
        /* if (cnt == 1)
            set_xy(0, 0, 319, 479);

        esp_lcd_panel_io_tx_param(io_handle, 0, 0x07E0, 380 * 420 * 2);
        // red
        else if (cnt == 2)
            esp_lcd_panel_io_tx_param(io_handle, 0, 0x001F, 380 * 420 * 2);
        // green
        else if (cnt == 3)
            esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 200, 300, 0x001F);
        // blue
        cnt++;
        if (cnt == 4)
            cnt = 1; */

        // printf("Flash size: %u Mb\n", spi_flash_get_chip_size() / (1024 * 1024));

        /* cnt_cpu_cycle = esp_cpu_get_ccount();
       cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
       printf("/ = %u tick\n", cnt_cpu_cycle_run);  */

        /*  cnt_cpu_cycle = esp_cpu_get_ccount();
        gpio_set_level(GPIO_NUM_19, 0); // 2MHz
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("hal off = %u tick\n ", cnt_cpu_cycle_run); */

        /* cnt_cpu_cycle = esp_cpu_get_ccount();
        gpio_set_level(16, 1); // 55 tick
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("hal on = %u tick\n", cnt_cpu_cycle_run);

        cnt_cpu_cycle = esp_cpu_get_ccount();
        gpio_set_level(16, 0); // 57 tick
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("hal off = %u tick\n\n", cnt_cpu_cycle_run);

        cnt_cpu_cycle = esp_cpu_get_ccount();
        GPIO.out_w1ts = (1 << 16); //5 tick
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("ll on = %u tick\n", cnt_cpu_cycle_run);

        cnt_cpu_cycle = esp_cpu_get_ccount();
        GPIO.out_w1tc = (1 << 16); // 3 tick
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("ll off = %u tick\n\n", cnt_cpu_cycle_run);

        cnt_cpu_cycle = esp_cpu_get_ccount();
        SET_PERI_REG_MASK(GPIO_OUT_REG, BIT(16)); // 18 tick
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("out_reg on = %u tick\n", cnt_cpu_cycle_run);

        cnt_cpu_cycle = esp_cpu_get_ccount();
        CLEAR_PERI_REG_MASK(GPIO_OUT_REG, BIT(16)); // 19 tick
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("out_reg off = %u tick\n\n", cnt_cpu_cycle_run);

        cnt_cpu_cycle = esp_cpu_get_ccount();
        SET_PERI_REG_MASK(GPIO_OUT_W1TS_REG, BIT(16)); // 20 tick
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("out_w1ts_reg on = %u tick\n", cnt_cpu_cycle_run);

        cnt_cpu_cycle = esp_cpu_get_ccount();
        SET_PERI_REG_MASK(GPIO_OUT_W1TC_REG, BIT(16)); // 19 tick
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("out_w1tc_reg off = %u tick\n\n", cnt_cpu_cycle_run);

        cnt_cpu_cycle = esp_cpu_get_ccount();
        *p_addr = 0x3FF54004; // 4 tick
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("addr on = %u tick\n ", cnt_cpu_cycle_run);

        cnt_cpu_cycle = esp_cpu_get_ccount();
        *p_addr = 0x3FF44004; // 3 tick
        cnt_cpu_cycle_run = esp_cpu_get_ccount() - cnt_cpu_cycle;
        printf("addr off = %u tick\n\n ", cnt_cpu_cycle_run);  */
        /*   if (enc_l())
              cnt -= 0x10000;
          if (enc_r())
              cnt += 0x10000;
          spi_flash_read(0 + cnt, read_buff, sizeof(read_buff));
          printf("CNT addr: 0x%08X\n",cnt);
          for (uint8_t i = 0; i < 10; i++)
          {
              printf("Address [%u] = 0x%08x\n", i, read_buff[i]);
          }
          printf("\n\n"); */

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
/******************* PRINT_TASK *****************************/
void print_task(void *pvParameters)
{
    // queue_t data_get;
    // rtc_t rtc;
    // float temp = 0.0f;
    while (1)
    {

        // printf("Check trans = %u\n", ch);

        // xQueueReceive(queue_data_print, &data_get, portMAX_DELAY);

        /* eeprom_write_short(0, 1234);
        uint16_t ee_get = eeprom_read_short(0);
        printf("EEPROM = %u\t LED PWM = %u\t Temp = %.1f *C\t Press = %u mmHg\n", ee_get, data_get.led_pwm, data_get.temp, data_get.press);
        if (ds3231_get_time(&rtc) == ESP_OK)
        {
            printf("%02u-%02u-20%2u\t %02u:%02u:%02u\t %u\n", rtc.date, rtc.month, rtc.year, rtc.hour, rtc.min, rtc.sec, rtc.days_of_week);
        }
        float temp = ds3231_read_temp();
        printf("DS3231 Temp: %.1f\n\n", temp); */
        // st7735_fill_screen(TFT_BLUE);
        // st7735_fill_screen(ST7735_GREEN);

        /*printf("SPI_ADDR    [0x%08X]\n", READ_PERI_REG(SPI_ADDR_REG(2)));
       printf("SPI_CMD    [0x%08X]\n", READ_PERI_REG(SPI_CMD_REG(2)));
       printf("SPI_USER    [0x%08X]\n", READ_PERI_REG(SPI_USER_REG(2)));
       printf("SPI_USER1    [0x%08X]\n", READ_PERI_REG(SPI_USER1_REG(2)));
       printf("SPI_USER2   [0x%08X]\n", READ_PERI_REG(SPI_USER2_REG(2)));
       printf("SPI_MOSI_DLEN    [0x%08X]\n", READ_PERI_REG(SPI_MOSI_DLEN_REG(2)));
       printf("SPI_W0_REG    [0x%08X]\n", READ_PERI_REG(SPI_W0_REG(2)));
       printf("SPI_PIN    [0x%08X]\n", READ_PERI_REG(SPI_PIN_REG(2)));
       printf("SPI_EXT2    [0x%08X]\n", READ_PERI_REG(SPI_EXT2_REG(2)));
       printf("SPI_CTRL   [0x%08X]\n", READ_PERI_REG(SPI_CTRL_REG(2)));
       printf("SPI_CLOCK    [0x%08X]\n", READ_PERI_REG(SPI_CLOCK_REG(2)));  */

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
/******************* BTN_TASK *****************************/
/* void btn_task(void *pvParameters)
{
    uint8_t msec = 0;

    while (1)
    {
        msec++;
        if (msec >= 2)
        {
            gpio_intr_enable(ENC_SW); // ��������� ����������
            msec = 0;
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
} */
/**************************************************/
// void lcd_init_(void)
// {
//     ESP_LOGI("LCD", "Initialize Intel 8080 bus");
//     esp_lcd_i80_bus_config_t bus_config = {
//         .dc_gpio_num = DC,
//         .wr_gpio_num = WR,
//         .clk_src = LCD_CLK_SRC_PLL160M,
//         .data_gpio_nums = {
//             DB0,
//             DB1,
//             DB2,
//             DB3,
//             DB4,
//             DB5,
//             DB6,
//             DB7,
//         },
//         .bus_width = 8,
//         .max_transfer_bytes = 0};
//     ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

//     esp_lcd_panel_io_i80_config_t io_config = {
//         .cs_gpio_num = -1,
//         .pclk_hz = 1 * 1000 * 1000,
//         .trans_queue_depth = 1,
//         .dc_levels = {
//             .dc_idle_level = 0,
//             .dc_cmd_level = 0,
//             .dc_dummy_level = 0,
//             .dc_data_level = 1,
//         },
//         //.on_color_trans_done = tx_done_cb,
//         //.user_ctx = &disp_drv,
//         .lcd_cmd_bits = 8,
//         .lcd_param_bits = 8,
//     };
//     ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

//     ESP_LOGI("LCD", "Install LCD driver");
//     esp_lcd_panel_dev_config_t panel_config = {
//         .reset_gpio_num = -1,
//         .color_space = ESP_LCD_COLOR_SPACE_RGB,
//         .bits_per_pixel = 16,
//     };
//     //     ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
//     //     esp_lcd_panel_init(panel_handle);

//     //     ESP_LOGI("LCD", "Initialize...");

//     //     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_SLPOUT, NULL, 0);
//     //     ets_delay_us(100 * 1000);
//     uint8_t data[1];
//     data[0] = 0x48;
//     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_MADCTL, data, 1);

//     //     data[0] = 0x05;
//     //     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_COLMOD, data, 1);

//     //     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_RAMWR, NULL, 0);
//     //     ets_delay_us(150 * 1000);

//     //     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_DISPON, NULL, 0);

//     //     ESP_LOGI("LCD", "Initialize done");

//     //     /************************ Read Periph Reg ***************************/
//     CLEAR_PERI_REG_MASK(I2S_FIFO_CONF_REG(0), I2S_DSCR_EN); // Disable DMA
//     //     SET_PERI_REG_MASK(I2S_CONF2_REG(0), I2S_LCD_EN);        // Enable LCD Mode
//     //                                                             // SET_PERI_REG_MASK(I2S_CONF2_REG(0), I2S_LCD_TX_WRX2_EN); // Enable dual WR
//     //     // SET_PERI_REG_MASK(I2S_CONF_REG(0), I2S_TX_MSB_RIGHT);
//     //     // SET_PERI_REG_MASK(I2S_FIFO_CONF_REG(0), I2S_TX_FIFO_MOD_FORCE_EN);
//     //     // SET_PERI_REG_MASK(I2S_FIFO_CONF_REG(0), I2S_RX_FIFO_MOD_FORCE_EN);
//     //     // SET_PERI_REG_BITS(I2S_CONF_CHAN_REG(0), I2S_TX_CHAN_MOD, 2, I2S_TX_CHAN_MOD_S);  // Mono
//     //     // SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), I2S_TX_FIFO_MOD, 1, I2S_TX_FIFO_MOD_S);  // 16bit single
//     //     // SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), I2S_TX_DATA_NUM, 32, I2S_TX_DATA_NUM_S); // 32bit data lenght
// }
/************************************************/
// void set_xy(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
// {
//     uint8_t data[4];

//     data[0] = x0 >> 8;
//     data[1] = x0;
//     data[2] = x1 >> 8;
//     data[3] = x1;
//     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_CASET, data, 4);

//     data[0] = y0 >> 8;
//     data[1] = y0;
//     data[2] = y1 >> 8;
//     data[3] = y1;
//     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_RASET, data, 4);

//     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_RAMWR, NULL, 0);
// }
/****************************************************/
void app_main(void)
{ /****** INPUT *******/
    gpio_config_t gpio_conf_in;
    gpio_conf_in.pin_bit_mask = ENC_SW_SEL;
    gpio_conf_in.mode = GPIO_MODE_INPUT;
    gpio_conf_in.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf_in.pull_down_en = GPIO_PULLUP_DISABLE;
    gpio_conf_in.intr_type = GPIO_INTR_ANYEDGE; // ��� ����������
    gpio_config(&gpio_conf_in);

    /****** OUTPUT *******/
    gpio_config_t gpio_conf_out;
    gpio_conf_out.pin_bit_mask = GPIO_SEL_19;
    gpio_conf_out.mode = GPIO_MODE_OUTPUT;
    gpio_conf_out.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf_out.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf_out.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio_conf_out);

    timer_init_();
    i2c_init_();
    encoder_init(34, 39, 36);
    eeprom_init(I2C_NUM_0);
    lcd_i80_bus_init(1);
    ili9488_i80_init(320,480);
    // lcd_init_();
    /*  lcd_send_cmd(0x23);
     lcd_send_cmd(0x24);
     uint8_t dtn[] = {0x5D, 0x6D, 0x7D, 0x8D};
     lcd_send_data8(&dtn, 4); */
    /* WRITE_PERI_REG(I2S_CLKM_CONF_REG(0), 0x00004002);
    WRITE_PERI_REG(I2S_SAMPLE_RATE_CONF_REG(0), 0x004201A8); */

    // print_reg();

    /*********** Config I2S **************/

    /****** PIN LCD *******/
    /* gpio_config_t gpio_lcd;
    gpio_lcd.pin_bit_mask = GPIO_SEL_13 | GPIO_SEL_12 | GPIO_SEL_14 | GPIO_SEL_27 | GPIO_SEL_26 | GPIO_SEL_25 | GPIO_SEL_0 | GPIO_SEL_4 | GPIO_SEL_15 | GPIO_SEL_2;
    gpio_lcd.mode = GPIO_MODE_OUTPUT;
    gpio_lcd.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_lcd.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_lcd.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio_lcd); */
    // ili9488_init();
    /*  gpio_set_level(WR, 1);
     gpio_set_level(DC, 0); */

    /* ******0x11 */
    /*  gpio_set_level(WR, 0);

     gpio_set_level(DB0, 1);
     gpio_set_level(DB1, 0);
     gpio_set_level(DB2, 0);
     gpio_set_level(DB3, 0);

     gpio_set_level(DB4, 1);
     gpio_set_level(DB5, 0);
     gpio_set_level(DB6, 0);
     gpio_set_level(DB7, 0);

     gpio_set_level(WR, 1); */

    /***0x29 */
    /* gpio_set_level(WR, 0);

    gpio_set_level(DB0, 1);
    gpio_set_level(DB1, 0);
    gpio_set_level(DB2, 0);
    gpio_set_level(DB3, 1);

    gpio_set_level(DB4, 0);
    gpio_set_level(DB5, 1);
    gpio_set_level(DB6, 0);
    gpio_set_level(DB7, 0);

    gpio_set_level(WR, 1); */

    /*****0x2c*/
    /* gpio_set_level(WR, 0);

     gpio_set_level(DB0, 0);
     gpio_set_level(DB1, 0);
     gpio_set_level(DB2, 1);
     gpio_set_level(DB3, 1);

     gpio_set_level(DB4, 0);
     gpio_set_level(DB5, 1);
     gpio_set_level(DB6, 0);
     gpio_set_level(DB7, 0);

     gpio_set_level(WR, 1);
  */

    /* // initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t sta_config = {
        .sta = {
            .ssid = STA_SSID,
            .password = STA_PASSWORD},
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // starts wifi usage
    ESP_ERROR_CHECK(esp_wifi_connect());

    esp_http_client_config_t client_config = {
        .url = "http://yandex.ru",
        .event_handler = client_event_handler};
    esp_http_client_handle_t client = esp_http_client_init(&client_config);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client); */
    /******************* �������� ����� *****************************/
    xTaskCreate(main_task, "main_task", 3000, NULL, 1, NULL);
    xTaskCreate(print_task, "print_task", 2000, NULL, 1, NULL);
    // xTaskCreate(btn_task, "btn_task", 2000, NULL, 1, NULL);

    /******************* �������� �������� *****************************/
    queue_data_print = xQueueCreate(1, sizeof(queue_t));
    // queue_rtc_print = xQueueCreate(1, sizeof(rtc_t));
}
