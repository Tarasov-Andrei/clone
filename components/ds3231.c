#include "ds3231.h"

#define DS3231_ADDRESS 0x68

#define DS3231_REG_TIME 0x00
#define DS3231_REG_TEMP 0x11

static i2c_port_t DS3231_PORT = 0;

/********************************************************************/
static uint8_t bcd_to_dec(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}
/********************************************************************/
static uint8_t dec_to_bcd(uint8_t decimal)
{
    return (((decimal / 10) << 4) | (decimal % 10));
}
/********************************************************************/
esp_err_t ds3231_get_time(rtc_t *rtc)
{
    esp_err_t ds3231_err = ESP_OK;
    uint8_t write_data = DS3231_REG_TIME;
    uint8_t read_buffer[7] = {0};

    ds3231_err = i2c_master_write_to_device(DS3231_PORT, DS3231_ADDRESS, &write_data, 1, 1000 / portTICK_PERIOD_MS);
    if (ds3231_err != ESP_OK)
    {
        return ds3231_err;
    }

    ds3231_err = i2c_master_read_from_device(DS3231_PORT, DS3231_ADDRESS, read_buffer, sizeof(read_buffer), 1000 / portTICK_PERIOD_MS);
    if (ds3231_err != ESP_OK)
    {
        return ds3231_err;
    }

    rtc->sec = bcd_to_dec(read_buffer[0] & 0x7F);
    rtc->min = bcd_to_dec(read_buffer[1] & 0x7F);
    rtc->hour = bcd_to_dec(read_buffer[2] & 0x3F);
    rtc->days_of_week = read_buffer[3] & 0x07;
    rtc->date = bcd_to_dec(read_buffer[4] & 0x3F);
    rtc->month = bcd_to_dec(read_buffer[5] & 0x1F);
    rtc->year = bcd_to_dec(read_buffer[6]);

    return ESP_OK;
}
/********************************************************************/
esp_err_t ds3231_set_time(rtc_t *rtc)
{
    esp_err_t ds3231_err = ESP_OK;
    uint8_t write_data = DS3231_REG_TIME;
    uint8_t write_buffer[8] = {0};

    write_buffer[0] = write_data;
    write_buffer[1] = dec_to_bcd(rtc->sec);
    write_buffer[2] = dec_to_bcd(rtc->min);
    write_buffer[3] = dec_to_bcd(rtc->hour);
    write_buffer[4] = rtc->days_of_week;
    write_buffer[5] = dec_to_bcd(rtc->date);
    write_buffer[6] = dec_to_bcd(rtc->month);
    write_buffer[7] = dec_to_bcd(rtc->year);

    ds3231_err = i2c_master_write_to_device(DS3231_PORT, DS3231_ADDRESS, write_buffer, sizeof(write_buffer), 1000 / portTICK_PERIOD_MS);
    if (ds3231_err != ESP_OK)
    {
        return ds3231_err;
    }

    return ESP_OK;
}
/********************************************************************/
float ds3231_read_temp(void)
{
    esp_err_t ds3231_err = ESP_OK;
    uint8_t write_data = DS3231_REG_TEMP;
    uint8_t read_buffer[2] = {0};

    ds3231_err = i2c_master_write_to_device(DS3231_PORT, DS3231_ADDRESS, &write_data, sizeof(write_data), 1000 / portTICK_PERIOD_MS);
    if (ds3231_err != ESP_OK)
    {
        return 0;
    }

    ds3231_err = i2c_master_read_from_device(DS3231_PORT, DS3231_ADDRESS, read_buffer, sizeof(read_buffer), 1000 / portTICK_PERIOD_MS);
    if (ds3231_err != ESP_OK)
    {
        return 0;
    }

    int16_t value = (read_buffer[0] << 8) | (read_buffer[1]);
    value = (value >> 6);

    float temp = value / 4.0f;
    return temp;
}
/********************************************************************/
void ds3231_init(i2c_port_t i2c_port)
{
    DS3231_PORT = i2c_port;
}
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
