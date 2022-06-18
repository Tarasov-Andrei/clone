#include "lcd_i80_bus.h"

#define PLL_D2_CLK (160 * 1000 * 1000) // MHz

void print_reg(void)
{
    ESP_LOGW("IO_MUX_GPIO15_REG", "[0x%08X]", READ_PERI_REG(IO_MUX_GPIO15_REG));
    ESP_LOGW("I2S_CONF_REG", "[0x%08X]", READ_PERI_REG(I2S_CONF_REG(0)));
    ESP_LOGW("I2S_CONF1_REG", "[0x%08X]", READ_PERI_REG(I2S_CONF1_REG(0)));
    ESP_LOGW("I2S_CONF2_REG", "[0x%08X]", READ_PERI_REG(I2S_CONF2_REG(0)));
    ESP_LOGW("I2S_TIMING_REG", "[0x%08X]", READ_PERI_REG(I2S_TIMING_REG(0)));
    ESP_LOGW("I2S_FIFO_CONF_REG", "[0x%08X]", READ_PERI_REG(I2S_FIFO_CONF_REG(0)));
    ESP_LOGW("I2S_CONF_CHAN_REG", "[0x%08X]", READ_PERI_REG(I2S_CONF_CHAN_REG(0)));
    ESP_LOGW("I2S_LC_HUNG_CONF_REG", "[0x%08X]", READ_PERI_REG(I2S_LC_HUNG_CONF_REG(0)));
    ESP_LOGW("I2S_CLKM_CONF_REG", "[0x%08X]", READ_PERI_REG(I2S_CLKM_CONF_REG(0)));
    ESP_LOGW("I2S_SAMPLE_RATE_CONF_REG", "[0x%08X]\n", READ_PERI_REG(I2S_SAMPLE_RATE_CONF_REG(0)));
    ESP_LOGW("I2S_PD_CONF_REG", "[0x%08X]", READ_PERI_REG(I2S_PD_CONF_REG(0)));
    ESP_LOGW("I2S_PDM_CONF_REG", "[0x%08X]", READ_PERI_REG(I2S_PDM_CONF_REG(0)));
    ESP_LOGW("I2S_PDM_FREQ_CONF_REG", "[0x%08X]", READ_PERI_REG(I2S_PDM_FREQ_CONF_REG(0)));

    ESP_LOGW("DR_REG_I2S_BASE_0", "[0x%08X]\n", READ_PERI_REG(DR_REG_I2S_BASE));
}

void lcd_i80_bus_init(uint8_t freq_pclk_Mhz)
{
    ESP_LOGI("LCD", "Inizialize...");
// Enable I2S
#if I2S_NUM
    periph_module_enable(PERIPH_I2S1_MODULE);
#else
    periph_module_enable(PERIPH_I2S0_MODULE);
#endif
    i2s.conf.tx_right_first = 1;
    i2s.conf.tx_slave_mod = 0;

    i2s.conf1.tx_pcm_conf = 0; // Decompress TX data
    i2s.conf1.tx_pcm_bypass = 1;
    i2s.conf1.tx_stop_en = 1; // Stop WS signal, if FIFO empty

    i2s.conf2.lcd_en = 1; // Set I2S in LCD mode
    i2s.conf2.lcd_tx_wrx2_en = 0;
    i2s.conf2.lcd_tx_sdx2_en = 0;

    i2s.fifo_conf.rx_fifo_mod_force_en = 1;
    i2s.fifo_conf.tx_fifo_mod_force_en = 1;
    i2s.fifo_conf.tx_fifo_mod = 1; // [1]-16 bit single, [3]-32 bit single
    i2s.fifo_conf.dscr_en = 0;     // Disable DMA
    i2s.fifo_conf.tx_data_num = 8; // TX Threshold data lenght

    i2s.conf_chan.tx_chan_mod = 1; // Mono

    /* Set I2S clock
     * because we set the I2S's left channel data same to right channel, so f_pclk = f_i2s/pclk_div/2
     */
    if (freq_pclk_Mhz == 0 || freq_pclk_Mhz > 20)
    {
        ESP_LOGE(__FUNCTION__, "Frequency invalid: (%u)", __LINE__);
        return;
    }
    uint32_t bck_div = PLL_D2_CLK / 4 / (freq_pclk_Mhz * 1000 * 1000);
    i2s.clkm_conf.val = 0;
    i2s.clkm_conf.clkm_div_b = 0;
    i2s.clkm_conf.clkm_div_a = 1;
    i2s.clkm_conf.clkm_div_num = 2; // N >= 2

    i2s.sample_rate_conf.tx_bck_div_num = bck_div; // M >= 2
    i2s.sample_rate_conf.tx_bits_mod = 8;

    // Clear tx start
    i2s.conf.tx_start = 0;

    /* Reset TX */
    i2s.conf.tx_reset = 1;
    i2s.conf.tx_reset = 0;

    /* Reset TX FIFO */
    i2s.conf.tx_fifo_reset = 1;
    i2s.conf.tx_fifo_reset = 0;

    /* Connect peripheral signals via GPIO matrix */
    uint8_t data_gpio[] =
        {
            LCD_DB0,
            LCD_DB1,
            LCD_DB2,
            LCD_DB3,
            LCD_DB4,
            LCD_DB5,
            LCD_DB6,
            LCD_DB7,
        };
#if SOC_I2S_TRANS_SIZE_ALIGN_WORD
#if I2S_NUM
    uint32_t signal_idx = I2S1O_DATA_OUT8_IDX;
#else
    uint32_t signal_idx = I2S0O_DATA_OUT8_IDX;
#endif // I2S_NUM

#else
#if I2S_NUM
    uint32_t signal_idx = I2S1O_DATA_OUT0_IDX;
#else
    uint32_t signal_idx = I2S0O_DATA_OUT0_IDX;
#endif // I2S_NUM

#endif // SOC_I2S_TRANS_SIZE_ALIGN_WORD
    for (uint8_t i = 0; i < 8; i++)
    {
        gpio_set_direction(data_gpio[i], GPIO_MODE_OUTPUT);
        gpio_matrix_out(data_gpio[i], signal_idx + i, 0, 0);
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[data_gpio[i]], PIN_FUNC_GPIO);
    }
    /* Config WS */
    gpio_set_direction(LCD_WR, GPIO_MODE_OUTPUT);
#if I2S_NUM
    gpio_matrix_out(LCD_WR, I2S1O_WS_OUT_IDX, 1, 0); // WS needs inverted
#else
    gpio_matrix_out(LCD_WR, I2S0O_WS_OUT_IDX, 1, 0);
#endif
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[LCD_WR], PIN_FUNC_GPIO);

    // Config DC
    gpio_set_direction(LCD_DC, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_DC, 1);
    ESP_LOGI("LCD", "Inizialize success");
}
/*********************************************/

// /************************************/
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
//         .max_transfer_bytes = 240 * 10 * sizeof(uint16_t)};
//     ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

//     esp_lcd_panel_io_i80_config_t io_config = {
//         .cs_gpio_num = -1,
//         .pclk_hz = 1 * 1000 * 1000,
//         .trans_queue_depth = 10,
//         .dc_levels = {
//             .dc_idle_level = 0,
//             .dc_cmd_level = 0,
//             .dc_dummy_level = 0,
//             .dc_data_level = 1,
//         },
//         //.on_color_trans_done = example_notify_lvgl_flush_ready,
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
//     // ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

//     uint8_t data[1];
//     ESP_LOGI("LCD", "Initialize...");

//     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_SLPOUT, NULL, 0);
//     ets_delay_us(100 * 1000);

//     data[0] = 0x48;
//     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_MADCTL, data, 1);

//     data[0] = 0x05;
//     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_COLMOD, data, 1);

//     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_RAMWR, NULL, 0);
//     ets_delay_us(150 * 1000);

//     esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_DISPON, NULL, 0);

//     ESP_LOGI("LCD", "Initialize done");
// }
// /************************************************/
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
// /************************************************/
// void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
// {
//     // set_xy(x, y, x + w - 1, y + h - 1);
//     uint8_t data[] = {color >> 8, color};

//     /* esp_lcd_panel_io_tx_color(io_handle, )

//     for (y = h; y > 0; y--)
//     {
//         for (x = w; x > 0; x--)
//         {
//             lcd_write_data(data, 2);
//         }
//     } */
// }