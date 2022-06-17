/* Windows 1251 */

#include "st7735.h"
#include "sdkconfig.h"
#include "stdio.h"
#include "string.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SPI_NUM 3
#define SPI_FREQ 30 // ������� SPI, MHz
#define DMA_EN 0   // 1 - ������������ DMA

#if DMA_EN
#define SPI_DMA_CH SPI_DMA_CH1
#endif

#define SPI_CS GPIO_NUM_5
#define SPI_DC GPIO_NUM_26
#define SPI_RST GPIO_NUM_27
#define SPI_MOSI GPIO_NUM_23
#define SPI_SCLK GPIO_NUM_18

#define ST7735_MADCTL_MY 0x80
#define ST7735_MADCTL_MX 0x40
#define ST7735_MADCTL_MV 0x20
#define ST7735_MADCTL_ML 0x10
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH 0x04

#define ST7735_IS_160X128 1
#define ST7735_WIDTH 160
#define ST7735_HEIGHT 128
#define ST7735_XSTART 0
#define ST7735_YSTART 0
#define ST7735_ROTATION (ST7735_MADCTL_MY | ST7735_MADCTL_MV)

#define ST7735_NOP 0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID 0x04
#define ST7735_RDDST 0x09

#define ST7735_SLPIN 0x10
#define ST7735_SLPOUT 0x11
#define ST7735_PTLON 0x12
#define ST7735_NORON 0x13

#define ST7735_INVOFF 0x20
#define ST7735_INVON 0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON 0x29
#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_RAMRD 0x2E

#define ST7735_PTLAR 0x30
#define ST7735_COLMOD 0x3A
#define ST7735_MADCTL 0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5

#define ST7735_RDID1 0xDA
#define ST7735_RDID2 0xDB
#define ST7735_RDID3 0xDC
#define ST7735_RDID4 0xDD

#define ST7735_PWCTR6 0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

#define u16_to_32(d) ((uint32_t)(d << 24) | (d << 8) | (d >> 8))
#define u16_convert(u) ((uint16_t)(u << 8) | (u >> 8))

#define CHECK_BUSY (READ_PERI_REG(SPI_CMD_REG(SPI_NUM)) & SPI_USR)

static spi_device_handle_t spi_device;

/********************************************************************/
static void st7735_send_cmd(uint8_t cmd)
{
#if DMA_EN
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length = 8;
    trans.tx_buffer = cmd;
    trans.user = (void *)0;
    spi_device_polling_transmit(spi_device, &trans);
#else
    while (CHECK_BUSY)
        ;
    // gpio_set_level(SPI_DC, 0);
    CLEAR_PERI_REG_MASK(GPIO_OUT_REG, BIT(SPI_DC));

    // ��������� ����� ������ ��������, � ����� -1
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);

    // ������ � ����� ��������
    WRITE_PERI_REG(SPI_W0_REG(SPI_NUM), cmd);

    // ������ ��������
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
#endif
}
/********************************************************************/
static void st7735_send_data(uint8_t *data, uint8_t len)
{
#if DMA_EN
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length = len * 8;
    trans.tx_buffer = data;
    trans.user = (void *)1;
    spi_device_polling_transmit(spi_device, &trans);
#else
    while (CHECK_BUSY)
        ;
    // gpio_set_level(SPI_DC, 1);
    SET_PERI_REG_MASK(GPIO_OUT_REG, BIT(SPI_DC));

    // ��������� ����� ������ ��������, � ����� -1
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
    while (len > 0)
    {
        // ������ � ����� ��������
        WRITE_PERI_REG(SPI_W0_REG(SPI_NUM), *data++);

        // ������ ��������
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        len--;
        while (CHECK_BUSY)
            ;
    }
#endif
}
/********************************************************************/
static void st7735_send_data16n(uint16_t data, uint32_t size_bits)
{
    uint32_t data_32 = u16_to_32(data);
    uint8_t cnt_send = 0;
    while (CHECK_BUSY)
        ;
    // gpio_set_level(SPI_DC, 1);
    SET_PERI_REG_MASK(GPIO_OUT_REG, BIT(SPI_DC));
send_bits:
    if (size_bits <= 512)
    {
        cnt_send = (uint8_t)(size_bits >> 5); // ������� ���-�� �������� �� 32��� (size_bits/32)

        // ��������� ����� ������ ��������, � ����� -1
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, size_bits - 1, SPI_USR_MOSI_DBITLEN_S);

        // ������ � ����� �������� + ��������� ������ ��������
        for (uint8_t idx = 0; idx < cnt_send; idx++)
        {
            // ������ � ����� �������� + ��������� ������ ��������
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (idx << 2)), data_32);
        }
        // ������ ��������
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
    }
    else
    {
        //  ��������� ����� ������ ��������, � ����� -1
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);

        while (size_bits > 0)
        {
            for (uint8_t idx = 0; idx < 16; idx++)
            {
                // ������ � ����� �������� + ��������� ������ ��������
                WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (idx << 2)), data_32);
            }
            // ������ ��������
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
            while (CHECK_BUSY)
                ;
            size_bits -= 512;
            goto send_bits;
        }
    }
}
/********************************************************************/
static void st7735_send_data16(uint16_t data)
{
    data = u16_convert(data);
    while (CHECK_BUSY)
        ;
    // gpio_set_level(SPI_DC, 1);
    SET_PERI_REG_MASK(GPIO_OUT_REG, BIT(SPI_DC));

    // ��������� ����� ������ ��������, � ����� -1
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 15, SPI_USR_MOSI_DBITLEN_S);

    // ������ � ����� ��������
    WRITE_PERI_REG(SPI_W0_REG(SPI_NUM), data);

    // ������ ��������
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
}
/********************************************************************/
static void st7735_set_address_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    st7735_send_cmd(ST7735_CASET);
    uint8_t data[] = {0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART};
    st7735_send_data(data, 4);

    st7735_send_cmd(ST7735_RASET);
    data[1] = y0 + ST7735_YSTART;
    data[3] = y1 + ST7735_YSTART;
    st7735_send_data(data, 4);

    st7735_send_cmd(ST7735_RAMWR);
}
/********************************************************************/
void st7735_h_line(uint8_t x0, uint8_t y0, uint8_t x1, uint16_t color)
{
    st7735_fill_rect(x0, y0, x1, y0, color);
}
/********************************************************************/
void st7735_v_line(uint8_t x0, uint8_t y0, uint8_t y1, uint16_t color)
{
    st7735_fill_rect(x0, y0, x0, y1, color);
}
/********************************************************************/
void st7735_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color)
{
    st7735_h_line(x0, y0, x1, color);
    st7735_h_line(x0, y1, x1, color);
    st7735_v_line(x0, y0 + 1, y1 - 1, color);
    st7735_v_line(x1, y0 + 1, y1 - 1, color);
}
/********************************************************************/
void st7735_fill_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color)
{
    st7735_set_address_window(x0, y0, x1, y1);
    st7735_send_data16n(color, ((x0 + x1 + 1) * (y0 + y1 + 1)) << 4); // w*h*16, ������ � �����
}
/********************************************************************/
void st7735_image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint32_t *data)
{
    uint32_t size_bits = w * h * 16; // ������ � �����
    st7735_set_address_window(x, y, x + w - 1, y + h - 1);

    // uint32_t data_32 = 0;
    uint8_t cnt_send = 0;
    while (CHECK_BUSY)
        ;
    // gpio_set_level(SPI_DC, 1);
    SET_PERI_REG_MASK(GPIO_OUT_REG, BIT(SPI_DC));
send_bits:
    if (size_bits <= 512)
    {
        cnt_send = (uint8_t)(size_bits >> 5); // ������� ���-�� �������� �� 32��� (size_bits/32)

        // ��������� ����� ������ ��������, � ����� -1
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, size_bits - 1, SPI_USR_MOSI_DBITLEN_S);

        // ������ � ����� �������� + ��������� ������ ��������
        for (uint8_t idx = 0; idx < cnt_send; idx++)
        {
            // ������ � ����� �������� + ��������� ������ ��������
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (idx << 2)), *data++);
        }
        // ������ ��������
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
    }
    else
    {
        //  ��������� ����� ������ ��������, � ����� -1
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);

        while (size_bits > 0)
        {
            for (uint8_t idx = 0; idx < 16; idx++)
            {
                // ������ � ����� �������� + ��������� ������ ��������
                WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (idx << 2)), *data++);
            }
            // ������ ��������
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
            while (CHECK_BUSY)
                ;
            size_bits -= 512;
            goto send_bits;
        }
    }
}
/********************************************************************/
void st7735_char(uint8_t x, uint8_t y, unsigned char c, uint16_t color, uint16_t bg, font_t fonts, uint8_t size)
{
    uint16_t he = 0; // ������
    uint16_t wi = 0; // ������
    uint16_t b = 0;

    if (size == 1)
    {
        st7735_set_address_window(x, y, x + fonts.width - 1, y + fonts.height - 1);

        for (he = 0; he < fonts.height; he++)
        {
            b = fonts.data[(c - 32) * fonts.height + he]; //������ ������ ������� 1px
            for (wi = 0; wi < fonts.width; wi++)
            {
                if ((b << wi) & 0x8000)
                {
                    st7735_send_data16(color);
                }
                else
                {
                    st7735_send_data16(bg);
                }
            }
        }
    }
    else // Size > 1
    {
        for (he = 0; he < fonts.height; he++)
        {
            if (he == fonts.height)
                b = 0x0000;
            else
                b = fonts.data[(c - 32) * fonts.height + he]; //������ ������ ������� 1px
            for (wi = 0; wi < fonts.width; wi++)
            {
                if (b & 0x8000)
                {
                    st7735_fill_rect(x + (wi * size), y + (he * size), size, size, color);
                }
                else
                {
                    st7735_fill_rect(x + (wi * size), y + (he * size), size, size, bg);
                }
                b <<= 1;
            }
        }
    }
}
/********************************************************************/
void st7735_char_rus(uint8_t x, uint8_t y, unsigned char c, uint16_t color, uint16_t bg, font_t fonts, uint8_t size)
{
    uint16_t he = 0; // ������
    uint16_t wi = 0; // ������
    uint16_t b = 0;

    if (size == 1)
    {
        st7735_set_address_window(x, y, x + fonts.width - 1, y + fonts.height - 1);

        for (he = 0; he < fonts.height; he++)
        {
            b = fonts.data[(c - 192) * fonts.height + he]; //������ ������ ������� 1px
            for (wi = 0; wi < fonts.width; wi++)
            {
                if ((b << wi) & 0x8000)
                {
                    st7735_send_data16(color);
                }
                else
                {
                    st7735_send_data16(bg);
                }
            }
        }
    }
    else // Size > 1
    {
        for (he = 0; he < fonts.height; he++)
        {
            if (he == fonts.height)
                b = 0x0000;
            else
                b = fonts.data[(c - 192) * fonts.height + he]; //������ ������ ������� 1px
            for (wi = 0; wi < fonts.width; wi++)
            {
                if (b & 0x8000)
                {
                    st7735_fill_rect(x + (wi * size), y + (he * size), size, size, color);
                }
                else
                {
                    st7735_fill_rect(x + (wi * size), y + (he * size), size, size, bg);
                }
                b <<= 1;
            }
        }
    }
}
/********************************************************************/
void st7735_print(uint8_t x, uint8_t y, const char *str, uint16_t color, uint16_t bg, font_t font, uint8_t size)
{
    while (*str)
    {
        st7735_char(x, y, *str, color, bg, font, size);
        x = x + (font.width * size);
        str++;
    }
}
/********************************************************************/
void st7735_print_rus(uint8_t x, uint8_t y, const char *str, uint16_t color, uint16_t bg, font_t font, uint8_t size)
{
    while (*str)
    {
        st7735_char_rus(x, y, *str, color, bg, font, size);
        x = x + (font.width * size);
        str++;
    }
}
/********************************************************************/
void st7735_fill_screen(uint16_t color)
{
    st7735_fill_rect(0, 0, ST7735_WIDTH - 1, ST7735_HEIGHT - 1, color);
    /* for (uint16_t i = 0; i < ST7735_WIDTH * ST7735_HEIGHT * 2; i = i + 2)
   {
       tft_buff[i] = color >> 8;
       tft_buff[i + 1] = color & 0xFF;
   }
   st7735_set_address_window(0, 0, ST7735_WIDTH - 1, ST7735_HEIGHT - 1);
   st7735_send_data(tft_buff, ST7735_WIDTH * ST7735_HEIGHT * 2); */
}
/********************************************************************/
static void st7735_send_cmd_list(void)
{
    uint8_t data[15] = {0};
    st7735_send_cmd(ST7735_SLPOUT);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    st7735_send_cmd(ST7735_FRMCTR1);
    data[0] = 0x0;
    data[1] = 0x06;
    data[2] = 0x03;
    st7735_send_data(data, 3);

    st7735_send_cmd(ST7735_FRMCTR2);
    data[0] = 0x01;
    data[1] = 0x2C;
    data[2] = 0x2D;
    st7735_send_data(data, 3);

    st7735_send_cmd(ST7735_FRMCTR3);
    data[0] = 0x01;
    data[1] = 0x2C;
    data[2] = 0x2D;
    data[3] = 0x01;
    data[4] = 0x2C;
    data[5] = 0x2D;
    st7735_send_data(data, 6);

    st7735_send_cmd(ST7735_INVCTR);
    data[0] = 0x07;
    st7735_send_data(data, 1);

    st7735_send_cmd(ST7735_PWCTR1);
    data[0] = 0xA2;
    data[1] = 0x02;
    data[2] = 0x84;
    st7735_send_data(data, 3);

    st7735_send_cmd(ST7735_PWCTR2);
    data[0] = 0xC5;
    st7735_send_data(data, 1);

    st7735_send_cmd(ST7735_PWCTR3);
    data[0] = 0x0A;
    data[1] = 0x0;
    st7735_send_data(data, 2);

    st7735_send_cmd(ST7735_PWCTR4);
    data[0] = 0x8A;
    data[1] = 0x2A;
    st7735_send_data(data, 2);

    st7735_send_cmd(ST7735_PWCTR5);
    data[0] = 0x8A;
    data[1] = 0xEE;
    st7735_send_data(data, 2);

    st7735_send_cmd(ST7735_VMCTR1);
    data[0] = 0x0E;
    st7735_send_data(data, 1);

    st7735_send_cmd(ST7735_INVOFF);

    st7735_send_cmd(ST7735_MADCTL);
    data[0] = ST7735_ROTATION;
    st7735_send_data(data, 1);

    st7735_send_cmd(ST7735_COLMOD);
    data[0] = 0x05;
    st7735_send_data(data, 1);

    st7735_send_cmd(ST7735_GMCTRP1);
    data[0] = 0x02;
    data[1] = 0x1C;
    data[2] = 0x07;
    data[3] = 0x12;
    data[4] = 0x37;
    data[5] = 0x32;
    data[6] = 0x29;
    data[7] = 0x2D;
    data[8] = 0x29;
    data[9] = 0x25;
    data[10] = 0x2B;
    data[11] = 0x39;
    data[12] = 0x0;
    data[13] = 0x01;
    data[14] = 0x03;
    data[15] = 0x10;
    st7735_send_data(data, 16);

    st7735_send_cmd(ST7735_GMCTRN1);
    data[0] = 0x03;
    data[1] = 0x1D;
    data[2] = 0x07;
    data[3] = 0x06;
    data[4] = 0x2E;
    data[5] = 0x2C;
    data[6] = 0x29;
    data[7] = 0x2D;
    data[8] = 0x2E;
    data[9] = 0x2E;
    data[10] = 0x37;
    data[11] = 0x3F;
    data[12] = 0x0;
    data[13] = 0x0;
    data[14] = 0x02;
    data[15] = 0x10;
    st7735_send_data(data, 16);

    st7735_send_cmd(ST7735_NORON);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    st7735_send_cmd(ST7735_DISPON);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
/********************************************************************/
static void spi_test_send(uint8_t *data)
{
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length = 8;
    trans.tx_buffer = data;
    spi_device_polling_transmit(spi_device, &trans);
}
/********************************************************************/
#if DMA_EN
static void spi_pre_trasfer_callback(spi_transaction_t *t)
{
    if ((uint8_t)t->user == 0)
        // gpio_set_level(SPI_DC, 0);
        CLEAR_PERI_REG_MASK(GPIO_OUT_REG, BIT(SPI_DC));
    else
        // gpio_set_level(SPI_DC, 1);
        SET_PERI_REG_MASK(GPIO_OUT_REG, BIT(SPI_DC));
}
#endif
/********************************************************************/
void st7735_init(void)
{
    spi_host_device_t spi_host = SPI_NUM - 1;
    spi_bus_config_t config_spi_bus = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
#if DMA_EN
        .max_transfer_sz = ST7735_WIDTH * ST7735_HEIGHT * 2,
#else
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
#endif
    };

    spi_device_interface_config_t config_spi_device = {
        .clock_speed_hz = SPI_FREQ * 1000 * 1000,
        .mode = 0,
        .spics_io_num = SPI_CS,
        .queue_size = 7,
#if DMA_EN
        .pre_cb = spi_pre_trasfer_callback,
#endif
    };
#if DMA_EN
    spi_bus_initialize(spi_host, &config_spi_bus, SPI_DMA_CH);
#else
    spi_bus_initialize(spi_host, &config_spi_bus, SPI_DMA_DISABLED); 
#endif

    spi_bus_add_device(spi_host, &config_spi_device, &spi_device);

    gpio_set_direction(SPI_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(SPI_RST, GPIO_MODE_OUTPUT);

    // Test, set config
    uint8_t test[2] = {0xFF, 0x00};
    spi_test_send(test);

    gpio_set_level(SPI_RST, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(SPI_RST, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    st7735_send_cmd_list();

    printf("SPI_USER    [0x%08X]\n", READ_PERI_REG(SPI_USER_REG(2)));
    /* printf("SPI_ADDR    [0x%08X]\n", READ_PERI_REG(SPI_ADDR_REG(2)));
    printf("SPI_CMD    [0x%08X]\n", READ_PERI_REG(SPI_CMD_REG(2)));
    printf("SPI_USER    [0x%08X]\n", READ_PERI_REG(SPI_USER_REG(2)));
    printf("SPI_USER1    [0x%08X]\n", READ_PERI_REG(SPI_USER1_REG(2)));
    printf("SPI_USER2   [0x%08X]\n", READ_PERI_REG(SPI_USER2_REG(2)));
    printf("SPI_MOSI_DLEN    [0x%08X]\n", READ_PERI_REG(SPI_MOSI_DLEN_REG(2)));
    printf("SPI_W0_REG    [0x%08X]\n", READ_PERI_REG(SPI_W0_REG(2)));
    printf("SPI_PIN    [0x%08X]\n", READ_PERI_REG(SPI_PIN_REG(2)));
    printf("SPI_EXT2    [0x%08X]\n", READ_PERI_REG(SPI_EXT2_REG(2)));
    printf("SPI_CTRL   [0x%08X]\n", READ_PERI_REG(SPI_CTRL_REG(2)));
    printf("SPI_CLOCK    [0x%08X]\n", READ_PERI_REG(SPI_CLOCK_REG(2))); */
}