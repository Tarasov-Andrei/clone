#include "ili9488_i80.h"
#include "lcd_i80_bus.h"
#include "sdkconfig.h"
#include "stdio.h"
#include "esp_log.h"

/************ LCD commands ***************/
#define ILI9488_PGAMCTRL 0xE0   // Positive gamma control
#define ILI9488_NGAMCTRL 0xE1   // Negative gamma control
#define ILI9488_PWCTRL1 0xC0    // Power control 1
#define ILI9488_PWCTRL2 0xC1    // Power control 2
#define ILI9488_PWCTRL3 0xC5    // Power control 3
#define ILI9488_MADCTL 0x36     // Memory data access control
#define ILI9488_COLMOD 0x3A     // Defines the format of RGB picture data 16, 18, 24 bits/px
#define ILI9488_INTMODCTRL 0xB0 // Interface mode control
#define ILI9488_FRAMECTRL 0xB1  // Frame rate control
#define ILI9488_INVCTRL 0xB4    // Display inversion control
#define ILI9488_DISCTRL 0xB6    // Display function control
#define ILI9488_SETIMAGE 0xE9   // Set image function
#define ILI9488_ADJCTRL 0xF7    // Adjust control
#define ILI9488_SLPOUT 0x11     // Exit sleep mode
#define ILI9488_DISPON 0x29     // Display on (enable frame buffer output)
#define ILI9488_CASET 0x2A      // Set column address
#define ILI9488_RASET 0x2B      // Set row address
#define ILI9488_RAMWR 0x2C      // Write frame memory

/* #define LCD_CMD_NOP 0x00        // This command is empty command
#define LCD_CMD_SWRESET 0x01    // Software reset registers (the built-in frame buffer is not affected)
#define LCD_CMD_RDDID 0x04      // Read 24-bit display ID
#define LCD_CMD_RDDST 0x09      // Read display status
#define LCD_CMD_RDDPM 0x0A      // Read display power mode
#define LCD_CMD_RDD_MADCTL 0x0B // Read display MADCTL
#define LCD_CMD_RDD_COLMOD 0x0C // Read display pixel format
#define LCD_CMD_RDDIM 0x0D      // Read display image mode
#define LCD_CMD_RDDSM 0x0E      // Read display signal mode
#define LCD_CMD_RDDSR 0x0F      // Read display self-diagnostic result
#define LCD_CMD_SLPIN 0x10      // Go into sleep mode (DC/DC, oscillator, scanning stopped, but memory keeps content)
#define LCD_CMD_SLPOUT 0x11     // Exit sleep mode
#define LCD_CMD_PTLON 0x12      // Turns on partial display mode
#define LCD_CMD_NORON 0x13      // Turns on normal display mode
#define LCD_CMD_INVOFF 0x20     // Recover from display inversion mode
#define LCD_CMD_INVON 0x21      // Go into display inversion mode
#define LCD_CMD_GAMSET 0x26     // Select Gamma curve for current display
#define LCD_CMD_DISPOFF 0x28    // Display off (disable frame buffer output)
#define LCD_CMD_DISPON 0x29     // Display on (enable frame buffer output)
#define LCD_CMD_CASET 0x2A      // Set column address
#define LCD_CMD_RASET 0x2B      // Set row address
#define LCD_CMD_RAMWR 0x2C      // Write frame memory
#define LCD_CMD_RAMRD 0x2E      // Read frame memory
#define LCD_CMD_PTLAR 0x30      // Define the partial area
#define LCD_CMD_VSCRDEF 0x33    // Vertical scrolling definition
#define LCD_CMD_TEOFF 0x34      // Turns of tearing effect
#define LCD_CMD_TEON 0x35       // Turns on tearing effect

#define LCD_CMD_MADCTL 0x36      // Memory data access control
#define LCD_CMD_MH_BIT (1 << 2)  // Display data latch order, 0: refresh left to right, 1: refresh right to left
#define LCD_CMD_BGR_BIT (1 << 3) // RGB/BGR order, 0: RGB, 1: BGR
#define LCD_CMD_ML_BIT (1 << 4)  // Line address order, 0: refresh top to bottom, 1: refresh bottom to top
#define LCD_CMD_MV_BIT (1 << 5)  // Row/Column order, 0: normal mode, 1: reverse mode
#define LCD_CMD_MX_BIT (1 << 6)  // Column address order, 0: left to right, 1: right to left
#define LCD_CMD_MY_BIT (1 << 7)  // Row address order, 0: top to bottom, 1: bottom to top

#define LCD_CMD_VSCSAD 0x37  // Vertical scroll start address
#define LCD_CMD_IDMOFF 0x38  // Recover from IDLE mode
#define LCD_CMD_IDMON 0x39   // Fall into IDLE mode (8 color depth is displayed)
#define LCD_CMD_COLMOD 0x3A  // Defines the format of RGB picture data
#define LCD_CMD_RAMWRC 0x3C  // Memory write continue
#define LCD_CMD_RAMRDC 0x3E  // Memory read continue
#define LCD_CMD_STE 0x44     // Set tear scanline, tearing effect output signal when display module reaches line N
#define LCD_CMD_GDCAN 0x45   // Get scanline
#define LCD_CMD_WRDISBV 0x51 // Write display brightness
#define LCD_CMD_RDDISBV 0x52 // Read display brightness value */
static uint16_t lcd_w = 0;
static uint16_t lcd_h = 0;
/********************************************************************/

static void lcd_send_cmd(uint8_t cmd)
{
    while (!(i2s.state.tx_idle))
    {
    }
    GPIO.out_w1tc = (1 << LCD_DC); // DC -> 0
    i2s.conf.tx_start = 0;         // TX stop
    i2s.conf.tx_reset = 1;         // TX reset
    i2s.conf.tx_reset = 0;         // TX reset

    i2s.fifo_conf.tx_fifo_mod = 3; // [1]-16 bit single, [3]-32 bit single
    i2s.fifo_conf.tx_data_num = 8; // TX Threshold data lenght
    i2s.sample_rate_conf.tx_bits_mod = 8;

    i2s.fifo_wr = cmd << 16; // Write to FIFO [23:16]
    i2s.conf.tx_start = 1;   // TX Start
}
/*********************************************/
static void lcd_send_data8n(uint8_t *data, uint8_t len)
{
    while (!(i2s.state.tx_idle))
    {
    }
    GPIO.out_w1ts = (1 << LCD_DC); // DC -> 1
    i2s.conf.tx_start = 0;         // TX stop
    i2s.conf.tx_reset = 1;         // TX reset
    i2s.conf.tx_reset = 0;         // TX reset

    i2s.fifo_conf.tx_fifo_mod = 3; // [1]-16 bit single, [3]-32 bit single
    i2s.fifo_conf.tx_data_num = 8; // TX Threshold data lenght
    i2s.sample_rate_conf.tx_bits_mod = 8;

    i2s.conf.tx_start = 1; // TX Start
    while (len > 0)
    {
        i2s.fifo_wr = *data << 16; // Write to FIFO [23:16]
        data++;
        len--;
    }
}
/********************************************************************/
static void lcd_send_data16(uint16_t data, uint32_t len)
{

    //uint8_t cnt_tx = 0;
    while (!(i2s.state.tx_idle))
    {
    }
    GPIO.out_w1ts = (1 << LCD_DC); // DC -> 1
    i2s.conf.tx_start = 0;         // TX stop
    i2s.conf.tx_reset = 1;         // TX reset
    i2s.conf.tx_reset = 0;         // TX reset

    i2s.fifo_conf.tx_fifo_mod = 1;  // [1]-16 bit single, [3]-32 bit single
    i2s.fifo_conf.tx_data_num = 16; // TX Threshold data lenght
    i2s.sample_rate_conf.tx_bits_mod = 8;

    i2s.fifo_wr = (uint32_t)(data << 8) | data; // Write to FIFO [23:16] + [7:0]
    i2s.conf.tx_start = 1;                      // TX Start

    while (len - 1 > 0)
    {
        i2s.fifo_wr = (uint32_t)(data << 8) | data; // Write to FIFO [23:16] + [7:0]
        len--;
    }
}
/********************************************************************/
static void lcd_send_data16n(uint16_t *data, uint32_t len)
{
    while (!(i2s.state.tx_idle))
    {
    }
    GPIO.out_w1ts = (1 << LCD_DC); // DC -> 1
    i2s.conf.tx_start = 0;         // TX stop
    i2s.conf.tx_reset = 1;         // TX reset
    i2s.conf.tx_reset = 0;         // TX reset

    i2s.fifo_conf.tx_fifo_mod = 1;  // [1]-16 bit single, [3]-32 bit single
    i2s.fifo_conf.tx_data_num = 16; // TX Threshold data lenght
    i2s.sample_rate_conf.tx_bits_mod = 8;

    i2s.conf.tx_start = 1; // TX Start
    while (len > 0)
    {
        i2s.fifo_wr = (*data << 8) | *data; // Write to FIFO [23:16] + [7:0]
        data++;
        len--;
    }
}
/********************************************************************/
static void lcd_set_address_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    lcd_send_cmd(ILI9488_CASET);
    uint16_t data[] = {x0, x1};
    lcd_send_data16n(data, 2);

    lcd_send_cmd(ILI9488_RASET);
    data[0] = y0;
    data[1] = y1;
    // data[2] = y1 >> 8;
    // data[3] = y1;
    lcd_send_data16n(data, 2);

    lcd_send_cmd(ILI9488_RAMWR);
}
/********************************************************************/
void lcd_h_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t color)
{
    lcd_fill_rect(x0, y0, x1, y0, color);
}
/********************************************************************/
void lcd_v_line(uint16_t x0, uint16_t y0, uint16_t y1, uint16_t color)
{
    lcd_fill_rect(x0, y0, x0, y1, color);
}
/********************************************************************/
void lcd_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    lcd_h_line(x0, y0, x1, color);
    lcd_h_line(x0, y1, x1, color);
    lcd_v_line(x0, y0 + 1, y1 - 1, color);
    lcd_v_line(x1, y0 + 1, y1 - 1, color);
}
/********************************************************************/
void lcd_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    lcd_set_address_window(x0, y0, x1, y1);
    lcd_send_data16(color, ((x0 + x1 + 1) * (y0 + y1 + 1))); // w*h*16
}
/********************************************************************/
void lcd_fill_screen(uint16_t color)
{
    lcd_fill_rect(0, 0, lcd_w - 1, lcd_h - 1, color);
}
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/

void ili9488_i80_init(uint16_t lcd_width, uint16_t lcd_heght)
{
    lcd_w = lcd_width;
    lcd_h = lcd_heght;
    uint8_t data[15];

    lcd_send_cmd(ILI9488_PGAMCTRL);
    data[0] = 0x00;
    data[1] = 0x03;
    data[2] = 0x09;
    data[3] = 0x08;
    data[4] = 0x16;
    data[5] = 0x0A;
    data[6] = 0x3F;
    data[7] = 0x78;
    data[8] = 0x4C;
    data[9] = 0x09;
    data[10] = 0x0A;
    data[11] = 0x08;
    data[12] = 0x16;
    data[13] = 0x1A;
    data[14] = 0x0F;
    lcd_send_data8n(data, 15);

    lcd_send_cmd(ILI9488_NGAMCTRL);
    data[0] = 0x00;
    data[1] = 0x16;
    data[2] = 0x19;
    data[3] = 0x03;
    data[4] = 0x0F;
    data[5] = 0x05;
    data[6] = 0x32;
    data[7] = 0x45;
    data[8] = 0x46;
    data[9] = 0x04;
    data[10] = 0x0E;
    data[11] = 0x0D;
    data[12] = 0x35;
    data[13] = 0x37;
    data[14] = 0x0F;
    lcd_send_data8n(data, 15);

    lcd_send_cmd(ILI9488_PWCTRL1);
    data[0] = 0x17; // Vreg1out
    data[1] = 0x15; // Vreg2out
    lcd_send_data8n(data, 2);

    lcd_send_cmd(ILI9488_PWCTRL2);
    data[0] = 0x41; // VGH,VGL
    lcd_send_data8n(data, 1);

    lcd_send_cmd(ILI9488_PWCTRL3);
    data[0] = 0x00;
    data[1] = 0x12; // Vcom
    data[2] = 0x80;
    lcd_send_data8n(data, 3);

    lcd_send_cmd(ILI9488_MADCTL);
    data[0] = 0x48;
    lcd_send_data8n(data, 1);

    lcd_send_cmd(ILI9488_COLMOD);
    data[0] = 0x55; // 16 bit color
    lcd_send_data8n(data, 1);

    lcd_send_cmd(ILI9488_INTMODCTRL);
    data[0] = 0x80; // SDO not use
    lcd_send_data8n(data, 1);

    lcd_send_cmd(ILI9488_FRAMECTRL);
    data[0] = 0xA0; // 60Hz
    lcd_send_data8n(data, 1);

    lcd_send_cmd(ILI9488_INVCTRL);
    data[0] = 0x02;
    lcd_send_data8n(data, 1);

    lcd_send_cmd(ILI9488_INVCTRL);
    data[0] = 0x02; // MCU
    lcd_send_data8n(data, 1);

    lcd_send_cmd(ILI9488_SETIMAGE);
    data[0] = 0x00; // Disable 24 bit data
    lcd_send_data8n(data, 1);

    lcd_send_cmd(ILI9488_ADJCTRL);
    data[0] = 0xA9;
    data[1] = 0x51;
    data[2] = 0x2C;
    data[3] = 0x82;
    lcd_send_data8n(data, 4); // D7 stream, loose

    lcd_send_cmd(ILI9488_SLPOUT); // Exit sleep
    esp_rom_delay_us(120 * 1000);
    lcd_send_cmd(ILI9488_DISPON); // Display on
    lcd_send_cmd(ILI9488_RAMWR);
}