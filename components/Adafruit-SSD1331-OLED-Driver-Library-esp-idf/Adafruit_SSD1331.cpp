/*************************************************** 
  This is a library for the 0.96" 16-bit Color OLED with SSD1331 driver chip

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/684

  These displays use SPI to communicate, 4 or 5 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "../Adafruit-SSD1331-OLED-Driver-Library-esp-idf/Adafruit_SSD1331.h"

#include "glcdfont.c"

#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <string.h>

#include "Adafruit_GFX.h"
#include "sdkconfig.h"

static char tag[] = "Adafruit_SSD1331";

extern gpio_dev_t GPIO;

#define PIN_NUM_MOSI CONFIG_HW_LCD_MOSI_GPIO
#define PIN_NUM_CLK  CONFIG_HW_LCD_CLK_GPIO
#define PIN_NUM_CS   CONFIG_HW_LCD_CS_GPIO
#define PIN_NUM_DC   CONFIG_HW_LCD_DC_GPIO
#define PIN_NUM_RST  CONFIG_HW_LCD_RESET_GPIO

#define LCD_SEL_CMD()   GPIO.out_w1tc = (1 << PIN_NUM_DC) // Low to send command
#define LCD_SEL_DATA()  GPIO.out_w1ts = (1 << PIN_NUM_DC) // High to send data
#define LCD_RST_SET()   GPIO.out_w1ts = (1 << PIN_NUM_RST)
#define LCD_RST_CLR()   GPIO.out_w1tc = (1 << PIN_NUM_RST)


#define SPI_NUM  0x3


/********************************** low level pin interface */

void Adafruit_SSD1331::spiwrite(uint8_t d) {

	spi_transaction_t trans_desc;
		trans_desc.address   = 0;
		trans_desc.command   = 0;
		trans_desc.flags     = 0;
		trans_desc.length    = 8;
		trans_desc.rxlength  = 0;
		trans_desc.tx_buffer = &d;
		trans_desc.rx_buffer = NULL;

		ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &trans_desc));
}

void Adafruit_SSD1331::writeCommand(uint8_t c) {

	LCD_SEL_CMD();
	spiwrite(c);
}


void Adafruit_SSD1331::writeData(uint8_t c) {

	LCD_SEL_DATA();
	spiwrite(c);

} 

/***********************************/

void Adafruit_SSD1331::goHome(void) {
  goTo(0,0);
}

void Adafruit_SSD1331::goTo(int x, int y) {
  if ((x >= WIDTH) || (y >= HEIGHT)) return;
  
  // set x and y coordinate
  writeCommand(SSD1331_CMD_SETCOLUMN);
  writeCommand(x);
  writeCommand(x);

  writeCommand(SSD1331_CMD_SETROW);
  writeCommand(y);
  writeCommand(y);
}

uint16_t Adafruit_SSD1331::Color565(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c;
  c = r >> 3;
  c <<= 6;
  c |= g >> 2;
  c <<= 5;
  c |= b >> 3;

  return c;
}

/**************************************************************************/
/*! 
    @brief  Draws a filled rectangle using HW acceleration
*/
/**************************************************************************/
/*
void Adafruit_SSD1331::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t fillcolor)
{
//Serial.println("fillRect");
  // check rotation, move rect around if necessary
  switch (getRotation()) {
  case 1:
	  gfx_swap(x, y);
	  gfx_swap(w, h);
    x = WIDTH - x - 1;
    break;
  case 2:
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    break;
  case 3:
	  gfx_swap(x, y);
	  gfx_swap(w, h);
    y = HEIGHT - y - 1;
    break;
  }

  // Bounds check
  if ((x >= TFTWIDTH) || (y >= TFTHEIGHT))
	return;

  // Y bounds check
  if (y+h > TFTHEIGHT)
  {
    h = TFTHEIGHT - y;
  }

  // X bounds check
  if (x+w > TFTWIDTH)
  {
    w = TFTWIDTH - x;
  }
  
  // fill!
  writeCommand(SSD1331_CMD_FILL);
  writeCommand(0x01);

  writeCommand(SSD1331_CMD_DRAWRECT);
  writeCommand(x & 0xFF);							// Starting column
  writeCommand(y & 0xFF);							// Starting row
  writeCommand((x+w-1) & 0xFF);	// End column
  writeCommand((y+h-1) & 0xFF);	// End row
  
  // Outline color
  writeCommand((uint8_t)((fillcolor >> 11) << 1));
  writeCommand((uint8_t)((fillcolor >> 5) & 0x3F));
  writeCommand((uint8_t)((fillcolor << 1) & 0x3F));
  // Fill color
  writeCommand((uint8_t)((fillcolor >> 11) << 1));
  writeCommand((uint8_t)((fillcolor >> 5) & 0x3F));
  writeCommand((uint8_t)((fillcolor << 1) & 0x3F));
 
  // Delay while the fill completes
  vTaskDelay(SSD1331_DELAYS_HWFILL/portTICK_PERIOD_MS);
}

void Adafruit_SSD1331::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t fillcolor)
{
//Serial.println("fillRect");
  // check rotation, move rect around if necessary
  switch (getRotation()) {
  case 1:
	  gfx_swap(x, y);
	  gfx_swap(w, h);
    x = WIDTH - x - 1;
    break;
  case 2:
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    break;
  case 3:
	  gfx_swap(x, y);
	  gfx_swap(w, h);
    y = HEIGHT - y - 1;
    break;
  }

  // Bounds check
  if ((x >= TFTWIDTH) || (y >= TFTHEIGHT))
	return;

  // Y bounds check
  if (y+h > TFTHEIGHT)
  {
    h = TFTHEIGHT - y;
  }

  // X bounds check
  if (x+w > TFTWIDTH)
  {
    w = TFTWIDTH - x;
  }

  // fill!
  writeCommand(SSD1331_CMD_FILL);
  writeCommand(0x00);

  writeCommand(SSD1331_CMD_DRAWRECT);
  writeCommand(x & 0xFF);							// Starting column
  writeCommand(y & 0xFF);							// Starting row
  writeCommand((x+w-1) & 0xFF);	// End column
  writeCommand((y+h-1) & 0xFF);	// End row

  // Outline color
  writeCommand((uint8_t)((fillcolor >> 11) << 1));
  writeCommand((uint8_t)((fillcolor >> 5) & 0x3F));
  writeCommand((uint8_t)((fillcolor << 1) & 0x3F));
  // Fill color
  writeCommand((uint8_t)((fillcolor >> 11) << 1));
  writeCommand((uint8_t)((fillcolor >> 5) & 0x3F));
  writeCommand((uint8_t)((fillcolor << 1) & 0x3F));

  // Delay while the fill completes
  vTaskDelay(SSD1331_DELAYS_HWFILL/portTICK_PERIOD_MS);
}

void Adafruit_SSD1331::setScrolling(int16_t hOffset, int16_t x0, int16_t hScroll, int16_t vScroll, uint16_t interval) {
	  writeCommand(SSD1331_CMD_SCROLLING);
	  writeCommand(hOffset);
	  writeCommand(x0);
	  writeCommand(hScroll);
	  writeCommand(vScroll);
	  writeCommand(interval);
	  vTaskDelay(SSD1331_DELAYS_HWFILL/portTICK_PERIOD_MS);
}
*/
void Adafruit_SSD1331::clearScreen() {
	clearWindow(0, 0, WIDTH-1, HEIGHT-1);
	vTaskDelay(SSD1331_DELAYS_HWFILL/portTICK_PERIOD_MS);
}
/*
void Adafruit_SSD1331::scroll(bool scroll) {
	if(scroll)
		writeCommand(SSD1331_CMD_SCROLLON);
	else
		writeCommand(SSD1331_CMD_SCROLLOFF);
}
*/
void Adafruit_SSD1331::clearWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1){
	writeCommand(SSD1331_CMD_CLEARWINDOW);
	writeCommand(x0);
	writeCommand(y0);
	writeCommand(x1);
	writeCommand(y1);

	vTaskDelay(SSD1331_DELAYS_HWFILL/portTICK_PERIOD_MS);
}
/*
void Adafruit_SSD1331::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {	
  // check rotation, move pixel around if necessary
  switch (getRotation()) {
  case 1:
    gfx_swap(x0, y0);
    gfx_swap(x1, y1);
    x0 = WIDTH - x0 - 1;
    x1 = WIDTH - x1 - 1;
    break;
  case 2:
    x0 = WIDTH - x0 - 1;
    y0 = HEIGHT - y0 - 1;
    x1 = WIDTH - x1 - 1;
    y1 = HEIGHT - y1 - 1;
    break;
  case 3:
    gfx_swap(x0, y0);
    gfx_swap(x1, y1);
    y0 = HEIGHT - y0 - 1;
    y1 = HEIGHT - y1 - 1;
    break;
  }

  // Boundary check
  if ((y0 >= TFTHEIGHT) && (y1 >= TFTHEIGHT))
	return;
  if ((x0 >= TFTWIDTH) && (x1 >= TFTWIDTH))
	return;	
  if (x0 >= TFTWIDTH)
    x0 = TFTWIDTH - 1;
  if (y0 >= TFTHEIGHT)
    y0 = TFTHEIGHT - 1;
  if (x1 >= TFTWIDTH)
    x1 = TFTWIDTH - 1;
  if (y1 >= TFTHEIGHT)
    y1 = TFTHEIGHT - 1;
  
  writeCommand(SSD1331_CMD_DRAWLINE);
  writeCommand(x0);
  writeCommand(y0);
  writeCommand(x1);
  writeCommand(y1);
  vTaskDelay(SSD1331_DELAYS_HWLINE/portTICK_PERIOD_MS);
  writeCommand((uint8_t)((color >> 11) << 1));
  writeCommand((uint8_t)((color >> 5) & 0x3F));
  writeCommand((uint8_t)((color << 1) & 0x3F));
  vTaskDelay(SSD1331_DELAYS_HWLINE/portTICK_PERIOD_MS);
}
//*/
void Adafruit_SSD1331::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height())) return;

  // check rotation, move pixel around if necessary
  switch (getRotation()) {
  case 1:
    gfx_swap(x, y);
    x = WIDTH - x - 1;
    break;
  case 2:
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    break;
  case 3:
    gfx_swap(x, y);
    y = HEIGHT - y - 1;
    break;
  }

  goTo(x, y);

  writeData((uint8_t)((color >> 11) << 1));
  writeData((uint8_t)((color >> 5) & 0x3F));
  writeData((uint8_t)((color << 1) & 0x3F));
}
/*
void Adafruit_SSD1331::pushColor(uint16_t color) {
	writeData((uint8_t)((color >> 11) << 1));
	writeData((uint8_t)((color >> 5) & 0x3F));
	writeData((uint8_t)((color << 1) & 0x3F));
}
*/

void Adafruit_SSD1331::begin(){

	gpio_set_direction((gpio_num_t)_dc, GPIO_MODE_OUTPUT);
	gpio_set_direction((gpio_num_t)_sclk, GPIO_MODE_OUTPUT);
	gpio_set_direction((gpio_num_t)_sid, GPIO_MODE_OUTPUT);

		ESP_LOGI(tag, "test abcd: sclk: %d, sid: %d", _sclk, _sid);
		spi_bus_config_t bus_config;
		bus_config.sclk_io_num   = _sclk; // CLK
		bus_config.mosi_io_num   = _sid; // MOSI
		bus_config.miso_io_num   = -1; // MISO
		bus_config.quadwp_io_num = -1; // Not used
		bus_config.quadhd_io_num = -1; // Not used
		ESP_LOGI(tag, "... Initializing bus.");
		ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &bus_config, 0));

		spi_device_interface_config_t dev_config;
		dev_config.address_bits     = 0;
		dev_config.command_bits     = 0;
		dev_config.dummy_bits       = 0;
		dev_config.mode             = 0;
		dev_config.duty_cycle_pos   = 0;
		dev_config.cs_ena_posttrans = 0;
		dev_config.cs_ena_pretrans  = 0;
		dev_config.clock_speed_hz   = 10000000; // 100KHz
		dev_config.spics_io_num     = _cs;
		dev_config.flags            = 0;
		dev_config.queue_size       = 1;
		dev_config.pre_cb           = NULL;
		dev_config.post_cb          = NULL;
		ESP_LOGI(tag, "... Adding device bus.");
		ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &dev_config, &spi_handle));


	  if (_rst >= 0) {
	    // Setup reset pin direction (used by both SPI and I2C)

	    gpio_set_direction((gpio_num_t)_rst, GPIO_MODE_OUTPUT);

	  	gpio_set_level((gpio_num_t)_rst, 1);
	    // VDD (3.3V) goes high at start, lets just chill for a ms

	    vTaskDelay(1/portTICK_PERIOD_MS);
	    // bring reset low

	  	gpio_set_level((gpio_num_t)_rst, 0);
	    // wait 10ms

	    vTaskDelay(10/portTICK_PERIOD_MS);
	    // bring out of reset

	  	gpio_set_level((gpio_num_t)_rst, 1);
	    // turn on VCC (9V?)
	  }

    // Initialization Sequence
    writeCommand(SSD1331_CMD_DISPLAYOFF);  	// 0xAE
    writeCommand(SSD1331_CMD_SETREMAP); 	// 0xA0
#if defined SSD1331_COLORORDER_RGB
    writeCommand(0xA0);				// RGB Color
#else
    writeCommand(0x84);				// BGR Color
#endif
    writeCommand(SSD1331_CMD_STARTLINE); 	// 0xA1
    writeCommand(0x0);
    writeCommand(SSD1331_CMD_DISPLAYOFFSET); 	// 0xA2
    writeCommand(0x0);
    writeCommand(SSD1331_CMD_NORMALDISPLAY);  	// 0xA4
    writeCommand(SSD1331_CMD_SETMULTIPLEX); 	// 0xA8
    writeCommand(0x3F);  			// 0x3F 1/64 duty
    writeCommand(SSD1331_CMD_SETMASTER);  	// 0xAD
    writeCommand(0x8E);
    writeCommand(SSD1331_CMD_POWERMODE);  	// 0xB0
    writeCommand(0x0B);
    writeCommand(SSD1331_CMD_PRECHARGE);  	// 0xB1
    writeCommand(0x31);
    writeCommand(SSD1331_CMD_CLOCKDIV);  	// 0xB3
    writeCommand(0xD0);  // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    writeCommand(SSD1331_CMD_PRECHARGEA);  	// 0x8A
    writeCommand(0x64);
    writeCommand(SSD1331_CMD_PRECHARGEB);  	// 0x8B
    writeCommand(0x78);
    writeCommand(SSD1331_CMD_PRECHARGEC);  	// 0x8C
    writeCommand(0x64);
    writeCommand(SSD1331_CMD_PRECHARGELEVEL);  	// 0xBB
    writeCommand(0x3A);
    writeCommand(SSD1331_CMD_VCOMH);  		// 0xBE
    writeCommand(0x3E);
    writeCommand(SSD1331_CMD_MASTERCURRENT);  	// 0x87
    writeCommand(0x06);
    writeCommand(SSD1331_CMD_CONTRASTA);  	// 0x81
    writeCommand(0x91);
    writeCommand(SSD1331_CMD_CONTRASTB);  	// 0x82
    writeCommand(0x50);
    writeCommand(SSD1331_CMD_CONTRASTC);  	// 0x83
    writeCommand(0x7D);
    writeCommand(SSD1331_CMD_DISPLAYON);	//--turn on oled panel

	ESP_LOGI(tag, "... Ready device.");

}

/********************************* low level pin initialization */

Adafruit_SSD1331::Adafruit_SSD1331(uint8_t cs, uint8_t dc, uint8_t sid, uint8_t sclk, uint8_t rst) : Adafruit_GFX(TFTWIDTH, TFTHEIGHT) {
    _cs = cs;
    _dc = dc;
    _sid = sid;
    _sclk = sclk;
    _rst = rst;
}

Adafruit_SSD1331::Adafruit_SSD1331(uint8_t cs, uint8_t rs, uint8_t rst) : Adafruit_GFX(TFTWIDTH, TFTHEIGHT) {
    _cs = cs;
    _dc = rs;
    _sid = 0;
    _sclk = 0;
    _rst = rst;
}
