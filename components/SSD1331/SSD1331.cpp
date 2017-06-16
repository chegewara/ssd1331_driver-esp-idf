/*
 * SSD1331.cpp
 *
 *  Created on: 15.06.2017
 *      Author: darek
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "SSD1331.h"

#include "sdkconfig.h"

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

void SSD1331::writeByte(const uint8_t data){
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 0x7, SPI_USR_MOSI_DBITLEN_S);
    WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), data);
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
}

void SSD1331::writeCommand(uint8_t cmd)
{
    LCD_SEL_CMD();
    writeByte(cmd);
}

void SSD1331::writeData(uint8_t data)
{
    LCD_SEL_DATA();
    writeByte(data);
}


SSD1331::SSD1331() : Adafruit_GFX(TFTWIDTH, TFTHEIGHT){
	// TODO Auto-generated constructor stub

}

#define gfx_swap(a, b) { uint16_t t = a; a = b; b = t; }

/**************************************************************************/
/*!
    @brief  Draws a filled rectangle using HW acceleration
*/
/**************************************************************************/
/*
void SSD1331::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t fillcolor)
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

void SSD1331::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t fillcolor)
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

void SSD1331::setScrolling(int16_t hOffset, int16_t x0, int16_t hScroll, int16_t vScroll, uint16_t interval) {
	  writeCommand(SSD1331_CMD_SCROLLING);
	  writeCommand(hOffset);
	  writeCommand(x0);
	  writeCommand(hScroll);
	  writeCommand(vScroll);
	  writeCommand(interval);
	  vTaskDelay(SSD1331_DELAYS_HWFILL/portTICK_PERIOD_MS);
}
*/
void SSD1331::clearScreen() {
	clearWindow(0, 0, WIDTH-1, HEIGHT-1);
}
/*
void SSD1331::scroll(bool scroll) {
	if(scroll)
		writeCommand(SSD1331_CMD_SCROLLON);
	else
		writeCommand(SSD1331_CMD_SCROLLOFF);
}
*/
void SSD1331::clearWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1){
	writeCommand(SSD1331_CMD_CLEARWINDOW);
	writeCommand(x0);
	writeCommand(y0);
	writeCommand(x1);
	writeCommand(y1);

	vTaskDelay(SSD1331_DELAYS_HWFILL/portTICK_PERIOD_MS);
}

void SSD1331::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
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

void SSD1331::drawPixel(int16_t x, int16_t y, uint16_t color) {

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

void SSD1331::goHome(void) {
  goTo(0,0);
}

void SSD1331::goTo(int x, int y) {
//  if ((x >= WIDTH) || (y >= HEIGHT)) return;

  // set x and y coordinate
  writeCommand(SSD1331_CMD_SETCOLUMN);
  writeCommand(x);
  writeCommand(x);

  writeCommand(SSD1331_CMD_SETROW);
  writeCommand(y);
  writeCommand(y);
}

void SSD1331::begin() {
	spiMasterInit();
	gpioInit();


    LCD_RST_SET();
    ets_delay_us(100000);

    LCD_RST_CLR();
    ets_delay_us(200000);

    LCD_RST_SET();
    ets_delay_us(200000);



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

}

void SSD1331::gpioInit() {
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO16_U,2);   //DC PIN
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO17_U,2);   //RESET PIN
//    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,2);    //BKL PIN
    WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, BIT16|BIT17);
}

void SSD1331::spiMasterInit()
{
    ets_printf("lcd spi pin mux init ...\r\n");
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO18_U,2);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO23_U,2);
//    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO22_U,2);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,2);
    WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, BIT18|BIT23|BIT5);

    ets_printf("lcd spi signal init\r\n");
    gpio_matrix_out(PIN_NUM_MOSI, VSPID_OUT_IDX,0,0);
    gpio_matrix_out(PIN_NUM_CLK, VSPICLK_OUT_IDX,0,0);
    gpio_matrix_out(PIN_NUM_CS, VSPICS0_OUT_IDX,0,0);
    ets_printf("Hspi config\r\n");

//    CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(SPI_NUM), SPI_TRANS_DONE << 5);
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP);
    CLEAR_PERI_REG_MASK(SPI_PIN_REG(SPI_NUM), SPI_CK_IDLE_EDGE);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM),  SPI_CK_OUT_EDGE);
    CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_NUM), SPI_WR_BIT_ORDER);
    CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_NUM), SPI_RD_BIT_ORDER);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_DOUTDIN);
    WRITE_PERI_REG(SPI_USER1_REG(SPI_NUM), 0);
//    SET_PERI_REG_BITS(SPI_CTRL2_REG(SPI_NUM), SPI_MISO_DELAY_MODE, 0, SPI_MISO_DELAY_MODE_S);
//    CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(SPI_NUM), SPI_SLAVE_MODE);

    WRITE_PERI_REG(SPI_CLOCK_REG(SPI_NUM), (1 << SPI_CLKCNT_N_S) | (1 << SPI_CLKCNT_L_S));//40MHz
    //WRITE_PERI_REG(SPI_CLOCK_REG(SPI_NUM), SPI_CLK_EQU_SYSCLK); // 80Mhz

    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP | SPI_CS_HOLD | SPI_USR_MOSI);
//    SET_PERI_REG_MASK(SPI_CTRL2_REG(SPI_NUM), ((0x4 & SPI_MISO_DELAY_NUM) << SPI_MISO_DELAY_NUM_S));
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_COMMAND);
    SET_PERI_REG_BITS(SPI_USER2_REG(SPI_NUM), SPI_USR_COMMAND_BITLEN, 0, SPI_USR_COMMAND_BITLEN_S);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_ADDR);
    SET_PERI_REG_BITS(SPI_USER1_REG(SPI_NUM), SPI_USR_ADDR_BITLEN, 0, SPI_USR_ADDR_BITLEN_S);
//    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MISO);
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MOSI);
    char i;
    for (i = 0; i < 16; ++i) {
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), 0);
    }
}

/*
#define U16x2toU32(m,l) ((((uint32_t)(l>>8|(l&0xFF)<<8))<<16)|(m>>8|(m&0xFF)<<8))

uint16_t myPalette[96*64*3];

void SSD1331::writeFrame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t *data[]) {
	int x, y;
	int i;
	uint16_t x1, y1;
	uint32_t xv, yv, dc;
	uint32_t temp[16];
	dc = (1 << PIN_NUM_DC);

	for (y=0; y<height; y++) {
		//start line
		x1 = xs+(width-1);
		y1 = ys+y+(height-1);
		xv = U16x2toU32(xs,x1);
		yv = U16x2toU32((ys+y),y1);

		while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
		GPIO.out_w1tc = dc;
		SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
		WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2A);
		SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
		while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
		GPIO.out_w1ts = dc;
		SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
		WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), xv);
		SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
		while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
		GPIO.out_w1tc = dc;
		SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
		WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2B);
		SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
		while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
		GPIO.out_w1ts = dc;
		SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
		WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), yv);
		SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
		while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
		GPIO.out_w1tc = dc;
		SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
		WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2C);
		SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
		while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);

		x = 0;
		GPIO.out_w1ts = dc;
		SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);
		while (x<width) {
			for (i=0; i<16; i++) {
				if(data == NULL){
					temp[i] = 0;
					x += 2;
					continue;
				}
				x1 = myPalette[(unsigned char)(data[y][x])]; x++;
				y1 = myPalette[(unsigned char)(data[y][x])]; x++;
				temp[i] = U16x2toU32(x1,y1);
			}
			while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
			for (i=0; i<16; i++) {
				WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), temp[i]);
			}
			SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
		}
	}
	while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
}
*/
