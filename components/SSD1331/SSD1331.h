/*
 * SSD1331.h
 *
 *  Created on: 15.06.2017
 *      Author: darek
 */
#include <Adafruit_GFX.h>

#ifndef _SSD1331_H_
#define _SSD1331_H_

#define RGB(R,G,B)  (((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3))
enum Color{
    BLACK     = RGB(  0,  0,  0), // black
    GREY      = RGB(192,192,192), // grey
    WHITE     = RGB(255,255,255), // white
    RED       = RGB(255,  0,  0), // red
    PINK      = RGB(255,192,203), // pink
    YELLOW    = RGB(255,255,  0), // yellow
    GOLDEN    = RGB(255,215,  0), // golden
    BROWN     = RGB(128, 42, 42), // brown
    BLUE      = RGB(  0,  0,255), // blue
    CYAN      = RGB(  0,255,255), // cyan
    GREEN     = RGB(  0,255,  0), // green
    PURPLE    = RGB(160, 32,240), // purple
};

#define SSD1331_COLORORDER_RGB
// #define SSD1331_COLORORDER_BGR

#if defined SSD1331_COLORORDER_RGB && defined SSD1331_COLORORDER_BGR
  #error "RGB and BGR can not both be defined for SSD1331_COLORODER."
#endif


// Timing Delays
#define SSD1331_DELAYS_HWFILL		(3)
#define SSD1331_DELAYS_HWLINE       (1)

// SSD1331 Commands
#define SSD1331_CMD_DRAWLINE 		0x21
#define SSD1331_CMD_DRAWRECT 		0x22
#define SSD1331_CMD_FILL 			0x26
//---------------------------------------------------//
#define SSD1331_CMD_COPY			0x23
#define SSD1331_CMD_DIMWINDOW		0x24
#define SSD1331_CMD_CLEARWINDOW		0x25
#define SSD1331_CMD_SCROLLING		0x27
#define SSD1331_CMD_SCROLLOFF		0x2E
#define SSD1331_CMD_SCROLLON		0x2F
//---------------------------------------------------//
#define SSD1331_CMD_SETCOLUMN 		0x15
#define SSD1331_CMD_SETROW    		0x75
#define SSD1331_CMD_CONTRASTA 		0x81
#define SSD1331_CMD_CONTRASTB 		0x82
#define SSD1331_CMD_CONTRASTC		0x83
#define SSD1331_CMD_MASTERCURRENT 	0x87
#define SSD1331_CMD_SETREMAP 		0xA0
#define SSD1331_CMD_STARTLINE 		0xA1
#define SSD1331_CMD_DISPLAYOFFSET 	0xA2
#define SSD1331_CMD_NORMALDISPLAY 	0xA4
#define SSD1331_CMD_DISPLAYALLON  	0xA5
#define SSD1331_CMD_DISPLAYALLOFF 	0xA6
#define SSD1331_CMD_INVERTDISPLAY 	0xA7
#define SSD1331_CMD_SETMULTIPLEX  	0xA8
#define SSD1331_CMD_SETMASTER 		0xAD
#define SSD1331_CMD_DISPLAYOFF 		0xAE
#define SSD1331_CMD_DISPLAYON     	0xAF
#define SSD1331_CMD_POWERMODE 		0xB0
#define SSD1331_CMD_PRECHARGE 		0xB1
#define SSD1331_CMD_CLOCKDIV 		0xB3
#define SSD1331_CMD_PRECHARGEA 		0x8A
#define SSD1331_CMD_PRECHARGEB 		0x8B
#define SSD1331_CMD_PRECHARGEC 		0x8C
#define SSD1331_CMD_PRECHARGELEVEL 	0xBB
#define SSD1331_CMD_VCOMH 			0xBE

class SSD1331 : public virtual Adafruit_GFX {
public:
	SSD1331();
	void begin();
	void writeFrame(const uint16_t x, const uint16_t y, const uint16_t width, const uint16_t height, const uint8_t *data[]);
	  void drawPixel(int16_t x, int16_t y, uint16_t color);
	  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
//	  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
/*	  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	  void pushColor(uint16_t c);
	  void setScrolling(int16_t hOffset, int16_t x0, int16_t hScroll, int16_t vScroll, uint16_t interval);
	  void scroll(bool scroll);*/
	  void clearWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
	  void clearScreen();

	  // commands
	  void goHome(void);
	  void goTo(int x, int y);

	static const int16_t TFTWIDTH = 96;
	  static const int16_t TFTHEIGHT = 64;
//private:
	void gpioInit();
	void spiMasterInit();

	void writeData(uint8_t data);
	void writeCommand(uint8_t data);

	void writeByte(const uint8_t data);

};

#endif /* COMPONENTS_SSD1331_SSD1331_H_ */
