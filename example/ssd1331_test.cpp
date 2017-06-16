#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "benchmark.h"

#include "sdkconfig.h"

static char tag[] = "SSD1331_test";

#if defined USE_ADAFRUIT_LIB
#define PIN_NUM_MOSI CONFIG_HW_LCD_MOSI_GPIO
#define PIN_NUM_CLK  CONFIG_HW_LCD_CLK_GPIO
#define PIN_NUM_CS   CONFIG_HW_LCD_CS_GPIO
#define PIN_NUM_DC   CONFIG_HW_LCD_DC_GPIO
#define PIN_NUM_RST  CONFIG_HW_LCD_RESET_GPIO

	Adafruit_SSD1331 disp = Adafruit_SSD1331(PIN_NUM_CS, PIN_NUM_DC, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_RST);
#else
	SSD1331 disp = SSD1331();
#endif
extern "C" {
	void ssd1331_test(void *ignore);
}

void ssd1331_test(void *ignore) {
	ESP_LOGD(tag, ">> entry point ssd1331_test");
	disp.begin();
	disp.clearScreen();
	Benchmark test = Benchmark(&disp);
	disp.setTextWrap(0);
	while(1){
		for(uint8_t r=0;r<4;r++){
			disp.setRotation(r);
			test.testText();
			vTaskDelay(1000);
			disp.clearScreen();
			test.testBox3d();
			vTaskDelay(1000);
			disp.clearScreen();
			test.testLines();
			vTaskDelay(500);
			disp.clearScreen();
			for(int a=0;a<360;a+=1){
				test.testRectangles(a);
				test.testRectangles2(a+30);
				vTaskDelay(3);
				disp.clearScreen();
			}
		}
	}
	ESP_LOGD(tag, "<< exit point ssd1331_test");
	vTaskDelete(NULL);
}
