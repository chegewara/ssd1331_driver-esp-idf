/*
 * benchmark.h
 *
 *  Created on: 11.06.2017
 *      Author: darek
 */

#ifndef _SSD1331_BENCHMARK_
#define _SSD1331_BENCHMARK_

//#define USE_ADAFRUIT_LIB

#if defined USE_ADAFRUIT_LIB
	#include "Adafruit_SSD1331.h"
#else
	#include "SSD1331.h"
#endif

class Benchmark {

public:
#if defined USE_ADAFRUIT_LIB
	Benchmark(Adafruit_SSD1331 *disp);
#else
	Benchmark(SSD1331 *disp);
#endif

	void testLines();
	void testFastLines();
	void drawClock();
	void testText();
	void testRectangles(int alpha);
	void testRectangles2(int alpha);
	void testBox3d();
private:
#if defined USE_ADAFRUIT_LIB
	Adafruit_SSD1331 *display;
#else
	SSD1331 *display;
#endif

	uint16_t x0, y0;
	void println(char*);
};

#endif
