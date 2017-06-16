This is ssd1331 driver for esp32 esp-idf. Basically its 2 different drivers. One is rewrited adafruit ssd1331 arduino driver and the other is based on ili9341 driver writen by SpriteTM here https://github.com/espressif/esp32-nesemu/tree/master/components/nofrendo-esp32. Both drivers are compatible and are using Adafruit_GFX library.

SSD1331 hardware accelerated functions are commented because they require few ticks delay to finish and this makes performance drop. Software functions are faster and works well on esp32.

Option to choose between drivers is in benchmark.h
    '#define USE_ADAFRUIT_LIB'
    
Pins can be assigned from make menuconfig.