#ifndef __WS2812_H
#define __WS2812_H

#include "Freenove_WS2812_Lib_for_ESP32.h"

// WS2812 LED pin
#define WS2812_PIN  48

// Initialize WS2812 LED
void ws2812Init(void);
// Set WS2812 LED color
void ws2812SetColor(int color);

#endif