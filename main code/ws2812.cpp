#include "ws2812.h"

// Initialize WS2812 LED strip with 1 LED, connected to WS2812_PIN, using GRB color order
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(1, WS2812_PIN, 1, TYPE_GRB);

// Initialize the WS2812 LED strip
void ws2812Init(void)
{
  // Start the LED strip communication
  strip.begin();
  // Set brightness to 0 to ensure the LED is off initially
  strip.setBrightness(0);
  // Call ws2812SetColor to explicitly turn off the LED
  ws2812SetColor(0);
}

// Set the color of the WS2812 LED
// Parameter 'color' is not used in this implementation, LED is always set to off
void ws2812SetColor(int color)
{
  // Set the LED color data for the first (and only) LED to (0, 0, 0), effectively turning it off
  strip.setLedColorData(0, 0, 0, 0);
  // Send the color data to the LED strip to update its state
  strip.show();
}