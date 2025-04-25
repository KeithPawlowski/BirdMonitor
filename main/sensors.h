#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include <RTClib.h> 

void sensorsInit();
void sensorTask(void *pvParameters);
float scaleLight(float lux);

extern SemaphoreHandle_t sensorSemaphore;
extern volatile bool recording;
extern volatile bool cooldown;
extern const char* dummyDateTime;
extern RTC_DS3231 rtc; 

#endif