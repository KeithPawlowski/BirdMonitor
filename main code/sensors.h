#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include <RTClib.h> 

// Initialize sensors
void sensorsInit();
// Sensor task for periodic data logging
void sensorTask(void *pvParameters);
// Scale light sensor lux value
float scaleLight(float lux);

// External variables
extern SemaphoreHandle_t sensorSemaphore; // Semaphore for sensor access
extern volatile bool recording; // Recording state
extern volatile bool cooldown; // Cooldown state
extern const char* dummyDateTime; // Unused dummy datetime
extern RTC_DS3231 rtc; // Real-time clock

#endif