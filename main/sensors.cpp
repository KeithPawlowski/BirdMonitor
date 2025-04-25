#include "sensors.h"
#include <Adafruit_SHT31.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_AHTX0.h>
#include "sd_read_write.h"

// Sensor objects
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_AHTX0 aht10;
Adafruit_VEML7700 veml7700 = Adafruit_VEML7700();
Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591);

void sensorsInit() {
    if (!sht31.begin(0x44)) {
        Serial.println("Couldn't find SHT31");
        while (1) delay(10);
    }
    if (!aht10.begin(&Wire)) {
        Serial.println("Couldn't find AHT10");
        while (1) delay(10);
    }
    if (!veml7700.begin(&Wire)) {
        Serial.println("Couldn't find VEML7700");
        while (1) delay(10);
    }
    if (!tsl2591.begin(&Wire)) {
        Serial.println("Couldn't find TSL2591");
        while (1) delay(10);
    }
    tsl2591.setGain(TSL2591_GAIN_MED);
    tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS);
}

float scaleLight(float lux) {
    float normalized = constrain(lux, 0, 1000);
    float scaled = 24 - (normalized / 1000 * 18);
    return scaled;
}

void logSensorData(const char* filename, const char* data) {
    char filepath[40];
    sprintf(filepath, "/LogFiles/%s", filename);

    // Get current time from RTC
    DateTime now = rtc.now();
    char timeStr[20];
    sprintf(timeStr, "%04d-%02d-%02d-%02d:%02d:%02d", 
            now.year(), now.month(), now.day(), 
            now.hour(), now.minute(), now.second());

    // Combine timestamp and data
    char line[100];
    sprintf(line, "%s - %s\n", timeStr, data);
    appendFile(SD_MMC, filepath, line);
}

void sensorTask(void *pvParameters) {
    while (1) {
        if (!recording && !cooldown && xSemaphoreTake(sensorSemaphore, portMAX_DELAY) == pdTRUE) {
            // Inside temperature (SHT31)
            float sht_temp = sht31.readTemperature();
            char sht_data[20];
            sprintf(sht_data, "Temp: %.2f C", sht_temp);
            logSensorData("instmp.log", sht_data);

            // Outside temperature (AHT10)
            sensors_event_t humidity, temp;
            aht10.getEvent(&humidity, &temp);
            char aht_data[20];
            sprintf(aht_data, "Temp: %.2f C", temp.temperature);
            logSensorData("outtmp.log", aht_data);

            // Inside light (VEML7700)
            float veml_lux = veml7700.readLux();
            float in_light_scaled = scaleLight(veml_lux);
            char veml_data[20];
            sprintf(veml_data, "Light: %.2f", in_light_scaled);
            logSensorData("inlight.log", veml_data);

            // Outside light (TSL2591)
            uint32_t lum = tsl2591.getFullLuminosity();
            uint16_t ir = lum >> 16;
            uint16_t full = lum & 0xFFFF;
            float tsl_lux = tsl2591.calculateLux(full, ir);
            float out_light_scaled = scaleLight(tsl_lux);
            char tsl_data[20];
            sprintf(tsl_data, "Light: %.2f", out_light_scaled);
            logSensorData("outlight.log", tsl_data);

            xSemaphoreGive(sensorSemaphore);
        }
        vTaskDelay(6000 / portTICK_PERIOD_MS);
    }
}