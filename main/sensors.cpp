#include "sensors.h"
#include <Adafruit_SHT31.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_AHTX0.h>
#include "sd_read_write.h"

// Sensor objects for temperature, humidity, and light sensors
Adafruit_SHT31 sht31 = Adafruit_SHT31(); // SHT31 for indoor temperature
Adafruit_AHTX0 aht10; // AHT10 for outdoor temperature and humidity
Adafruit_VEML7700 veml7700 = Adafruit_VEML7700(); // VEML7700 for indoor light
Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591); // TSL2591 for outdoor light
int debug2 = 0;

// Initialize all sensors
void sensorsInit() {
    // Initialize SHT31 at I2C address 0x44; halt if initialization fails
    if (!sht31.begin(0x44)) {
        if (debug2){
            Serial.println("Couldn't find SHT31 (temp sensor)");
        }
        while (1) delay(10);
    }
    // Initialize AHT10 on I2C bus; halt if initialization fails
    if (!aht10.begin(&Wire)) {
        if (debug2) {
            Serial.println("Couldn't find AHT10 (temp + humidity)");

        }       
        while (1) delay(10);
    }
    // Initialize VEML7700 on I2C bus; halt if initialization fails
    if (!veml7700.begin(&Wire)) {
        if (debug2) {
            Serial.println("Couldn't find VEML7700 (light sensor)");
        }   
             while (1) delay(10);
    }
    // Initialize TSL2591 on I2C bus; halt if initialization fails
    if (!tsl2591.begin(&Wire)) {
        if (debug2) {
            Serial.println("Couldn't find TSL2591 (light sensor)");

        }   
        while (1) delay(10);
    }
    // Set TSL2591 gain to medium for balanced sensitivity
    tsl2591.setGain(TSL2591_GAIN_MED);
    // Set TSL2591 integration time to 300ms for light measurement
    tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS);
}

// Scale light sensor lux value to a range suitable for logging
float scaleLight(float lux) {
    // Constrain lux value between 0 and 1000 to prevent extreme values
    float normalized = constrain(lux, 0, 1000);
    // Scale normalized lux to a value between 6 and 24 (inverted scale)
    float scaled = 24 - (normalized / 1000 * 18);
    return scaled;
}

// Log sensor data to a specified file with a timestamp
void logSensorData(const char* filename, const char* data) {
    // Construct file path for logging in /LogFiles directory
    char filepath[40];
    sprintf(filepath, "/LogFiles/%s", filename);

    // Get current time from RTC
    DateTime now = rtc.now();
    // Format timestamp as YYYY-MM-DD-HH:MM:SS
    char timeStr[20];
    sprintf(timeStr, "%04d-%02d-%02d-%02d:%02d:%02d", 
            now.year(), now.month(), now.day(), 
            now.hour(), now.minute(), now.second());

    // Combine timestamp and data into a single line
    char line[100];
    sprintf(line, "%s - %s\n", timeStr, data);
    // Append the line to the specified file on SD card
    appendFile(SD_MMC, filepath, line);
}

// Sensor task to periodically read and log sensor data
void sensorTask(void *pvParameters) {
    while (1) {
        // Check if not recording, not in cooldown, and acquire sensor semaphore
        if (!recording && !cooldown && xSemaphoreTake(sensorSemaphore, portMAX_DELAY) == pdTRUE) {
            // Read indoor temperature from SHT31
            float sht_temp = sht31.readTemperature();
            char sht_data[20];
            // Format temperature data
            sprintf(sht_data, "Temp: %.2f C", sht_temp);
            // Log to instmp.log
            logSensorData("instmp.log", sht_data);

            // Read outdoor temperature and humidity from AHT10
            sensors_event_t humidity, temp;
            aht10.getEvent(&humidity, &temp);
            char aht_data[20];
            // Format temperature data
            sprintf(aht_data, "Temp: %.2f C", temp.temperature);
            // Log to outtmp.log
            logSensorData("outtmp.log", aht_data);

            // Read indoor light level from VEML7700
            float veml_lux = veml7700.readLux();
            // Scale light value
            float in_light_scaled = scaleLight(veml_lux);
            char veml_data[20];
            // Format light data
            sprintf(veml_data, "Light: %.2f", in_light_scaled);
            // Log to inlight.log
            logSensorData("inlight.log", veml_data);

            // Read outdoor light level from TSL2591
            uint32_t lum = tsl2591.getFullLuminosity();
            uint16_t ir = lum >> 16; // Extract IR component
            uint16_t full = lum & 0xFFFF; // Extract full spectrum
            // Calculate lux value
            float tsl_lux = tsl2591.calculateLux(full, ir);
            // Scale light value
            float out_light_scaled = scaleLight(tsl_lux);
            char tsl_data[20];
            // Format light data
            sprintf(tsl_data, "Light: %.2f", out_light_scaled);
            // Log to outlight.log
            logSensorData("outlight.log", tsl_data);

            // Release sensor semaphore
            xSemaphoreGive(sensorSemaphore);
        }
        if (debug2) {
            Serial.println("sensorTask(): Environmental data logged successfully");
        }
        // Delay for 10 minutes (600,000ms) before next sensor reading
        vTaskDelay(600000 / portTICK_PERIOD_MS);
    }
}