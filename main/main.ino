#include <RTClib.h>
#include "esp_camera.h"
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"
#include "ws2812.h"
#include "sd_read_write.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp_heap_caps.h"
#include <Wire.h>
#include "sensors.h"
#include <driver/i2s.h>

// I2C pins
#define I2C_SDA 21
#define I2C_SCL 47

// I2S pins
#define I2S_WS  45
#define I2S_SD  42
#define I2S_SCK 41
#define I2S_PORT I2S_NUM_0

// Audio config
#define SAMPLE_RATE   32000U
#define SAMPLE_BITS   16
#define RECORD_TIME   20
#define AUDIO_BUFFER_SIZE (2 * 1024 * 1024) // 2MB
#define VOLUME_GAIN   4
#define WAV_HEADER_SIZE 44

// Buffer config
#define NUM_BUFFERS 15 // system really only uses 10 
#define FRAMES_PER_BUFFER 20 //20 frames to each buffer
#define MAX_FRAME_SIZE 20000 // 25KB max 
//12 buffers * 20 Frames per buffer * 25KB = 6MB 
#define PIR_PIN 14

uint8_t *buffers[NUM_BUFFERS][FRAMES_PER_BUFFER];
size_t lengths[NUM_BUFFERS][FRAMES_PER_BUFFER];
volatile int frame_count = 0;
int total_frame_index = 0;
volatile int activeBuffer = 0;

SemaphoreHandle_t bufferSemaphore;
SemaphoreHandle_t sensorSemaphore;
SemaphoreHandle_t audioSemaphore;
volatile bool bufferReady[NUM_BUFFERS] = {false};

volatile bool recording = false;
volatile bool cooldown = false;
volatile bool simulatePIR = false;
volatile int captureFolderNum = 0;

RTC_DS3231 rtc;
uint8_t* audioBuffer = NULL;
volatile size_t audioBytesWritten = 0;
volatile bool audioRecording = false;

// New function to find the next available capture folder index
int findNextCaptureIndex(fs::FS &fs, const char *dirname) {
    File root = fs.open(dirname);
    if (!root) {
        Serial.println("Failed to open /input directory");
        return 0; // Default to 0 if directory can't be opened
    }
    if (!root.isDirectory()) {
        Serial.println("/input is not a directory");
        return 0;
    }

    int highestIndex = -1;
    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            const char *name = file.name();
            // Check if the directory name matches "captureXXXX" pattern
            if (strncmp(name, "capture", 7) == 0 && strlen(name) == 11) {
                int index = atoi(name + 7); // Extract number after "capture"
                if (index >= 0 && index <= 9999) { // Ensure it's a valid 4-digit number
                    if (index > highestIndex) {
                        highestIndex = index;
                    }
                }
            }
        }
        file = root.openNextFile();
    }
    root.close();

    // Return the next index (highest found + 1), or 0 if no capture folders exist
    int nextIndex = (highestIndex >= 0) ? highestIndex + 1 : 0;
    Serial.printf("Highest capture index found: %d, starting at: %d\n", highestIndex, nextIndex);
    return nextIndex;
}

void writeBatchToSD(int bufferNum, int count, const char* folderName) {
    Serial.printf("Core %d: Writing %d frames from buffer %d at index %d to %s\n", 
                  xPortGetCoreID(), count, bufferNum, total_frame_index, folderName);
    for (int i = 0; i < count; i++) {
        char filename[40];
        sprintf(filename, "%s/%03d.jpg", folderName, total_frame_index + i);
        writejpg(SD_MMC, filename, buffers[bufferNum][i], lengths[bufferNum][i]);
    }
    total_frame_index += count;
}

void sdWriteTask(void *pvParameters) {
    while (1) {
        for (int b = 0; b < NUM_BUFFERS; b++) {
            if (bufferReady[b] && xSemaphoreTake(bufferSemaphore, portMAX_DELAY) == pdTRUE) { 
                char folderName[30];
                sprintf(folderName, "/input/capture%04d", captureFolderNum);
                writeBatchToSD(b, FRAMES_PER_BUFFER, folderName);
                bufferReady[b] = false;
                xSemaphoreGive(bufferSemaphore);
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void i2s_install() {
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = i2s_bits_per_sample_t(SAMPLE_BITS),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = false
    };
    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) Serial.printf("I2S install failed: %d\n", err);
    else Serial.println("I2S installed successfully");
}

void i2s_setpin() {
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };
    esp_err_t err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) Serial.printf("I2S set pin failed: %d\n", err);
    else Serial.println("I2S pins set successfully");
}

void audioTask(void *pvParameters) {
    Serial.println("Audio task started");
    while (1) {
        if (audioRecording) {
            size_t bytesRead;
            esp_err_t result = i2s_read(I2S_PORT, audioBuffer + audioBytesWritten, 
                                      16384, &bytesRead, 50 / portTICK_PERIOD_MS); // 16KB chunks
            if (result == ESP_OK && bytesRead > 0) {
                audioBytesWritten += bytesRead;
                if (audioBytesWritten >= AUDIO_BUFFER_SIZE) {
                    Serial.println("Audio buffer full, stopping");
                    audioRecording = false;
                }
            } else if (result != ESP_OK) {
                Serial.printf("Audio read error: %d\n", result);
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS); // Fast polling
    }
}

void stopAudioRecording(const char* folderName) {
    audioRecording = false;
    unsigned long audio_stop_start = micros(); // Timing start for audio stop
    if (xSemaphoreTake(audioSemaphore, 2000 / portTICK_PERIOD_MS) == pdTRUE) { // 2s timeout
        i2s_stop(I2S_PORT);
        Serial.printf("Audio recording stopped, bytes written: %d\n", audioBytesWritten);
        if (audioBytesWritten > 0) {
            for (size_t i = 0; i < audioBytesWritten; i += SAMPLE_BITS / 8) {
                (*(uint16_t *)(audioBuffer + i)) <<= VOLUME_GAIN;
            }
            uint8_t wav_header[WAV_HEADER_SIZE];
            generate_wav_header(wav_header, audioBytesWritten, SAMPLE_RATE);
            char audioPath[40];
            sprintf(audioPath, "%s/audio.wav", folderName);
            File file = SD_MMC.open(audioPath, FILE_WRITE);
            if (file) {
                file.write(wav_header, WAV_HEADER_SIZE);
                file.write(audioBuffer, audioBytesWritten);
                file.close();
                Serial.printf("Wrote %d bytes of audio to %s\n", audioBytesWritten + WAV_HEADER_SIZE, audioPath);
            } else {
                Serial.println("Failed to write audio file");
            }
        } else {
            Serial.println("No audio data to write");
        }
        xSemaphoreGive(audioSemaphore);
    } else {
        Serial.println("Failed to stop audio recording cleanly");
    }
    unsigned long audio_stop_end = micros(); // Timing end for audio stop
    Serial.printf("Audio stop duration: %lu ms\n", (audio_stop_end - audio_stop_start) / 1000);
}

void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate) {
    uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
    uint32_t byte_rate = sample_rate * SAMPLE_BITS / 8;
    const uint8_t set_wav_header[] = {
        'R', 'I', 'F', 'F', file_size, file_size >> 8, file_size >> 16, file_size >> 24,
        'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 0x10, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x01, 0x00, sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24,
        byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, 0x02, 0x00, 0x10, 0x00,
        'd', 'a', 't', 'a', wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24,
    };
    memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}

void pirTask(void *pvParameters) {
    while (1) {
        if (!recording && !cooldown && digitalRead(PIR_PIN) == LOW) {
            Serial.println("Motion detected (beam broken), starting recording...");
            recording = true;

            char folderName[30];
            sprintf(folderName, "/input/capture%04d", captureFolderNum);
            createDir(SD_MMC, folderName);

            char timestampPath[40];
            sprintf(timestampPath, "%s/timestamp.txt", folderName);
            DateTime now = rtc.now();
            char timeStr[20];
            sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
                    now.year(), now.month(), now.day(), 
                    now.hour(), now.minute(), now.second());
            writeFile(SD_MMC, timestampPath, timeStr);

            total_frame_index = 0;
            audioBytesWritten = 0;
            const int frame_rate = 20; // *20*
            const int video_duration = 20; 
            const int total_photos = frame_rate * video_duration; // 400
            const unsigned long frame_interval = 1000000 / frame_rate;

            Serial.println("Starting I2S for audio");
            unsigned long audio_start = micros(); // Timing start for audio setup
            i2s_start(I2S_PORT);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (xSemaphoreTake(audioSemaphore, portMAX_DELAY) == pdTRUE) {
                audioRecording = true;
                xSemaphoreGive(audioSemaphore);
            }
            size_t bytesRead;
            esp_err_t result = i2s_read(I2S_PORT, audioBuffer, 1024, &bytesRead, 100 / portTICK_PERIOD_MS);
            unsigned long audio_setup_end = micros(); // Timing end for audio setup
            Serial.printf("Audio setup duration: %lu ms\n", (audio_setup_end - audio_start) / 1000);
            if (result == ESP_OK) {
                audioBytesWritten = bytesRead;
                Serial.printf("Initial audio read: %d bytes\n", bytesRead);
            } else {
                Serial.printf("Initial audio read failed: %d\n", result);
            }

            unsigned long capture_start_time = micros();
            for (int i = 0; i < total_photos; i++) {
                unsigned long frame_start = micros();
                ws2812SetColor(3);

                camera_fb_t *fb = esp_camera_fb_get();
                if (fb != NULL) {
                    if (fb->len <= MAX_FRAME_SIZE) {
                        if (!bufferReady[activeBuffer]) {
                            memcpy(buffers[activeBuffer][frame_count], fb->buf, fb->len);
                            lengths[activeBuffer][frame_count] = fb->len;
                            frame_count++;
                            if (frame_count >= FRAMES_PER_BUFFER) {
                                bufferReady[activeBuffer] = true;
                                activeBuffer = (activeBuffer + 1) % NUM_BUFFERS;
                                frame_count = 0;
                            }
                        } else {
                            Serial.println("Buffer busy, frame skipped");
                        }
                    } else {
                        Serial.printf("Frame %d too large: %d bytes\n", i, fb->len);
                    }
                    esp_camera_fb_return(fb);
                }

                ws2812SetColor(2);
                unsigned long elapsed = micros() - capture_start_time;
                if (elapsed >= video_duration * 1000000) break;
                while (micros() - frame_start < frame_interval) {
                    delayMicroseconds(100);
                }
            }

            if (frame_count > 0 && !bufferReady[activeBuffer]) {
                bufferReady[activeBuffer] = true;
            }

            unsigned long capture_end_time = micros();
            Serial.printf("Core %d: Capture duration: %lu ms\n", xPortGetCoreID(), 
                          (capture_end_time - capture_start_time) / 1000);
            stopAudioRecording(folderName);

            recording = false;
            cooldown = true;
            Serial.println("Recording complete, entering 1-minute cooldown...");
            vTaskDelay(60000 / portTICK_PERIOD_MS);
            cooldown = false;
            captureFolderNum++;
            Serial.printf("Cooldown complete, next recording will use %s\n", 
                          "/input/capture%04d", captureFolderNum);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(false);

    ws2812Init();
    sdmmcInit();

    // Scan /input for existing capture folders and set initial captureFolderNum
    createDir(SD_MMC, "/input"); // Ensure /input exists
    captureFolderNum = findNextCaptureIndex(SD_MMC, "/input");

    createDir(SD_MMC, "/LogFiles");
    pinMode(PIR_PIN, INPUT_PULLUP);

    Wire.begin(I2C_SDA, I2C_SCL);
    sensorsInit();

    if (!rtc.begin(&Wire)) {
        Serial.println("Couldn't find DS3231 RTC");
        ws2812SetColor(1);
        while (1) delay(10);
    }
    Serial.println("DS3231 RTC initialized successfully");

    if (psramFound()) {
        size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
        Serial.printf("Detected PSRAM size: %u bytes (~%u MB)\n", psram_size, psram_size / (1024 * 1024));
    } else {
        Serial.println("No PSRAM detected!");
    }

    if (cameraSetup() == 1) {
        ws2812SetColor(2);
    } else {
        ws2812SetColor(1);
        return;
    }

    for (int b = 0; b < NUM_BUFFERS; b++) {
        for (int i = 0; i < FRAMES_PER_BUFFER; i++) {
            buffers[b][i] = (uint8_t *)ps_malloc(MAX_FRAME_SIZE);
            if (buffers[b][i] == NULL) {
                Serial.println("Video PSRAM allocation failed!");
                ws2812SetColor(1);
                while (1);
            }
        }
    }

    audioBuffer = (uint8_t *)ps_malloc(AUDIO_BUFFER_SIZE);
    if (audioBuffer == NULL) {
        Serial.println("Audio PSRAM allocation failed!");
        ws2812SetColor(1);
        while (1);
    }

    i2s_install();
    i2s_setpin();

    File testFile = SD_MMC.open("/test.txt", FILE_WRITE);
    if (testFile) {
        testFile.println("SD card test");
        testFile.close();
        Serial.println("SD test OK");
    } else {
        Serial.println("SD test failed");
    }

    bufferSemaphore = xSemaphoreCreateMutex();
    sensorSemaphore = xSemaphoreCreateMutex();
    audioSemaphore = xSemaphoreCreateMutex();
    if (bufferSemaphore == NULL || sensorSemaphore == NULL || audioSemaphore == NULL) {
        Serial.println("Semaphore creation failed!");
        while (1);
    }

    // Create the initial capture folder with the determined index
    char initialFolder[30];
    sprintf(initialFolder, "/input/capture%04d", captureFolderNum);
    createDir(SD_MMC, initialFolder);

    xTaskCreatePinnedToCore(sdWriteTask, "SDWriteTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(pirTask, "PIRTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(audioTask, "AudioTask", 4096, NULL, 2, NULL, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}

int cameraSetup(void) {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_VGA; // (480 x 640) *VGA*
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12; //scale of 0-63, higher value --> more compression (lower quality) *15*
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x", err);
        return 0;
    }

    struct _sensor *cam_sensor = esp_camera_sensor_get();
    cam_sensor->set_vflip(cam_sensor, 1);
    cam_sensor->set_brightness(cam_sensor, 1);
    cam_sensor->set_saturation(cam_sensor, 0);

    return 1;
}