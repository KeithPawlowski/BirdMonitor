#include <RTClib.h>
#include "esp_camera.h"
#define CAMERA_MODEL_ESP32S3_EYE // Define the camera model
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

// I2C pins for sensor communication
#define I2C_SDA 21
#define I2C_SCL 47

// I2S pins for audio recording
#define I2S_WS  45 // Word Select
#define I2S_SD  42 // Serial Data
#define I2S_SCK 41 // Serial Clock
#define I2S_PORT I2S_NUM_0 // I2S port number

// Audio configuration
#define SAMPLE_RATE   32000U // Audio sample rate (32kHz)
#define SAMPLE_BITS   16 // Bits per sample
#define RECORD_TIME   20 // Recording duration in seconds
#define AUDIO_BUFFER_SIZE (2 * 1024 * 1024) // 2MB audio buffer
#define VOLUME_GAIN   4 // Audio amplification factor
#define WAV_HEADER_SIZE 44 // Size of WAV file header

// Buffer configuration for camera frames
#define NUM_BUFFERS 15 // Total buffers (though only 10 used)
#define FRAMES_PER_BUFFER 20 // Frames per buffer
#define MAX_FRAME_SIZE 20000 // Maximum frame size (25KB)
#define PIR_PIN 14 // PIR sensor pin

// Arrays to store frame buffers and their lengths
uint8_t *buffers[NUM_BUFFERS][FRAMES_PER_BUFFER];
size_t lengths[NUM_BUFFERS][FRAMES_PER_BUFFER];
// Track current frame count and total frame index
volatile int frame_count = 0;
int total_frame_index = 0;
// Track active buffer index
volatile int activeBuffer = 0;

// Semaphores for thread-safe access
SemaphoreHandle_t bufferSemaphore;
SemaphoreHandle_t sensorSemaphore;
SemaphoreHandle_t audioSemaphore;
// Flags to indicate buffer readiness
volatile bool bufferReady[NUM_BUFFERS] = {false};

// Flags for system state
volatile bool recording = false; // Is recording active?
volatile bool cooldown = false; // Is system in cooldown?
volatile bool simulatePIR = false; // Simulate PIR trigger
volatile int captureFolderNum = 0; // Current capture folder index

// Real-time clock object
RTC_DS3231 rtc;
// Audio buffer for recording
uint8_t* audioBuffer = NULL;
// Track bytes written to audio buffer
volatile size_t audioBytesWritten = 0;
// Flag for audio recording state
volatile bool audioRecording = false;

// Find the next available capture folder index
int findNextCaptureIndex(fs::FS &fs, const char *dirname) {
    // Open the directory
    File root = fs.open(dirname);
    // Check if directory is valid
    if (!root || !root.isDirectory()) {
        return 0; // Return 0 if directory can't be opened
    }

    int highestIndex = -1;
    // Iterate through directory entries
    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            const char *name = file.name();
            // Check for directories named "captureXXXX"
            if (strncmp(name, "capture", 7) == 0 && strlen(name) == 11) {
                int index = atoi(name + 7);
                if (index >= 0 && index <= 9999) {
                    if (index > highestIndex) {
                        highestIndex = index;
                    }
                }
            }
        }
        file = root.openNextFile();
    }
    root.close();

    // Return next available index
    int nextIndex = (highestIndex >= 0) ? highestIndex + 1 : 0;
    return nextIndex;
}

// Write a batch of frames to SD card
void writeBatchToSD(int bufferNum, int count, const char* folderName) {
    // Write each frame as a JPEG file
    for (int i = 0; i < count; i++) {
        char filename[40];
        // Construct filename as folderName/XXX.jpg
        sprintf(filename, "%s/%03d.jpg", folderName, total_frame_index + i);
        // Write JPEG data to SD card
        writejpg(SD_MMC, filename, buffers[bufferNum][i], lengths[bufferNum][i]);
    }
    // Update total frame index
    total_frame_index += count;
}

// Task to handle writing buffers to SD card
void sdWriteTask(void *pvParameters) {
    while (1) {
        // Check each buffer
        for (int b = 0; b < NUM_BUFFERS; b++) {
            // If buffer is ready and semaphore is acquired
            if (bufferReady[b] && xSemaphoreTake(bufferSemaphore, portMAX_DELAY) == pdTRUE) { 
                char folderName[30];
                // Construct folder name as /input/captureXXXX
                sprintf(folderName, "/input/capture%04d", captureFolderNum);
                // Write buffer frames to SD
                writeBatchToSD(b, FRAMES_PER_BUFFER, folderName);
                // Mark buffer as not ready
                bufferReady[b] = false;
                // Release semaphore
                xSemaphoreGive(bufferSemaphore);
            }
        }
        // Short delay to prevent CPU hogging
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// Configure I2S interface for audio recording
void i2s_install() {
    // I2S configuration structure
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Master mode, receive
        .sample_rate = SAMPLE_RATE, // 32kHz
        .bits_per_sample = i2s_bits_per_sample_t(SAMPLE_BITS), // 16 bits
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Mono audio
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S), // I2S standard
        .intr_alloc_flags = 0, // No interrupt flags
        .dma_buf_count = 8, // 8 DMA buffers
        .dma_buf_len = 1024, // 1024 samples per buffer
        .use_apll = false // No APLL clock
    };
    // Install I2S driver
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

// Set I2S pin configuration
void i2s_setpin() {
    // I2S pin configuration structure
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK, // Serial clock pin
        .ws_io_num = I2S_WS, // Word select pin
        .data_out_num = -1, // No output
        .data_in_num = I2S_SD // Serial data input
    };
    // Assign pins to I2S port
    i2s_set_pin(I2S_PORT, &pin_config);
}

// Task to handle audio recording
void audioTask(void *pvParameters) {
    while (1) {
        // If audio recording is active
        if (audioRecording) {
            size_t bytesRead;
            // Read audio data into buffer
            esp_err_t result = i2s_read(I2S_PORT, audioBuffer + audioBytesWritten, 
                                      16384, &bytesRead, 50 / portTICK_PERIOD_MS);
            if (result == ESP_OK && bytesRead > 0) {
                // Update bytes written
                audioBytesWritten += bytesRead;
                // Stop if buffer is full
                if (audioBytesWritten >= AUDIO_BUFFER_SIZE) {
                    audioRecording = false;
                }
            }
        }
        // Short delay to prevent CPU hogging
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// Stop audio recording and save to WAV file
void stopAudioRecording(const char* folderName) {
    // Stop audio recording
    audioRecording = false;
    // Acquire audio semaphore with timeout
    if (xSemaphoreTake(audioSemaphore, 2000 / portTICK_PERIOD_MS) == pdTRUE) {
        // Stop I2S port
        i2s_stop(I2S_PORT);
        if (audioBytesWritten > 0) {
            // Apply volume gain to audio data
            for (size_t i = 0; i < audioBytesWritten; i += SAMPLE_BITS / 8) {
                (*(uint16_t *)(audioBuffer + i)) <<= VOLUME_GAIN;
            }
            // Generate WAV header
            uint8_t wav_header[WAV_HEADER_SIZE];
            generate_wav_header(wav_header, audioBytesWritten, SAMPLE_RATE);
            // Construct audio file path
            char audioPath[40];
            sprintf(audioPath, "%s/audio.wav", folderName);
            // Write WAV file to SD card
            File file = SD_MMC.open(audioPath, FILE_WRITE);
            if (file) {
                file.write(wav_header, WAV_HEADER_SIZE);
                file.write(audioBuffer, audioBytesWritten);
                file.close();
            }
        }
        // Release audio semaphore
        xSemaphoreGive(audioSemaphore);
    }
}

// Generate WAV file header
void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate) {
    // Calculate file size and byte rate
    uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
    uint32_t byte_rate = sample_rate * SAMPLE_BITS / 8;
    // WAV header template
    const uint8_t set_wav_header[] = {
        'R', 'I', 'F', 'F', file_size, file_size >> 8, file_size >> 16, file_size >> 24,
        'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 0x10, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x01, 0x00, sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24,
        byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, 0x02, 0x00, 0x10, 0x00,
        'd', 'a', 't', 'a', wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24,
    };
    // Copy template to output buffer
    memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}

// Task to handle PIR sensor and recording
void pirTask(void *pvParameters) {
    while (1) {
        // Check if not recording, not in cooldown, and PIR sensor is triggered (LOW)
        if (!recording && !cooldown && digitalRead(PIR_PIN) == LOW) {
            // Start recording
            recording = true;

            // Create new capture folder
            char folderName[30];
            sprintf(folderName, "/input/capture%04d", captureFolderNum);
            createDir(SD_MMC, folderName);

            // Write timestamp to file
            char timestampPath[40];
            sprintf(timestampPath, "%s/timestamp.txt", folderName);
            DateTime now = rtc.now();
            char timeStr[20];
            sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
                    now.year(), now.month(), now.day(), 
                    now.hour(), now.minute(), now.second());
            writeFile(SD_MMC, timestampPath, timeStr);

            // Reset frame counters
            total_frame_index = 0;
            audioBytesWritten = 0;
            const int frame_rate = 20; // 20 FPS
            const int video_duration = 20; // 20 seconds
            const int total_photos = frame_rate * video_duration; // Total frames
            const unsigned long frame_interval = 1000000 / frame_rate; // Frame interval in microseconds

            // Start audio recording
            i2s_start(I2S_PORT);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (xSemaphoreTake(audioSemaphore, portMAX_DELAY) == pdTRUE) {
                audioRecording = true;
                xSemaphoreGive(audioSemaphore);
            }
            size_t bytesRead;
            // Initial audio read to start buffer
            esp_err_t result = i2s_read(I2S_PORT, audioBuffer, 1024, &bytesRead, 100 / portTICK_PERIOD_MS);
            if (result == ESP_OK) {
                audioBytesWritten = bytesRead;
            }

            // Capture video frames
            unsigned long capture_start_time = micros();
            for (int i = 0; i < total_photos; i++) {
                unsigned long frame_start = micros();
                // Get camera frame
                camera_fb_t *fb = esp_camera_fb_get();
                if (fb != NULL) {
                    // Check if frame size is within limit
                    if (fb->len <= MAX_FRAME_SIZE) {
                        if (!bufferReady[activeBuffer]) {
                            // Copy frame to buffer
                            memcpy(buffers[activeBuffer][frame_count], fb->buf, fb->len);
                            lengths[activeBuffer][frame_count] = fb->len;
                            frame_count++;
                            // If buffer is full, mark it ready
                            if (frame_count >= FRAMES_PER_BUFFER) {
                                bufferReady[activeBuffer] = true;
                                activeBuffer = (activeBuffer + 1) % NUM_BUFFERS;
                                frame_count = 0;
                            }
                        }
                    }
                    // Return frame buffer to camera
                    esp_camera_fb_return(fb);
                }

                // Stop if duration exceeded
                unsigned long elapsed = micros() - capture_start_time;
                if (elapsed >= video_duration * 1000000) break;
                // Maintain frame rate
                while (micros() - frame_start < frame_interval) {
                    delayMicroseconds(100);
                }
            }

            // Mark final buffer as ready if it contains frames
            if (frame_count > 0 && !bufferReady[activeBuffer]) {
                bufferReady[activeBuffer] = true;
            }

            // Stop and save audio recording
            stopAudioRecording(folderName);

            // End recording
            recording = false;
            // Enter cooldown for 2 minutes
            cooldown = true;
            vTaskDelay(120000 / portTICK_PERIOD_MS);
            cooldown = false;
            // Increment capture folder index
            captureFolderNum++;
        }
        // Check PIR sensor every 100ms
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Arduino setup function
void setup() {
    // Initialize WS2812 LED
    ws2812Init();
    // Initialize SD card
    sdmmcInit();

    // Create input directory
    createDir(SD_MMC, "/input");
    // Find next capture folder index
    captureFolderNum = findNextCaptureIndex(SD_MMC, "/input");

    // Create logging directory
    createDir(SD_MMC, "/LogFiles");
    // Configure PIR sensor pin with pull-up
    pinMode(PIR_PIN, INPUT_PULLUP);

    // Initialize I2C bus
    Wire.begin(I2C_SDA, I2C_SCL);
    // Initialize sensors
    sensorsInit();

    // Initialize RTC; halt if failed
    if (!rtc.begin(&Wire)) {
        while (1) delay(10);
    }

    // Check for PSRAM (no action taken)
    if (psramFound()) {
    } else {
    }

    // Initialize camera; halt if failed
    if (cameraSetup() != 1) {
        while (1) delay(10);
    }

    // Allocate frame buffers in PSRAM
    for (int b = 0; b < NUM_BUFFERS; b++) {
        for (int i = 0; i < FRAMES_PER_BUFFER; i++) {
            buffers[b][i] = (uint8_t *)ps_malloc(MAX_FRAME_SIZE);
            if (buffers[b][i] == NULL) {
                while (1); // Halt on allocation failure
            }
        }
    }

    // Allocate audio buffer in PSRAM
    audioBuffer = (uint8_t *)ps_malloc(AUDIO_BUFFER_SIZE);
    if (audioBuffer == NULL) {
        while (1); // Halt on allocation failure
    }

    // Configure I2S for audio
    i2s_install();
    i2s_setpin();

    // Test SD card write
    File testFile = SD_MMC.open("/test.txt", FILE_WRITE);
    if (testFile) {
        testFile.println("SD card test");
        testFile.close();
    }

    // Create semaphores
    bufferSemaphore = xSemaphoreCreateMutex();
    sensorSemaphore = xSemaphoreCreateMutex();
    audioSemaphore = xSemaphoreCreateMutex();
    if (bufferSemaphore == NULL || sensorSemaphore == NULL || audioSemaphore == NULL) {
        while (1); // Halt on semaphore creation failure
    }

    // Create initial capture folder
    char initialFolder[30];
    sprintf(initialFolder, "/input/capture%04d", captureFolderNum);
    createDir(SD_MMC, initialFolder);

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(sdWriteTask, "SDWriteTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(pirTask, "PIRTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(audioTask, "AudioTask", 4096, NULL, 2, NULL, 1);
}

// Arduino loop function (empty, as tasks handle all work)
void loop() {
    vTaskDelay(portMAX_DELAY);
}

// Configure and initialize camera
int cameraSetup(void) {
    // Camera configuration structure
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; // LEDC channel for clock
    config.ledc_timer = LEDC_TIMER_0; // LEDC timer
    config.pin_d0 = Y2_GPIO_NUM; // Data pins
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM; // External clock
    config.pin_pclk = PCLK_GPIO_NUM; // Pixel clock
    config.pin_vsync = VSYNC_GPIO_NUM; // Vertical sync
    config.pin_href = HREF_GPIO_NUM; // Horizontal reference
    config.pin_sccb_sda = SIOD_GPIO_NUM; // I2C data
    config.pin_sccb_scl = SIOC_GPIO_NUM; // I2C clock
    config.pin_pwdn = PWDN_GPIO_NUM; // Power down
    config.pin_reset = RESET_GPIO_NUM; // Reset
    config.xclk_freq_hz = 20000000; // 20MHz clock
    config.frame_size = FRAMESIZE_VGA; // VGA resolution
    config.pixel_format = PIXFORMAT_JPEG; // JPEG format
    config.grab_mode = CAMERA_GRAB_LATEST; // Use latest frame
    config.fb_location = CAMERA_FB_IN_PSRAM; // Store frames in PSRAM
    config.jpeg_quality = 12; // JPEG quality (lower is higher quality)
    config.fb_count = 2; // Two frame buffers

    // Initialize camera with configuration
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        return 0; // Return 0 on failure
    }

    // Configure camera sensor settings
    struct _sensor *cam_sensor = esp_camera_sensor_get();
    cam_sensor->set_vflip(cam_sensor, 1); // Flip image vertically
    cam_sensor->set_brightness(cam_sensor, 1); // Increase brightness
    cam_sensor->set_saturation(cam_sensor, 0); // Normal saturation

    return 1; // Return 1 on success
}