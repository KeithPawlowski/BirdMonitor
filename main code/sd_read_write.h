#ifndef __SD_READ_WRITE_H
#define __SD_READ_WRITE_H

#include "Arduino.h"
#include "FS.h"
#include "SD_MMC.h"

// Pin definitions for SD card communication
#define SD_MMC_CMD  38 // Command pin
#define SD_MMC_CLK  39 // Clock pin
#define SD_MMC_D0   40 // Data pin

// Initialize SD card
void sdmmcInit(void); 

// List directory contents recursively
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
// Create a directory
void createDir(fs::FS &fs, const char * path);
// Remove a directory
void removeDir(fs::FS &fs, const char * path);
// Read a file's contents
void readFile(fs::FS &fs, const char * path);
// Write a string to a file
void writeFile(fs::FS &fs, const char * path, const char * message);
// Append a string to a file
void appendFile(fs::FS &fs, const char * path, const char * message);
// Rename a file
void renameFile(fs::FS &fs, const char * path1, const char * path2);
// Delete a file
void deleteFile(fs::FS &fs, const char * path);
// Test file I/O performance
void testFileIO(fs::FS &fs, const char * path);

// Write JPEG data to a file
void writejpg(fs::FS &fs, const char * path, const uint8_t *buf, size_t size);
// Count files in a directory
int readFileNum(fs::FS &fs, const char * dirname);

#endif