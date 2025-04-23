#include "sd_read_write.h"

// Initialize SD card in 1-bit mode
void sdmmcInit(void){
    // Configure SD card pins
    SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
    // Initialize SD card with specified settings
    SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5);
}

// List directory contents recursively
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    // Open directory
    File root = fs.open(dirname);
    // Check if directory is valid
    if(!root || !root.isDirectory()){
        return;
    }

    // Iterate through directory entries
    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            // Recursively list subdirectories if levels remain
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        }
        file = root.openNextFile();
    }
}

// Create a directory
void createDir(fs::FS &fs, const char * path){
    fs.mkdir(path);
}

// Remove a directory
void removeDir(fs::FS &fs, const char * path){
    fs.rmdir(path);
}

// Read a file's contents (reads but does not store)
void readFile(fs::FS &fs, const char * path){
    // Open file
    File file = fs.open(path);
    if(!file){
        return;
    }
    // Read all bytes
    while(file.available()){
        file.read();
    }
}

// Write a string to a file
void writeFile(fs::FS &fs, const char * path, const char * message){
    // Open file in write mode
    File file = fs.open(path, FILE_WRITE);
    if(file){
        // Write message
        file.print(message);
    }
}

// Append a string to a file
void appendFile(fs::FS &fs, const char * path, const char * message){
    // Open file in append mode
    File file = fs.open(path, FILE_APPEND);
    if(file){
        // Append message
        file.print(message);
    }
}

// Rename a file
void renameFile(fs::FS &fs, const char * path1, const char * path2){
    fs.rename(path1, path2);
}

// Delete a file
void deleteFile(fs::FS &fs, const char * path){
    fs.remove(path);
}

// Test file I/O performance
void testFileIO(fs::FS &fs, const char * path){
    // Open file for reading
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    if(file){
        // Read file contents
        len = file.size();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        file.close();
    }

    // Write test data
    file = fs.open(path, FILE_WRITE);
    if(file){
        // Write 1MB of data (512 bytes x 2048)
        for(size_t i = 0; i < 2048; i++){
            file.write(buf, 512);
        }
        file.close();
    }
}

// Write JPEG data to a file
void writejpg(fs::FS &fs, const char * path, const uint8_t *buf, size_t size){
    // Open file in write mode
    File file = fs.open(path, FILE_WRITE);
    if(file){
        // Write JPEG data
        file.write(buf, size);
    }
}

// Count files in a directory
int readFileNum(fs::FS &fs, const char * dirname){
    // Open directory
    File root = fs.open(dirname);
    if(!root || !root.isDirectory()){
        return -1;
    }

    // Count files
    File file = root.openNextFile();
    int num = 0;
    while(file){
        file = root.openNextFile();
        num++;
    }
    return num;  
}