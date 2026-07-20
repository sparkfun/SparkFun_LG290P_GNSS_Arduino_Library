/*
  Update the LG290P firmware from a file one an SD card
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 30th April 2026
  License: MIT. Please see LICENSE.md for more information.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Quadband GNSS RTK Breakout - LG290P (GPS-26620) https://www.sparkfun.com/products/26620

  This example shows how to update the firmware on the LG290P using the SparkFun_LG290P_GNSS library and an SD card. 
  The firmware file must be named "lg_firmware.pkg" and placed in the root directory of the SD card. 
  The module version will be output at the start, and after a successful update, the new version will be printed.  

  Hardware Connections:
  Connect RX3 (green wire) of the LG290P to pin 15 on the ESP32
  Connect TX3 (orange wire) of the LG290P to pin 4 on the ESP32
  To make this easier, a 4-pin locking JST cable can be purchased here: https://www.sparkfun.com/products/17240
  Note: Almost any ESP32 pins can be used for serial.
  Connect a multi-band GNSS antenna: https://www.sparkfun.com/products/21801
*/

#include "SdFat.h"
#include <SparkFun_LG290P_GNSS.h>

// Adjust these values according to your configuration
//------------------------------------------------------------------------------

// Pre-defined boards - comment / uncomment as needed:
// #define POSTCARD
#define ESP32_THING_PLUS_C

#ifdef  POSTCARD

// https://www.sparkfun.com/sparkfun-rtk-postcard.html
int pin_UART1_TX = 22;
int pin_UART1_RX = 21;
int pin_RESET = 33;
int pin_spiSCK = 32;
int pin_spiPICO = 26; //microSD SDI
int pin_spiPOCI = 25; //microSD SDO
int pin_microSD_CS = 27;
const char * platform = "SparkFun RTK Postcard";

#else   // POSTCARD
#ifdef ESP32_THING_PLUS_C

// https://www.sparkfun.com/sparkfun-thing-plus-esp32-wroom-usb-c.html
int pin_UART1_TX = 17;
int pin_UART1_RX = 16;
int pin_RESET = 4; // The Free pin
int pin_SCK = 18;
int pin_PICO = 23; // microSD SDI
int pin_POCI = 19; // microSD SDO
int pin_microSD_CS = 5;
const char * platform = "SparkFun ESP32 Thing Plus C";

#else  // ESP32_THING_PLUS_C

// Redboard IoT
int pin_UART1_TX = 4;
int pin_UART1_RX = 13;
int pin_RESET = -1;
int pin_SCK = 18;
int pin_PICO = 23; // microSD SDI
int pin_POCI = 19; // microSD SDO
int pin_microSD_CS = 5;
const char * platform = "Redboard IoT";

#endif  // ESP32_THING_PLUS_C
#endif  // POSTCARD

#define SD_CONFIG SdSpiConfig(pin_microSD_CS, SHARED_SPI, SD_SCK_MHZ(16))
//#define SD_CONFIG SdSpiConfig(pin_microSD_CS, DEDICATED_SPI, SD_SCK_MHZ(16))

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
#if SD_FAT_TYPE == 0
SdFat sd;
File dir;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 dir;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile dir;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile dir;
FsFile file;
#else  // SD_FAT_TYPE
#error invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32
int gnss_baud = 460800; // Baud rate for GNSS module

LG290P myGnss;

bool useSoftwareReset = true;

void printMenu()
{
    Serial.println();
    Serial.println("r) Reset ESP32");
    if (pin_RESET >= 0)
        Serial.printf("s) Use software reset: %s\r\n", useSoftwareReset ? "enabled" : "disabled");
    Serial.println("u) Update firmware with lg*.pkg from the SD card root folder");
}

void setup()
{
    Serial.begin(115200);
    delay(250);

    Serial.println("LG290P Library firmware update test");
    Serial.flush();

    if (pin_RESET >= 0) {
        pinMode(pin_RESET, OUTPUT);
        digitalWrite(pin_RESET, HIGH);
    }

    SPI.begin(pin_SCK, pin_POCI, pin_PICO);

    if (sd.begin(SD_CONFIG) == false)
    {
        Serial.println("Failed to start SD. Freezing...");
        while (1)
            ;
    }
    Serial.println("SD started");

    SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

    Serial.println("Starting connection to GNSS module...");

    if (myGnss.begin(SerialGNSS, "LG290P") == true)
        Serial.println("GNSS module found");
    else
        Serial.println(
            "Failed to find GNSS module. It may have been damaged by a previous failed update attempt. Proceeding.");

    std::string version, buildDate, buildTime;
    if (myGnss.getVersionInfo(version, buildDate, buildTime))
        Serial.printf("Version info: %s %s %s\r\n", version.c_str(), buildDate.c_str(), buildTime.c_str());
    else
        Serial.printf("Version info unavailable");

    printMenu();
}

void loop()
{
    if (Serial.available())
    {
        byte incoming = Serial.read();
        if (incoming == 'r')
        {
            ESP.restart();
        }
        else if (incoming == 's')
        {
            useSoftwareReset ^= 1;
            printMenu();
        }
        else if (incoming == 'u')
        {
            Serial.println("Select firmware file:");

            // Open root directory
            if (!dir.open("/")) {
                Serial.println("ERROR: dir.open failed");
                return;
            }

            // Open next file in root.
            // Warning, openNext starts at the current position of dir so a
            // rewind may be necessary in your application.
            bool fileSelected = false;
            while (file.openNext(&dir, O_RDONLY)) {
                if (!file.isDir()) {
                    char fileName[50];
                    file.getName(fileName, 50);
                    size_t fileNameLen = strlen(fileName);
                    //Serial.println(fileName);
                    if (fileNameLen > 10) {
                        if (((fileName[0] == 'L') || (fileName[0] == 'l')) &&
                            ((fileName[1] == 'G') || (fileName[1] == 'g')) &&
                            (fileName[fileNameLen - 4] == '.') &&
                            ((fileName[fileNameLen - 3] == 'P') || (fileName[fileNameLen - 3] == 'p')) &&
                            ((fileName[fileNameLen - 2] == 'K') || (fileName[fileNameLen - 2] == 'k')) &&
                            ((fileName[fileNameLen - 1] == 'G') || (fileName[fileNameLen - 1] == 'g'))) {
                            Serial.print("Update with ");
                            Serial.print(fileName);
                            Serial.print(" (");
                            Serial.print(file.fileSize());
                            Serial.println(" bytes) (Y/N)?");
                            do {
                                if (Serial.available())
                                {
                                    byte incoming = Serial.read();
                                    if ((incoming == 'Y') || (incoming == 'y'))
                                        fileSelected = true;
                                    break;
                                }
                            } while (1);
                        }
                    }
                }
                if (!fileSelected)
                    file.close();
                else
                    break;
            }

            if (!file)
            {
                Serial.println("ERROR: no firmware file found / selected");
                return;
            }

            Serial.println("Starting firmware update...");

            uint32_t fileSize = file.size();
            Serial.printf("Firmware file: %lu bytes\r\n", fileSize);

            Serial.println("Calculating CRC32 of firmware file...");

            // Pass 1: compute CRC over the 4-byte LE size prefix then the file data
            uint32_t crc = LG290P::initFirmwareCrc32(fileSize);
            uint8_t chunk[512];
            while (file.available())
            {
                size_t n = file.read(chunk, sizeof(chunk));
                crc = LG290P::computeFirmwareCrc32(crc, chunk, n);
            }
            Serial.printf("CRC32: 0x%08X\r\n", crc);
            file.seek(0);

            // Set skipSoftwareReset true if we have pin_RESET and useSoftwareReset is true
            bool skipSoftwareReset = false;
            if (pin_RESET >= 0) {
                skipSoftwareReset = !useSoftwareReset;

                if (skipSoftwareReset) {
                    Serial.println("Doing a hardware reset to put LG290P into bootloader mode");
                    digitalWrite(pin_RESET, LOW);
                    delay(100);
                    digitalWrite(pin_RESET, HIGH);
                    delay(250); // Tune and test this delay carefully!
                }
            }

            // Begin update: reboot, sync, version, firmware info, erase (~30 s)
            if (!myGnss.updateFirmwareBegin(fileSize, crc, skipSoftwareReset))
            {
                Serial.println("ERROR: Failed to enter bootloader mode. You may need to reset"
                               " / power cycle the unit to catch it in bootloader mode.");
                file.close();
                return;
            }
            Serial.println("Device erased. Uploading firmware...");

            // Pass 2: stream firmware to device

            // Indicate progress
            int barWidthInCharacters = 20; // Width of progress bar, ie [###### % complete
            long portionSize = fileSize / barWidthInCharacters;
            int barWidth = 0;

            uint32_t totalSent = 0;
            bool ok = true;
            while (file.available() && ok)
            {
                size_t n = file.read(chunk, sizeof(chunk));
                ok = myGnss.updateFirmware(chunk, n);
                totalSent += n;

                // Indicate progress
                if (totalSent > barWidth * portionSize)
                {
                    // Advance the bar
                    barWidth++;
                    Serial.print("\n\r[");
                    for (int x = 0; x < barWidth; x++)
                        Serial.print("=");
                    Serial.printf("%d%%", totalSent * 100 / fileSize);
                    if (totalSent == fileSize)
                        Serial.println("]");
                }
            }
            file.close();

            if (!ok)
            {
                Serial.println("ERROR: Firmware upload failed");
                return;
            }

            Serial.print("Sending last packet. Device will then take up to 30 seconds to verify and reboot... ");
            bool endOk = myGnss.updateFirmwareEnd();
            Serial.println(endOk ? "OK" : "FAILED");
            if (!endOk)
                return;

            Serial.print("updateFirmwareIsFinished: ");
            bool finishOk = myGnss.updateFirmwareIsFinished();
            Serial.println(finishOk ? "OK - firmware update successful!"
                                    : "FAILED (device did not reboot within 15 seconds)");

            std::string newVersion, newBuildDate, newBuildTime;
            if (myGnss.getVersionInfo(newVersion, newBuildDate, newBuildTime))
                Serial.printf("Version info: %s %s %s\r\n", newVersion.c_str(), newBuildDate.c_str(),
                              newBuildTime.c_str());
            else
                Serial.printf("Version info unavailable");

            printMenu();
        }
    }
}
