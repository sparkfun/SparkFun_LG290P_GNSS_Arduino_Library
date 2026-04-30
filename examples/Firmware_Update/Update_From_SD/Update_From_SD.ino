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

int pin_SCK;
int pin_PICO; // microSD SDI
int pin_POCI; // microSD SDO
int pin_microSD_CS;
SdFs *sd;

HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32
#define pin_UART1_TX 4
#define pin_UART1_RX 13
int gnss_baud = 460800; // Baud rate for GNSS module

LG290P myGnss;

void setup()
{
    Serial.begin(115200);
    delay(250);

    Serial.println("LG290P Library firmware update test");

    // Redboard IoT
    pin_PICO = 23;
    pin_POCI = 19;
    pin_SCK = 18;
    pin_microSD_CS = 5;

    SPI.begin(pin_SCK, pin_POCI, pin_PICO);

    sd = new SdFat();

    if (sd->begin(SdSpiConfig(pin_microSD_CS, SHARED_SPI, SD_SCK_MHZ(16))) == false)
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

    Serial.println("r) Reset");
    Serial.println("u) Update firmware with lg_firmware.pkg on the SD card");
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
        else if (incoming == 'u')
        {
            Serial.println("Starting firmware update...");

            FsFile fwFile = sd->open("lg_firmware.pkg", O_RDONLY);
            if (!fwFile)
            {
                Serial.println("ERROR: Failed to open firmware file on SD");
                return;
            }

            uint32_t fileSize = fwFile.size();
            Serial.printf("Firmware file: %u bytes\r\n", fileSize);

            Serial.println("Calculating CRC32 of firmware file...");

            // Pass 1: compute CRC over the 4-byte LE size prefix then the file data
            uint32_t crc = LG290P::initFirmwareCrc32(fileSize);
            uint8_t chunk[512];
            while (fwFile.available())
            {
                size_t n = fwFile.read(chunk, sizeof(chunk));
                crc = LG290P::computeFirmwareCrc32(crc, chunk, n);
            }
            Serial.printf("CRC32: 0x%08X\r\n", crc);
            fwFile.seek(0);

            // Begin update: reboot, sync, version, firmware info, erase (~30 s)
            if (!myGnss.updateFirmwareBegin(fileSize, crc))
            {
                Serial.println("ERROR: Failed to enter bootloader mode. You may need to power cycle the unit to catch "
                               "it in bootloader mode.");
                fwFile.close();
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
            while (fwFile.available() && ok)
            {
                size_t n = fwFile.read(chunk, sizeof(chunk));
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
            fwFile.close();

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
        }
    }
}
