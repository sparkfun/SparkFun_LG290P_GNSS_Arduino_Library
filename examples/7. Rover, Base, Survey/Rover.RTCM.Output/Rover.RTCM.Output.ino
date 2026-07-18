/*
  Enable RTCM Ephemeris output - to better understand the RTCM-1230 changes with firmware v2.02
  By: SparkFun Electronics / Paul Clark, Nathan Seidle and Mikal Hart
  Date: July 17, 2024
  License: MIT. See license file for more information.

  This example shows how to configure RTCM data over Serial.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  LG290P Breakout: https://www.sparkfun.com/sparkfun-quadband-gnss-rtk-breakout-lg290p-qwiic.html
  LG290P PiHat: https://www.sparkfun.com/sparkfun-gnss-flex-phat-lg290p.html

  Hardware Connections:
  Connect the LG290P to an ESP32 Thing Plus's UART.
  Open the serial monitor at 115200 baud to see the output
*/

#include <SparkFun_LG290P_GNSS.h> // Click here to get the library: http://librarymanager/All#SparkFun_LG290P

// Adjust these values according to your configuration
//------------------------------------------------------------------------------

// Pre-defined boards - comment / uncomment as needed:
// #define ESP32_RPI_FLEX
// #define POSTCARD
#define ESP32_THING_PLUS_C

// https://www.sparkfun.com/sparkfun-gnss-flex-phat.html
#ifdef ESP32_RPI_FLEX

// ESP32 WROOM with 40-pin Raspberry Pi GPIO connector
// https://copperhilltech.com/esp32-development-board-with-raspberry-pi-gpio/

// UART1_TX (IO15) --> RPi GPIO Connector 10 (GPIO15/TXD) --> Raspberry Pi Flex Hat J4-19 (RXD1) --> Flex connector J3-19 (RXD1) --> LG290P 21 (RXD)
//int pin_UART1_TX = 15;

// UART1_RX (IO14) <-- RPi GPIO Connector 8 (GPIO14/RXD) <-- Raspberry Pi Flex Hat J4-15 (TXD1) <-- Flex connector J3-15 (TXD1) <-- LG290P 20 (TXD1)
//int pin_UART1_RX = 14;

// UART1_TX (IO5/CE0) --> RPi GPIO Connector 24 (GPIO8/CE0) --> Raspberry Pi Flex Hat J4-12 (RXD2) --> Flex connector J3-12 (RXD2) --> LG290P 7 (RXD)
int pin_UART1_TX = 5;

// UART1_RX (IO19/MISO) <-- RPi GPIO Connector 21 (GPIO9/MISO) <-- Raspberry Pi Flex Hat J4-10 (TXD2) <-- Flex connector J3-10 (TXD2) <-- LG290P 6 (TXD1)
int pin_UART1_RX = 19;

// Reset                                           ___                            _____               _____
//  No connection --> Raspberry Pi Flex Hat J4-16 (RST) --> Flex connector J3-16 (RESET)--> LG290P 8 (Reset)
int pin_RESET = -1;

const char * platform = "ESPBERRY & SparkFun GNSS Flex pHAT";

#else  // ESP32_RPI_FLEX
#ifdef  POSTCARD

// https://www.sparkfun.com/sparkfun-rtk-postcard.html
int pin_UART1_TX = 22;
int pin_UART1_RX = 21;
int pin_RESET = 33;
const char * platform = "SparkFun RTK Postcard";

#else   // POSTCARD
#ifdef ESP32_THING_PLUS_C

// https://www.sparkfun.com/sparkfun-thing-plus-esp32-wroom-usb-c.html
int pin_UART1_TX = 17;
int pin_UART1_RX = 16;
int pin_RESET = 4; // The Free pin
const char * platform = "SparkFun ESP32 Thing Plus C";

#else   // ESP32_THING_PLUS_C

// ???
int pin_UART1_TX = 14;
int pin_UART1_RX = 13;
int pin_RESET = -1;
const char * platform = "???";

#endif  // ESP32_THING_PLUS_C
#endif  // POSTCARD
#endif  // ESP32_RPI_FLEX

//------------------------------------------------------------------------------

int gnss_baud = 460800;

LG290P myGNSS;
HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
    Serial.begin(115200); // You may need to increase this for high navigation rates!
    delay(250);
    Serial.println();
    Serial.println("SparkFun RTCM Output example");

  // Issue the reset - if pin_RESET is defined
  if (pin_RESET != -1)
  {
    Serial.println("Resetting the LG290P");
    pinMode(pin_RESET, OUTPUT);
    digitalWrite(pin_RESET, 0);
    delay(100);
    digitalWrite(pin_RESET, 1);
  }

    Serial.println("Initializing device...");

    // We must start the serial port before using it in the library
    // Increase buffer size to handle high baud rate streams
    SerialGNSS.setRxBufferSize(4096);
    SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

    // myGNSS.enableDebugging(Serial);        // Print all debug to Serial
    if (myGNSS.begin(SerialGNSS, "SFE_LG290P_GNSS_Library", output) == false) // Give the serial port over to the library
    {
        Serial.println("LG290P failed to respond. Check ports and baud rates. Freezing...");
        while (true)
            ;
    }
    Serial.println("LG290P detected!");

    // Check firmware version and print info
    int versionMajor;
    int versionMinor;
    int versionCombined;
    myGNSS.getFirmwareVersionMajor(versionMajor);
    myGNSS.getFirmwareVersionMinor(versionMinor);
    myGNSS.getFirmwareVersion(versionCombined); // v2.01 becomes 201
    Serial.printf("Firmware v%d.%d (%d)\r\n", versionMajor, versionMinor, versionCombined);

    Serial.println("Ensuring ROVER mode");
    myGNSS.ensureModeRover();

    bool response = true;

    // Reduce the fix interval to 1000ms
    if (response)
    {
        const int fixInterval = 1000;
        Serial.printf("Changing the fix interval to %dms\r\n", fixInterval);
        response = myGNSS.setFixInterval(fixInterval);

        if (response)
            Serial.println(F("Fix interval configured successfully"));
        else
            Serial.println(F("Fix interval configuration failed!"));
    }

    // Subscribe to all RTCM messages. Call rtcmCallback when each message arrives
    if (response)
    {
        response = myGNSS.rtcmSubscribeAll(rtcmCallback);

        if (response)
            Serial.println(F("RTCM callback configured successfully"));
        else
            Serial.println(F("RTCM callback configuration failed!"));
    }

    // Turn off unneeded NMEA sentences
    if (response)
    {
        response =
            // setMessageRate(const char *msgName, int rate, int msgVer = -1)
            // if (msgVer == -1) the message version is omitted
            myGNSS.setMessageRate("GLL", 0) &&
            myGNSS.setMessageRate("GSA", 0) &&
            myGNSS.setMessageRate("GSV", 0) &&
            myGNSS.setMessageRate("GGA", 0) &&
            myGNSS.setMessageRate("RMC", 0) &&
            myGNSS.setMessageRate("VTG", 0);

        if (response)
            Serial.println(F("NMEA messages were configured successfully"));
        else
            Serial.println(F("NMEA message configuration failed!"));
    }

    // Turn on needed PQTM sentences
    // See issue #26. We can't rely on getHour etc. to enable PVT if getHour
    // is only called from within update(). We need to enable PVT manually.
    if (response)
    {
        response =
            // setMessageRate(const char *msgName, int rate, int msgVer = -1)
            // if (msgVer == -1) the message version is omitted
            myGNSS.setMessageRate("PQTMPVT", 1, 1);

        if (response)
            Serial.println(F("PQTM messages were configured successfully"));
        else
            Serial.println(F("PQTM message configuration failed!"));
    }

    // Set the Ephemeris interval (EPH_Interval) to (e.g.) 10 seconds
    // The library does not (yet) include a helper method for PQTMCFGRTCM so
    // we need to configure manually with sendOkCommand
    if (response)
    {
        const int ephInterval = 10;
        char cfgRtcm[40];
        snprintf(cfgRtcm, sizeof(cfgRtcm), ",W,4,0,15,07,06,2,%d", ephInterval); // Enable MSM4
        response = myGNSS.sendOkCommand("PQTMCFGRTCM", cfgRtcm);

        if (response)
            Serial.println(F("RTCM configured successfully"));
        else
            Serial.println(F("RTCM configuration failed!"));
    }

    // Enable RTCM Ephemeris sentences
    if (response)
    {
        response =
            // setMessageRate(const char *msgName, int rate, int msgVer = -1)
            // if (msgVer == -1) the message version is omitted
            myGNSS.setMessageRate("RTCM3-1019", 1, -1) &&
            myGNSS.setMessageRate("RTCM3-1020", 1, -1) &&
            myGNSS.setMessageRate("RTCM3-1042", 1, -1) &&
            myGNSS.setMessageRate("RTCM3-1046", 1, -1);

        if (response)
        {
            // Protocol Specification says 1230 rate can only be set to "1" (or "0")
            // Yet we can successfully set a rate of (e.g.) 20 here...
            const int rate1230 = 20;
            response = myGNSS.setMessageRate("RTCM3-1230", rate1230, -1);
        }

        if (response)
            Serial.println(F("RTCM messages enabled"));
        else
            Serial.println(F("RTCM messages failed to enable"));
    }

    if (response)
    {
        Serial.println(F("Module configuration complete"));
    }
}

void loop()
{
    myGNSS.update(); // Process bytes as they arrive from LG290P. Call rtcmCallback as needed
}

void displayData(int rtcm)
{
  // After a gap of >= 2s, display the helpful header
  static unsigned long lastPrint = 0;
  if ((millis() - lastPrint) > 2000)
    displayHeader();
  lastPrint = millis();

  Serial.printf("%02d:%02d:%02d %-4d %-6d %-6d %-6d %-6d %-6d\r\n",
    myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), rtcm,
    myGNSS.getRtcmCount(1019), myGNSS.getRtcmCount(1020), myGNSS.getRtcmCount(1042), myGNSS.getRtcmCount(1046), myGNSS.getRtcmCount(1230));
}

void displayHeader()
{
  const char *headings[] = { "Time", "RTCM", "1019", "1020", "1042", "1046", "1230"};
  int widths[] =           {  8,      4,      6,      6,      6,      6,     6 };
  int items = sizeof(widths) / sizeof(widths[0]);
  Serial.println();

  // Header
  for (int i=0; i<items; ++i)
  {
    char buf[10]; sprintf(buf, "%%-%ds ", widths[i]);
    Serial.printf(buf, headings[i]);
  }
  Serial.println();

  // Dashes
  for (int i=0; i<items; ++i)
  {
    std::string dashes(widths[i], '-');
    Serial.printf("%s%s", dashes.c_str(), i == items - 1 ? "" : "-");
  }
  Serial.println();
}

void rtcmCallback(RtcmPacket &packet)
{
    displayData(packet.type);
}

//----------------------------------------
// Output a buffer of data
//
// Inputs:
//   buffer: Address of a buffer of data to output
//   length: Number of bytes of data to output
//----------------------------------------
void output(uint8_t * buffer, size_t length)
{
    size_t bytesWritten;

    if (Serial)
    {
        while (length)
        {
            // Wait until space is available in the FIFO
            while (Serial.availableForWrite() == 0);

            // Output the character
            bytesWritten = Serial.write(buffer, length);
            buffer += bytesWritten;
            length -= bytesWritten;
        }
    }
}
