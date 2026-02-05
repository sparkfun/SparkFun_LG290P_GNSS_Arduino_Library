/*
  Use ESP32 WiFi to push RTCM data to RTK2Go (Caster) as a Server, using Survey In to calculate base location
  By: SparkFun Electronics / Nathan Seidle and Mikal Hart
  Date: November 10, 2024
  License: MIT. See license file for more information.

  This example shows how to gather RTCM data over Serial and push it to a casting service over WiFi.
  The base location is calculated using the "Survey In" technique.

  Here, the Arduino/ESP32 is acting as a 'server' to a 'caster'. In this case we will
  use RTK2Go.com as our caster because it is free. A rover (car, surveyor stick, etc)
  can then connect to RTK2Go as a 'client' and get the RTCM data it needs.

  You will need to register your mountpoint here: http://www.rtk2go.com/new-reservation/
  (They'll probably block the credentials we include in this example)

  To see if your mountpoint is active go here: http://rtk2go.com:2101/

  This is a proof of concept. Serving RTCM to a caster over WiFi is useful when you need to
  set up a high-precision base station.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  RTK Surveyor: https://www.sparkfun.com/products/17369

  Hardware Connections:
  Connect the LG290P to an ESP32 Thing Plus's UART.
  Open the serial monitor at 115200 baud to see the output
*/

#include <WiFi.h>
#include "secrets.h"
#include <SparkFun_LG290P_GNSS.h> // Click here to get the library: http://librarymanager/All#SparkFun_LG290P

WiFiClient ntripCaster;

// Adjust these values according to your configuration
//------------------------------------------------------------------------------
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

// Reset                                                 ___                            _____               _____
//  No connection --> Raspberry Pi Flex Hat J4-16 (RST) --> Flex connector J3-16 (RESET)--> LG290P 8 (Reset)
int pin_RESET = -1;

const char * platform = "ESPBERRY & SparkFun GNSS Flex pHAT";

#else  // ESP32_RPI_FLEX
#ifdef  POSTCARD

// https://www.sparkfun.com/sparkfun-rtk-postcard.html
int pin_UART1_TX = 21;
int pin_UART1_RX = 22;
int pin_RESET = 33;
const char * platform = "SparkFun RTK Postcard";

#else   // POSTCARD

// ???
int pin_UART1_TX = 14;
int pin_UART1_RX = 13;
int pin_RESET = -1;
const char * platform = "???";

#endif  // POSTCARD
#endif  // ESP32_RPI_FLEX

//------------------------------------------------------------------------------

int gnss_baud = 460800;
const int maxTimeBeforeHangup_ms = 10000; // If we fail to get a complete RTCM frame after 10s, then disconnect from caster

LG290P myGNSS;
HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32

// Global Variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSentRTCM_ms = 0;           // Time of last data pushed to socket
uint32_t serverBytesSent = 0;       // Just a running total
long lastReport_ms = 0;             // Time of last report of bytes sent
double ecefX, ecefY, ecefZ;         // ECEF coordinates
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
    Serial.begin(115200); // You may need to increase this for high navigation rates!
    delay(250);
    Serial.println();
    Serial.println("SparkFun NTRIP Server Survey In example");

  // Issue the reset
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

    Serial.print("Connecting to local WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.print("\nWiFi connected with IP: ");
    Serial.println(WiFi.localIP());

    bool response = myGNSS.rtcmSubscribeAll(rtcmCallback);
    response = response && myGNSS.setModeBase(false);
    if (response)
        Serial.println(F("Mode set successfully"));
    else
        Serial.println(F("Mode set failed!"));

    // Turn off unneeded NMEA sentences
    if (response)
    {
        response =
            myGNSS.setMessageRate("GLL", 0) &&
            myGNSS.setMessageRate("GSA", 0) &&
            myGNSS.setMessageRate("GSV", 0) &&
            myGNSS.setMessageRate("GGA", 0) &&
            myGNSS.setMessageRate("RMC", 0);

        if (response)
            Serial.println(F("NMEA messages were configured successfully"));
        else
            Serial.println(F("NMEA message configuration failed!"));
    }

    // Make sure necessary RTCM sentences are enabled
    if (response)
    {
        response =
            myGNSS.setMessageRate("RTCM3-1005", 1) &&
            myGNSS.setMessageRate("RTCM3-107X", 1, 0) &&
            myGNSS.setMessageRate("RTCM3-108X", 1, 0) &&
            myGNSS.setMessageRate("RTCM3-109X", 1, 0) &&
            myGNSS.setMessageRate("RTCM3-112X", 1, 0);

        if (response)
            Serial.println(F("RTCM messages enabled"));
        else
            Serial.println(F("RTCM failed to enable. Are you sure you have an LG290P?"));
    }

    if (response)
    {
        // -1280208.308,-4716803.847,4086665.811 is SparkFun HQ in ECEF coordinates so...
        // Note: If you leave these coordinates in place and setup your antenna *not* at SparkFun, your receiver
        // will be very confused and fail to generate correction data because, well, you aren't at SparkFun...
        // See this tutorial on getting PPP coordinates: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station/all
        //
        Serial.print("Setting 'survey in' mode... "); Serial.flush();
        response = myGNSS.setSurveyInMode(60);
        if (response)
            Serial.println(F("'survey in' mode set."));
        else
            Serial.println(F("failed to set 'survey in' mode."));
    }

    if (response)
    {
        int v = -1, c = -1;
        Serial.println("Waiting for status = VALID");
        while (true)
        {
            myGNSS.update();
            int validity = myGNSS.getSurveyInStatus();
            int count = myGNSS.getSurveyInObservations();
            if (v != validity || c != count)
            {
                Serial.printf("Status: %s, Count: %d/60\r\n", validity == 0 ? "INVALID" : validity == 1 ? "IN-PROGRESS" : "VALID", count);
                c = count;
                v = validity;
                if (validity >= 2)
                    break;
            }
        }
    }


    if (response)
    {
        Serial.println(F("Module configuration complete"));
        Serial.println(F("Press any key to start NTRIP Server."));
    }
    else
    {
        Serial.println(F("Freezing process..."));
        while (true)
            ;
    }
}

void loop()
{
    if (Serial.available())
    {
        beginServing();
        while (Serial.available()) Serial.read(); // Empty buffer of any newline chars
        Serial.println(F("Press any key to start NTRIP Server."));
    }
}

void beginServing()
{
    Serial.println("Feeding RTCM to caster. Press any key to stop");
    while (Serial.available()) // Flush the serial buffer
        Serial.read();

    if (!ntripCaster.connected())
    {
        Serial.printf("Opening HTTP connection to %s\r\n", casterHost);

        if (!ntripCaster.connect(casterHost, casterPort)) // Attempt connection
        {
            Serial.println(F("Connection to caster failed"));
            return;
        }
    }
    Serial.printf("Connected to %s:%d\r\n", casterHost, casterPort);
    Serial.printf("Feeding NTRIP Data from mount point %s\r\n", mountPoint);

    std::string request = std::string("SOURCE ") + mountPointPW + " /" + mountPoint + "\r\nSource-Agent: NTRIP SparkFun LG290P Server v1.0\r\n\r\n";
    Serial.printf("Sending request: '%s'\r\n", request.c_str());
    ntripCaster.write(request.c_str(), request.length());

    // Wait for response
    unsigned long timeout = millis();
    while (ntripCaster.available() == 0)
    {
        if (millis() - timeout > 5000)
        {
            Serial.println(F("Caster timed out!"));
            ntripCaster.stop();
            return;
        }
        myGNSS.update();
    }

    // Check reply
    char response[512] = {0};
    bool connectionSuccess = false;
    size_t bytesToRead = std::min(sizeof response - 1, (size_t)ntripCaster.available());
    size_t bytesRead = ntripCaster.readBytes(response, bytesToRead);
    if (strstr(response, "200") != nullptr) // Look for 'ICY 200 OK'
        connectionSuccess = true;
    else if (strstr(response, "401") != nullptr) // Look for '401 Unauthorized'
    {
        Serial.println(F("Hey - your credentials look bad! Check your caster username and password."));
        connectionSuccess = false;
    }

    Serial.printf("Caster responsed with %d bytes\r\n", bytesRead);
    Serial.printf("Caster responded with: '%s'\r\n", response);

    if (!connectionSuccess)
    {
        Serial.printf("Failed to connect to %s: %s\r\n", casterHost, response);
        return;
    }
    Serial.printf("Connected to %s\r\n", casterHost);
    lastReport_ms = lastSentRTCM_ms = millis(); // Reset timeout

    // This is the main sending loop. (Sending itself happens in RTCM callback function below.)
    while (ntripCaster.connected() && !Serial.available())
    {
        myGNSS.update(); // Process bytes as they arrive from LG290P

        // Terminate HTTP connection if we don't have new data for 10s
        // RTK2Go will ban your IP address if you abuse it. See http://www.rtk2go.com/how-to-get-your-ip-banned/
        // So let's not leave the connection open/hanging without data
        if (millis() - lastSentRTCM_ms > maxTimeBeforeHangup_ms)
        {
            Serial.println("RTCM timeout. Disconnecting...");
            ntripCaster.stop();
            return;
        }

        // Report some statistics every second
        if (millis() - lastReport_ms >= 1000)
        {
            lastReport_ms = millis();
            displayData();
        }
    }

    if (Serial.available())
    {
        Serial.println(F("User pressed a key"));
        Serial.println(F("Disconnecting..."));
    }
    else
    {
        Serial.println(F("Lost connection..."));
    }
    ntripCaster.stop();

    while (Serial.available()) // Flush any residual data in serial buffer
        Serial.read();
}
void displayData()
{
  // Every 20th line draw the helpful header
  static int linecount = 0;
  if (linecount++ % 20 == 0)
  {
    displayHeader();
  }

  Serial.printf("%02d:%02d:%02d %-12.3f %-12.3f %-12.3f %4s %-6d %-6d %-6d %-6d %-6d %-10d\r\n",
    myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(),
    ecefX, ecefY, ecefZ, "",
    myGNSS.getRtcmCount(1005), myGNSS.getRtcmCount(1074), myGNSS.getRtcmCount(1084), myGNSS.getRtcmCount(1094), myGNSS.getRtcmCount(1124),
    serverBytesSent);
}

void displayHeader()
{
  const char *headings[] = { "Time", "ECEF-X", "ECEF-Y", "ECEF-Z", "RTCM", "1005", "1074", "1084", "1094", "1124", "Bytes-Sent"};
  int widths[] =           {  8,      12,       12,       12,       4,      6,      6,      6,      6,      6,      10 };
  int items = sizeof widths / sizeof widths[0];
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
    if (ntripCaster.connected())
    {
        // Serial.printf("Sending RTCM packet type %d...\r\n", packet.type);
        ntripCaster.write(packet.buffer, packet.bufferlen);
        lastSentRTCM_ms = millis();
        serverBytesSent += packet.bufferlen;
        if (packet.type == 1005)
        {
            ecefX = packet.getEcefX();
            ecefY = packet.getEcefY();
            ecefZ = packet.getEcefZ();
        }
    }
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
