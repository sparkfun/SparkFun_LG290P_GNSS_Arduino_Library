/*
  Use ESP32 WiFi to get RTCM data from RTK2Go (caster) as a Client
  By: Nathan Seidle and Mikal Hart
  Date: October 20, 2024
  License: MIT. See license file for more information.

  This example shows how to obtain RTCM data from a NTRIP Caster over WiFi
  and push it over Serial to an LG290P

  In this example, the microcontroller is acting as a client to a 'caster'. In this case we will
  use RTK2Go.com as our caster because it is free. See the NTRIPServer example to see how
  to push RTCM data to the caster.

  You will need to have a valid mountpoint available. To see available mountpoints go here: http://rtk2go.com:2101/

  This is a proof of concept to show how to connect to a caster via HTTP. Using WiFi for a rover
  is generally a bad idea because of limited WiFi range in the field.

  For more information about NTRIP Clients and the differences between Rev1 and Rev2 of the protocol
  please see: https://www.use-snip.com/kb/knowledge-base/ntrip-rev1-versus-rev2-formats/

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  RTK Surveyor: https://www.sparkfun.com/products/18443
  RTK Express: https://www.sparkfun.com/products/18442

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ESP32 Thing Plus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/
#include <WiFi.h>
#include <SparkFun_LG290P_GNSS.h> // Click here to get the library: http://librarymanager/All#SparkFun_LG290P
#include "secrets.h"

// Adjust these values according to your configuration
int pin_UART1_TX = 14;
int pin_UART1_RX = 13;
int gnss_baud = 460800;

LG290P myGNSS;
HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32

// The ESP32 core has a built in base64 library but not every platform does
// We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" // Built-in ESP32 library
#else
#include <Base64.h> // nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

// Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceivedRTCM_ms = 0;
int maxTimeBeforeHangup_ms = 10000; // If we fail to get a complete RTCM frame after 10s, then disconnect from caster
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun NTRIP example");
  Serial.println("Initializing device...");

  // We must start the serial port before using it in the library
  // Increase buffer size to handle high baud rate streams
  SerialGNSS.setRxBufferSize(4096);
  SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);
  
  myGNSS.enableDebugging(Serial); // Print all debug to Serial
  if (myGNSS.begin(SerialGNSS) == false)     // Give the serial port over to the library
  {
    Serial.println("LG290P failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }
  Serial.println("LG290P detected!");

  Serial.print(F("Connecting to local WiFi"));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.print(F("WiFi connected with IP: "));
  Serial.println(WiFi.localIP());

  while (Serial.available()) Serial.read();
  Serial.println(F("Press any key to start NTRIP Client."));
}

void loop()
{
  if (Serial.available())
  {
    beginClient();
    while (Serial.available()) Serial.read(); // Empty buffer of any newline chars
    Serial.println(F("Press any key to start NTRIP Client."));
  }
}

// Connect to NTRIP Caster, receive RTCM, and push to its output to the LG290P over serial
void beginClient()
{
  WiFiClient ntripClient;

  Serial.println(F("Subscribing to caster. Press key to stop"));
  delay(10); // Wait for any serial to arrive
  while (Serial.available()) Serial.read(); // Flush

  // Make sure we're connected
  if (!ntripClient.connected())
  {
    Serial.printf("Opening HTTP connection to %s\r\n", casterHost);

    if (!ntripClient.connect(casterHost, casterPort)) // Attempt connection
    {
      Serial.println(F("Connection to caster failed"));
      return;
    }
  }

  Serial.printf("Connected to %s: %d\r\n", casterHost, casterPort);
  Serial.printf("Requesting NTRIP Data from mount point %s\r\n", mountPoint);

  std::string request = std::string("GET /") + mountPoint + " HTTP/1.0\r\nUser-Agent: NTRIP SparkFun LG290P Client v1.0\r\n";
  std::string credentials;
  if (strlen(casterUser) == 0)
  {
    credentials = "Accept: */*\r\nConnection: close\r\n";
  }
  else
  {
    // Pass base64 encoded user:pw
    std::string userCredentials = std::string(casterUser) + ":" + casterUserPW;
    Serial.printf("Sending credentials: %s\r\n", userCredentials.c_str());

#if defined(ARDUINO_ARCH_ESP32)
    // Encode with ESP32 built-in library
    base64 b;
    String strEncodedCredentials = b.encode(userCredentials.c_str());
    credentials = std::string("Authorization: Basic ") + strEncodedCredentials.c_str() + "\r\n";
#else
    // Encode with nfriendly library
    int encodedLen = base64_enc_len(userCredentials.length());
    char encodedCredentials[encodedLen]; // Create array large enough to house encoded data
    base64_encode(encodedCredentials, userCredentials, userCredentials.length()); // Note: Input array is consumed
    credentials = std::string("Authorization: Basic ") + encodedCredentials + "\r\n";
#endif
  }

  request += credentials + "\r\n";

  Serial.printf("Sending server request: '%s'\r\n", request.c_str());
  ntripClient.write(request.c_str(), request.length());

  // Wait for response
  unsigned long timeout = millis();
  while (ntripClient.available() == 0)
  {
    if (millis() - timeout > 5000)
    {
      Serial.println(F("Caster timed out!"));
      ntripClient.stop();
      return;
    }
    myGNSS.update();
  }

  // Check reply
  char response[512] = {0};
  bool connectionSuccess = false;
  size_t bytesToRead = std::min(sizeof response, (size_t)ntripClient.available());
  size_t bytesRead = ntripClient.readBytes(response, bytesToRead);
  if (strstr(response, "200") != nullptr) // Look for 'ICY 200 OK'
    connectionSuccess = true;
  else if (strstr(response, "401") != nullptr) //Look for '401 Unauthorized'
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
  lastReceivedRTCM_ms = millis(); // Reset timeout

  // Main processing loop
  while (ntripClient.connected() && !Serial.available())
  {
    // Is there any RTCM data to process?
    size_t bytesToWrite = ntripClient.available();
    if (bytesToWrite)
    {
        lastReceivedRTCM_ms = millis();
        uint8_t rtcmData[64];
        while (bytesToWrite > 0)
        {
            size_t count = ntripClient.readBytes(rtcmData, std::min(sizeof rtcmData, bytesToWrite));

            // Wait for sufficient space in TX serial buffer before transmitting RTCM to LG290P
            while (SerialGNSS.availableForWrite() < count)
              myGNSS.update();

            SerialGNSS.write(rtcmData, count);
            bytesToWrite -= count;
            myGNSS.update();
        }
    }

    // Make sure to keep handling any incoming sentences from the LG290P
    myGNSS.update();

    // Time to display some data?
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 1000)
    {
      lastUpdate = millis();
      displayData();
    }

    // Terminate connection if data flow has stalled for 10 seconds
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
    {
      Serial.println(F("RTCM timeout. Disconnecting..."));
      if (ntripClient.connected())
        ntripClient.stop();
      return;
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
  ntripClient.stop();
}

void displayData()
{
  // Every 20th line draw the helpful header
  static int linecount = 0;
  if (linecount++ % 20 == 0)
  {
    displayHeader();
  }

  // Fix quality requires some special formatting
  char qualbuf[32];
  const char *qualities[] = { "No-Fix", "3D-Fix", "DGPS-Fix", "GPS-PPS", "RTK-Fix", "RTK-Flt" };
  int qual = myGNSS.getFixQuality();
  snprintf(qualbuf, sizeof qualbuf, "%s(%d)", (qual >= 0 && qual <= 5) ? qualities[qual] : "Unknown", qual);

  Serial.printf("%02d/%02d/%04d %02d:%02d:%02d %-12.8f %-13.8f %-8.2f %-3d %-3d %-11s %-5.2f %-5.2f %-7.3f %-7.3f %-7.3f %-7.3f %-7.3f\r\n",
    myGNSS.getDay(), myGNSS.getMonth(), myGNSS.getYear(),
    myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(),
    myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getAltitude(), 
    myGNSS.getSatellitesUsedCount(), myGNSS.getSatellitesInViewCount(), qualbuf, myGNSS.getHdop(), myGNSS.getPdop(),
    myGNSS.getNorthError(), myGNSS.getEastError(), myGNSS.getDownError(), myGNSS.get2DError(), myGNSS.get3DError());
}

void displayHeader()
{
  const char *headings[] = { "Date", "Time", "Latitude", "Longitude", "Altitude", "Sat", "SIV", "Fix-Quality", "HDOP", "PDOP", "N Err", "E Err", "D Err", "2D Err", "3D Err" };
  int widths[] =           {  10,     8,      12,         13,          8,          3,     3,     11,            5,      5,      7,       7,       7,       7,      7    };
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