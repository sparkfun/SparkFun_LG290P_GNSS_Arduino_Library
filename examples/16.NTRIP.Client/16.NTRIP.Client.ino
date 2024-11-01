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
#include "secrets.h"

#include <SparkFun_LG290P_GNSS.h> // Click here to get the library: http://librarymanager/All#SparkFun_LG290P

// Adjust these values according to your configuration
int pin_UART1_TX = 14;
int pin_UART1_RX = 13;
int gnss_baud = 460800;

LG290P myGNSS;
HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32

// The ESP32 core has a built in base64 library but not every platform does
// We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

//Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceivedRTCM_ms = 0; //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
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
  SerialGNSS.setRxBufferSize(1024);
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
}

void loop()
{
  if (Serial.available())
  {
    beginClient();
    while (Serial.available()) Serial.read(); // Empty buffer of any newline chars
  }

  Serial.println(F("Press any key to start NTRIP Client."));

  delay(1000);
}

// Connect to NTRIP Caster, receive RTCM, and push to its output to the LG290P over serial
void beginClient()
{
  WiFiClient ntripClient;
  long rtcmCount = 0;

  Serial.println(F("Subscribing to Caster. Press key to stop"));
  delay(10); // Wait for any serial to arrive
  while (Serial.available()) Serial.read(); // Flush

  while (Serial.available() == 0)
  {
    // Connect if we are not already. Limit to 5s between attempts.
    if (ntripClient.connected() == false)
    {
      Serial.print(F("Opening socket to "));
      Serial.println(casterHost);

      if (ntripClient.connect(casterHost, casterPort) == false) //Attempt connection
      {
        Serial.println(F("Connection to caster failed"));
        return;
      }
      else
      {
        Serial.printf("Connected to %s: %d\r\n", casterHost, casterPort);
        Serial.printf("Requesting NTRIP Data from mount point %s\r\n", mountPoint);

        const int SERVER_BUFFER_SIZE  = 512;
        char serverRequest[SERVER_BUFFER_SIZE + 1];

        snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun LG290P Client v1.0\r\n",
                 mountPoint);

        char credentials[512];
        if (strlen(casterUser) == 0)
        {
          strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
        }
        else
        {
          //Pass base64 encoded user:pw
          char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
          snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

          Serial.print(F("Sending credentials: "));
          Serial.println(userCredentials);

#if defined(ARDUINO_ARCH_ESP32)
          //Encode with ESP32 built-in library
          base64 b;
          String strEncodedCredentials = b.encode(userCredentials);
          char encodedCredentials[strEncodedCredentials.length() + 1];
          strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
          snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
#else
          //Encode with nfriendly library
          int encodedLen = base64_enc_len(strlen(userCredentials));
          char encodedCredentials[encodedLen]; //Create array large enough to house encoded data
          base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
#endif
        }
        strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
        strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

        Serial.print(F("serverRequest size: "));
        Serial.print(strlen(serverRequest));
        Serial.print(F(" of "));
        Serial.print(sizeof(serverRequest));
        Serial.println(F(" bytes available"));

        Serial.println(F("Sending server request:"));
        Serial.println(serverRequest);
        ntripClient.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripClient.available() == 0)
        {
          if (millis() - timeout > 5000)
          {
            Serial.println(F("Caster timed out!"));
            ntripClient.stop();
            return;
          }
          delay(10);
        }

        //Check reply
        bool connectionSuccess = false;
        char response[512];
        int responseSpot = 0;
        while (ntripClient.available())
        {
          if (responseSpot == sizeof(response) - 1) break;

          response[responseSpot++] = ntripClient.read();
          if (strstr(response, "200") != nullptr) //Look for 'ICY 200 OK'
            connectionSuccess = true;
          if (strstr(response, "401") != nullptr) //Look for '401 Unauthorized'
          {
            Serial.println(F("Hey - your credentials look bad! Check your caster username and password."));
            connectionSuccess = false;
          }
        }
        response[responseSpot] = '\0';

        Serial.print(F("Caster responded with: "));
        Serial.println(response);

        if (connectionSuccess == false)
        {
          Serial.print(F("Failed to connect to "));
          Serial.print(casterHost);
          Serial.print(F(": "));
          Serial.println(response);
          return;
        }
        else
        {
          Serial.print(F("Connected to "));
          Serial.println(casterHost);
          lastReceivedRTCM_ms = millis(); //Reset timeout
        }
      } //End attempt to connect
    } //End connected == false

    if (ntripClient.connected())
    {
      uint8_t rtcmData[512 * 8]; //Most incoming data is around 500 bytes but may be larger
      rtcmCount = 0;

      //Print any available RTCM data
      while (true)
      {
#if false
        if (ntripClient.available())
        {
            //Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
            rtcmData[rtcmCount++] = ntripClient.read();
            if (rtcmCount == sizeof(rtcmData)) break;
        }
        myGNSS.update();

      static unsigned long lastUpdate = 0;
      if (rtcmCount > 0 || millis() - lastUpdate > 1000)
      {
        if (rtcmCount > 0)
           lastReceivedRTCM_ms = millis();
        lastUpdate = millis();

        SerialGNSS.write(rtcmData, rtcmCount);
        rtcmCount = 0;
        // myGNSS.pushRawData(rtcmData, rtcmCount, false);
        Serial.printf("Lat/Long/Alt: %.8f/%.8f/%.2f\r\n", myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getAltitude());
        Serial.printf("Horizontal Speed: %.2fm/s Course: %.2f degrees\r\n",
                    myGNSS.getHorizontalSpeed(), myGNSS.getCourse());
        Serial.printf("Date (yyyy/mm/dd): %04d/%02d/%02d Time (hh:mm:ss) %02d:%02d:%02d.%03d\r\n",
                    myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay(),
                    myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), myGNSS.getMillisecond());
        Serial.printf("Satellites in view: %d\r\n", myGNSS.getSatellitesInViewCount());
        Serial.printf("Fix type: %d - ", myGNSS.getFixType());
        switch (myGNSS.getFixType())
        {
        default:
            Serial.println("Unknown");
            break;
        case 0:
            Serial.println("No fix");
            break;
        case 1:
            Serial.println("3D Fix");
            break;
        case 2:
            Serial.println("DGPS Fix");
            break;
        case 3:
            Serial.println("GPS PPS Mode, fix valid");
            break;
        case 4:
            Serial.println("RTK Fix");
            break;
        case 5:
            Serial.println("RTK Float");
            break;
        }
        Serial.println();
      }
#else
        int processRtcm = ntripClient.available();
        if (processRtcm)
        {
            uint8_t rtcmData[512];
            lastReceivedRTCM_ms = millis();
            for (size_t bytesToWrite = ntripClient.available(); bytesToWrite > 0; )
            {
                int count = std::min(sizeof rtcmData, bytesToWrite);
                count = ntripClient.readBytes(rtcmData, count);
                SerialGNSS.write(rtcmData, count);
                bytesToWrite -= count;
                myGNSS.update();
            }
        }
        myGNSS.update();

        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate > 1000)
        {
          lastUpdate = millis();
          displayData();
        }
#endif
      }
    }

    //Close socket if we don't have new data for 10s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
    {
      Serial.println(F("RTCM timeout. Disconnecting..."));
      if (ntripClient.connected() == true)
        ntripClient.stop();
      return;
    }

    delay(10);
  }

  Serial.println(F("User pressed a key"));
  Serial.println(F("Disconnecting..."));
  ntripClient.stop();
}

void parse(const uint8_t *data, int count)
{
    int offset = 0;
    while (offset < count)
    {
        bool good = count - offset > 6 && data[offset] == 0xD3;
        int payloadlen;
        if (good)
        {
            payloadlen = (data[offset + 1] << 8) | data[offset + 2];
            good = offset + payloadlen + 6  <= count;
        }

        if (good)
        {
            int type = (data[offset + 3] << 4) | (data[offset + 4] >> 4);
            Serial.printf("%d:%d ", type, payloadlen);

            if (type == 1005 || type == 1006)
            {
                double ecefX = extract_38bit_signed(data + offset + 3, 34) / 10000.0;
                double ecefY = extract_38bit_signed(data + offset + 3, 74) / 10000.0;
                double ecefZ = extract_38bit_signed(data + offset + 3, 114) / 10000.0;
                Serial.printf("(%f,%f,%f)\r\n", ecefX, ecefY, ecefZ);
            }
            offset += payloadlen + 6;
        }
        else
        {
            break;
        }
    }
    Serial.printf("**%d/%d**\r\n", offset, count);
}

int64_t extract_38bit_signed(const uint8_t *packet, int bit_offset)
{
    // Extract 38-bit value starting from the given bit_offset
    int64_t value = 0;
    int byte_offset = bit_offset / 8;
    int bit_in_byte = bit_offset % 8;
    
    // We need to grab up to 6 bytes and mask out the unused 2 bits.
    for (int i=0; i<6; ++i)
    {
      value <<= 8;
      value |= (int64_t)(packet[byte_offset + i]);
    }

    // Shift right to discard the unwanted bits and align to 38-bit value
    value >>= (10 - bit_in_byte);
    
    // Mask to keep only 38 bits
    value &= 0x3FFFFFFFFFLL;  // 38-bit mask (0x3FFFFFFFFF is 38 ones in binary)

    // Check if the sign bit (38th bit) is set and extend the sign for 64-bit int
    if (value & (1LL << 37)) {
        value |= ~((1LL << 38) - 1);  // Extend the sign
    }
    
    return value;
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

  Serial.printf("%02d/%02d/%04d %02d:%02d:%02d %-12.8f %-13.8f %-8.2f %-7.2f %-7.2f %-7.2f %-7.2f %-3d %-3d %-11s %-5.2f %-5.2f %-4d %-7.2f\r\n",
    myGNSS.getDay(), myGNSS.getMonth(), myGNSS.getYear(),
    myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(),
    myGNSS.getLatitude(), myGNSS.getLongitude(),
    myGNSS.getAltitude(), myGNSS.getHorizontalSpeed(), myGNSS.getNorthVelocity(), myGNSS.getEastVelocity(), myGNSS.getDownVelocity(),
    myGNSS.getSatellitesUsedCount(), myGNSS.getSatellitesInViewCount(), qualbuf, myGNSS.getHdop(), myGNSS.getPdop(),
    myGNSS.getLeapSeconds(), myGNSS.getGeoidalSeparation());
}

void displayHeader()
{
  const char *headings[] = { "Date", "Time", "Latitude", "Longitude", "Altitude", "Speed", "North", "East", "Down", "Sat", "SIV", "Fix-Quality", "HDOP", "PDOP", "Leap", "Sep" };
  int widths[] =           {  10,     8,      12,         13,          8,          7,       7,       7,      7,      3,     3,     11,            5,      5,      4,      7    };
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