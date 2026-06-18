/*
  ESP32_Firmware_Update.ino

  Work with the Firmware_Update program on the PC to update the firmware
  on the Quectel GNSS device.  The following messages are exchanged.

        .----------.                      .-----------.
        |    PC    |   Hello Micro        |   Micro   |
        |          |--------------------->|           |
        |          |<---------------------|           |
        |          |   Hello PC           |           |
        |          |                      |           |
        |          |   Reset GNSS         |           |
        |          |--------------------->|           |
        |          |<---------------------|           |
        |          |   GNSS reset         |           |
        |          |                      |           |
        |          |   Detect GNSS        |           |
        |          |--------------------->|           |
        |          |<---------------------|           |
        |          |   GNSS Detected      |           |
        |          |                      |           |
        |          | Get firmware version |           |
        |          |--------------------->|           |
        |          |<---------------------|           |
        |          | Firmware version ... |           |
        |          |                      |           |
        |          |   Sync GNSS          |           |
        |          |--------------------->|           |
        |          |<---------------------|           |
        |          |   Sync OK            |           |
        |          |                      |           |
        |          |                      |           |
        |          |   Command Packet     |           |
        |          |--------------------->|           |
        |          |   Binary data        |           |
        |          |--------------------->|           |
        |          |<---------------------|           |
        |          |   Command Done       |           |
        |          |<---------------------|           |
        |          |   Binary data        |           |
        |          |                      |           |
        '----------'                      '-----------'
*/

#include <SparkFun_LG290P_GNSS.h> // Click here to get the library: http://librarymanager/All#SparkFun_LG290P

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

// ???
int pin_UART1_TX = 14;
int pin_UART1_RX = 13;
int pin_RESET = -1;
const char * platform = "???";

#endif  // POSTCARD
#endif  // ESP32_RPI_FLEX

//------------------------------------------------------------------------------

uint8_t binaryData[65536 + 256];

char command[8192];
uint16_t commandLength;

int gnss_baud = 460800;
bool gnssDetected;

char line[256];
LG290P myGNSS;
HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32

unsigned long lastCheck = 0;

//----------------------------------------
// Application entry point
//----------------------------------------
void setup()
{
    Serial.begin(115200);
    sprintf(line, "%s\r\n", __FILE__);
    output((uint8_t *)line, strlen(line));
}

//----------------------------------------
// Infinite loop to do the firmware update
//----------------------------------------
void loop()
{
    // Wait for a command from the PC
    if (getCommand())
        processCommand();
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

//----------------------------------------
// Dump the contents of a buffer
//----------------------------------------
void dumpBuffer(size_t offset,
                const uint8_t * buffer,
                size_t length)
{
    size_t bytes;
    const uint8_t *end;
    size_t index;
    char line[132];

    end = &buffer[length];
    while (buffer < end)
    {
        // Determine the number of bytes to display on the line
        bytes = end - buffer;
        if (bytes > (16 - (offset & 0xf)))
            bytes = 16 - (offset & 0xf);

        // Display the offset
        sprintf(line, "0x%08lx: ", offset);

        // Skip leading bytes
        for (index = 0; index < (offset & 0xf); index++)
            strcat(&line[strlen(line)], "   ");

        // Display the data bytes
        for (index = 0; index < bytes; index++)
            sprintf(&line[strlen(line)], "%02x ", buffer[index]);

        // Separate the data bytes from the ASCII
        for (; index < (16 - (offset & 0xf)); index++)
            strcat(&line[strlen(line)], "   ");
        strcat(&line[strlen(line)], " ");

        // Skip leading bytes
        for (index = 0; index < (offset & 0xf); index++)
            strcat(&line[strlen(line)], " ");

        // Display the ASCII values
        for (index = 0; index < bytes; index++)
            sprintf(&line[strlen(line)], "%c", ((buffer[index] < ' ') || (buffer[index] >= 0x7f)) ? '.' : buffer[index]);

        // Output the line
        sprintf(line, "%s\r\n", line);
        output((uint8_t *)line, strlen(line));

        // Set the next line of data
        buffer += bytes;
        offset += bytes;
    }
}

//----------------------------------------
// Get the binary response from the GNSS device
//----------------------------------------
size_t getBinaryResponseFromGnss()
{
    uint8_t * data;
    uint8_t * dataEnd;
    size_t length;

    data = binaryData;
    dataEnd = &data[1];
    while (data < dataEnd)
    {
        // Read the next byte of serial data
        if (SerialGNSS.available())
        {
            *data = SerialGNSS.read();

            // Check for the head byte (0xAA)
            if (*data == 0xaa)
                data++;
        }
    }

    // Read in the class ID, message ID and payload length
    //   0: 0xAA    Head byte
    //   1:   xx    Class ID
    //   2:   xx    Message ID
    //   3:   xx    Payload length (upper 8 bits)
    //   4:   xx    Payload length (lower 8 bits)
    dataEnd = &data[1 + 1 + 2];
    while (data < dataEnd)
    {
        if (SerialGNSS.available())
            *data++ = SerialGNSS.read();
    };

    // Read in the payload
    length = (((uint16_t)binaryData[3]) << 8) | binaryData[4];
    dataEnd = &data[length];
    while (data < dataEnd)
    {
        if (SerialGNSS.available())
            *data++ = SerialGNSS.read();
    }

    // Read in the CRC and the tail
    dataEnd = &data[4 + 1];
    while (data < dataEnd)
    {
        if (SerialGNSS.available())
            *data++ = SerialGNSS.read();
    }

    // Return the binary packet length, head through tail
    return data - binaryData;
}

//----------------------------------------
// Get a binary command from the PC
//----------------------------------------
size_t getBinaryCommandFromPc()
{
    uint8_t * data;
    uint8_t * dataEnd;
    size_t length;

    data = binaryData;
    dataEnd = &data[1];
    while (data < dataEnd)
    {
        // Read the next byte of serial data
        if (Serial.available())
        {
            *data = Serial.read();

            // Check for the head byte (0xAA)
            if (*data == 0xaa)
                data++;
        }
    }

    // Read in the class ID, message ID and payload length
    //   0: 0xAA    Head byte
    //   1:   xx    Class ID
    //   2:   xx    Message ID
    //   3:   xx    Payload length (upper 8 bits)
    //   4:   xx    Payload length (lower 8 bits)
    dataEnd = &data[1 + 1 + 2];
    while (data < dataEnd)
    {
        if (Serial.available())
            *data++ = Serial.read();
    };

    // Read in the payload
    length = (((uint16_t)binaryData[3]) << 8) | binaryData[4];
    dataEnd = &data[length];
    while (data < dataEnd)
    {
        if (Serial.available())
            *data++ = Serial.read();
    }

    // Read in the CRC and the tail
    dataEnd = &data[4 + 1];
    while (data < dataEnd)
    {
        if (Serial.available())
            *data++ = Serial.read();
    }

    // Return the binary packet length, head through tail
    return data - binaryData;
}

//----------------------------------------
// Write data to the GNSS device
//----------------------------------------
void gnssWriteData(const uint8_t * data, size_t length)
{
    const uint8_t * dataEnd;

    dataEnd = &data[length];
    while (data < dataEnd)
    {
        if (SerialGNSS.availableForWrite())
            SerialGNSS.write(*data++);
    }
}

//----------------------------------------
// Synchronize with the bootloader following the reset
//----------------------------------------
void bootloaderSync()
{
    uint32_t currentMsec;
    int offset;
    uint8_t response[4];
    uint32_t startMsec;
    uint8_t word1[4];
    uint8_t word2[4];

    // Construct the word in little endian order
    // Sync word 1: 0x514C1309
    word1[0] = 0x09;
    word1[1] = 0x13;
    word1[2] = 0x4c;
    word1[3] = 0x51;

    // Sync word 2: 0x1203A504
    word2[0] = 0x04;
    word2[1] = 0xa5;
    word2[2] = 0x03;
    word2[3] = 0x12;

    // For the next 500 mSec, send sync word 1
    startMsec = millis();
    do
    {
        currentMsec = millis();
        gnssWriteData(word1, sizeof(word1));

        // Wait for the response
        offset = 0;
        memset(response, 0, sizeof(response));
        while ((offset < 4) && ((millis() - currentMsec) < 20))
        {
            if (SerialGNSS.available())
                response[offset++] = SerialGNSS.read();
        }

        // Retransmit sync word 1 upon timeout
    } while ((offset < 4) && ((millis() - startMsec) < 500));

    // Handle the timeout case
    if (offset == 0)
    {
        strcpy(line, "ERROR: Timeout during sync word 1!\r\n");
        output((uint8_t *)line, strlen(line));
    }

    // Handle the error
    // Response word 1: 0xAAFC3A4D
    else if ((offset != 4)
        || (response[0] != 0x4d)
        || (response[1] != 0x3a)
        || (response[2] != 0xfc)
        || (response[3] != 0xaa))
    {
        sprintf(line, "ERROR: Sync word 1 response failure 0x%02x%02x%02x%02x\r\n",
                      response[3], response[2], response[1], response[0]);
        output((uint8_t *)line, strlen(line));
    }

    // Valid response
    else
    {
        // Send the second SYNC word
        currentMsec = millis();
        gnssWriteData(word2, sizeof(word2));

        // Wait for the response
        offset = 0;
        memset(response, 0, sizeof(response));
        while ((offset < 4) && ((millis() - currentMsec) < 20))
        {
            if (SerialGNSS.available())
                response[offset++] = SerialGNSS.read();
        }

        // Handle the timeout case
        if (offset == 0)
        {
            strcpy(line, "ERROR: Timeout during sync word 1!\r\n");
            output((uint8_t *)line, strlen(line));
        }

        // Handle the error
        // Response word 2: 0x55FD5BA0
        else if ((offset != 4)
            || (response[0] != 0xa0)
            || (response[1] != 0x5b)
            || (response[2] != 0xfd)
            || (response[3] != 0x55))
        {
            sprintf(line, "ERROR: Sync word 2 response failure 0x%02x%02x%02x%02x\r\n",
                          response[3], response[2], response[1], response[0]);
            output((uint8_t *)line, strlen(line));
        }

        // Valid response
        else
        {
            strcpy(line, "Sync OK\r\n");
            output((uint8_t *)line, strlen(line));
        }
    }
}

//----------------------------------------
// Process the commands
//----------------------------------------
void processCommand()
{
    // Wait for a connection
    if (strcmp(command, "Hello Micro") == 0)
        Serial.println("Hello PC\r\n");

    // Reset the GNSS device
    else if (strcmp(command, "Reset GNSS") == 0)
    {
        // Issue the reset
        if (pin_RESET != -1)
        {
            pinMode(pin_RESET, OUTPUT);
            digitalWrite(pin_RESET, 0);
            delay(100);
            digitalWrite(pin_RESET, 1);
            delay(500);
        }
        Serial.println("GNSS reset\r\n");
    }

    // Detect the GNSS device
    else if (strcmp(command, "Detect GNSS") == 0)
    {
        // We must start the serial port before using it in the library
        // Increase buffer size to handle high baud rate streams
        SerialGNSS.setRxBufferSize(1024);
        SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);
        gnssDetected = myGNSS.begin(SerialGNSS, "SFE_LG290P_GNSS_Library", output);
        if (gnssDetected == false)     // Give the serial port over to the library
        {
            gnssDetected = myGNSS.begin(SerialGNSS, "SFE_LG290P_GNSS_Library", output);
            if (gnssDetected == false)     // Give the serial port over to the library
            {
                Serial.println("ERROR: GNSS failed to respond!\r\n");
                return;
            }
        }
        Serial.println("GNSS detected\r\n");
    }

    // Get the firmware version
    else if (strcmp(command, "Get firmware version") == 0)
    {
        int versionMajor;
        int versionMinor;
        int versionCombined;

        // Check firmware version and print info
        myGNSS.getFirmwareVersionMajor(versionMajor);
        myGNSS.getFirmwareVersionMinor(versionMinor);
        myGNSS.getFirmwareVersion(versionCombined); // v2.1 becomes 201
        sprintf(line, "Firmware Version: %d.%d (%d)\r\n", versionMajor, versionMinor, versionCombined);
        output((uint8_t *)line, strlen(line));
    }

    // Write the first sync word
    else if (strcmp(command, "Sync GNSS") == 0)
    {
        // Issue the reset
        if (pin_RESET == -1)
        {
            if (gnssDetected)
                // Attempt to reset using the firmware
                myGNSS.sendCommand("PQTMSRR", "", 1, false);
            else
            {
                strcpy(line, "ERROR: No reset pin specified!\r\n");
                output((uint8_t *)line, strlen(line));
                return;
            }
        }
        else
        {
            // Reset the GNSS device
            pinMode(pin_RESET, OUTPUT);
            digitalWrite(pin_RESET, 0);
            delay(100);
            digitalWrite(pin_RESET, 1);
        }

        // Reestablish the serial connection to the GNSS
        SerialGNSS.end();
        SerialGNSS.setRxBufferSize(1024);
        SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

        // Attempt to synchronize with the boot loader following reset
        bootloaderSync();
    }

    // Receive a bootloader command
    else if (strcmp(command, "Command Packet") == 0)
    {
        // Receive the binary blob that follows
        size_t length = getBinaryCommandFromPc();

        // Send the command to the GNSS device
        gnssWriteData(binaryData, length);

        // Receive the binary respones from the GNSS device
        length = getBinaryResponseFromGnss();

        // Notify the PC of the response
        strcpy(line, "Command Done\r\n");
        output((uint8_t *)line, strlen(line));

        // Send the binary response
        output(binaryData, length);
    }
}

//----------------------------------------
// Get a command from the PC
//----------------------------------------
bool getCommand()
{
    bool gotCommand = false;
    if (Serial)
    {
        // Wait for data from the PC
        while (Serial.available())
        {
            // Read the data from the PC
            char data = Serial.read();

            // The command is complete upon receiving either a CR or LF
            if ((data == '\r') || (data == '\n'))
            {
                gotCommand = (commandLength > 0);
                commandLength = 0;
                break;
            }

            // Add this character to the command buffer
            command[commandLength++] = data;
            command[commandLength] = 0;
        }
    }

    // Notify the caller when the command is available
    return gotCommand;
}
