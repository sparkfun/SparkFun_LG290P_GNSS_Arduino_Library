/*
Firmware_Update.c

  Update the Quectel GNSS firmware

  This program exchanges the following messages between the PC or Raspberry
  Pi system and the microprocessor connected to the Quectel GNSS device.

    .----------.                                             .----------------.
    |    PC    |                                             | MicroProcessor |
    |          |                                             |                |
    |          |                 Hello Micro                 |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |<--------------------------------------------|                |
    |          |                  Hello PC                   |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |                  Sync GNSS                  |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |<--------------------------------------------|                |
    |          |                   Sync OK                   |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |               Command Packet                |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |-------------------------------------------->|                |
    |          |          Query Bootloader Version           |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |                Command Done                 |                |
    |          |<--------------------------------------------|                |
    |          |<--------------------------------------------|                |
    |          |          Query Bootloader Version           |                |
    |          | Message received and executed successfully. |                |
    |          |          Bootloader version: 1.0.1          |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |               Command Packet                |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |-------------------------------------------->|                |
    |          |            Firmware Information             |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |                Command Done                 |                |
    |          |<--------------------------------------------|                |
    |          |<--------------------------------------------|                |
    |          |            Firmware Information             |                |
    |          | Message received and executed successfully. |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |               Command Packet                |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |-------------------------------------------->|                |
    |          |               Erase Firmware                |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |                Command Done                 |                |
    |          |<--------------------------------------------|                |
    |          |<--------------------------------------------|                |
    |          |               Erase Firmware                |                |
    |          | Message received and executed successfully. |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |               Command Packet                |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |-------------------------------------------->|                |
    |          |      Firmware Packet: Packet 0 of 613       |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |                Command Done                 |                |
    |          |<--------------------------------------------|                |
    |          |<--------------------------------------------|                |
    |          |      Firmware Packet: Packet 0 of 613       |                |
    |          | Message received and executed successfully. |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |               Command Packet                |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |-------------------------------------------->|                |
    |          |      Firmware Packet: Packet 1 of 613       |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |                Command Done                 |                |
    |          |<--------------------------------------------|                |
    |          |<--------------------------------------------|                |
    |          |      Firmware Packet: Packet 1 of 613       |                |
    |          | Message received and executed successfully. |                |
    |          |                                             |                |
    |          |                                             |                |
                                    * * *
    |          |                                             |                |
    |          |                                             |                |
    |          |               Command Packet                |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |-------------------------------------------->|                |
    |          |     Firmware Packet: Packet 613 of 613      |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |                Command Done                 |                |
    |          |<--------------------------------------------|                |
    |          |<--------------------------------------------|                |
    |          |     Firmware Packet: Packet 613 of 613      |                |
    |          | Message received and executed successfully. |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |               Command Packet                |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |-------------------------------------------->|                |
    |          |                   Reset                     |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |                Command Done                 |                |
    |          |<--------------------------------------------|                |
    |          |<--------------------------------------------|                |
    |          |                   Reset                     |                |
    |          | Message received and executed successfully. |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |                 Detect GNSS                 |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |<--------------------------------------------|                |
    |          |                GNSS detected                |                |
    |          |                                             |                |
    |          |                                             |                |
    |          |            Get firmware version             |                |
    |    PC    |-------------------------------------------->| MicroProcessor |
    |          |<--------------------------------------------|                |
    |          |         Firmware Version: 1.3 (103)         |                |
    |          |                                             |                |
    |    PC    |                                             | MicroProcessor |
    '----------'                                             '----------------'

    Building on Windows 10 or 11
    * Install CygWin from cygwin.org
        * Select over internet
        * When you get to page that looks like a spreadsheet
            * If necessary expand window to right to expose the search box
                * Search for gcc
                    * For the gcc entry click on the dropdown menu next to SKIP
                    * Select the version to use (typically the latest non-Test)
                    * SKIP should change to NET
                * Repeat search and version selection for
                    * make
                    * unzip
                    * Some usefull utilities:
                        * nano
                        * openssh
    * Open a CygWin shell window
    * git clone https://github.com/sparkfun/SparkFun_RTK_Postcard.git
    * cd SparkFun_RTK_Postcard/Firmware/Old
    * unzip LG290P03AANR01A03S.zip
    * mkdir   -p   ~/Arduino/libraries
    * cd   ~/Arduino/libraries
    * git clone https://github.com/sparkfun/SparkFun_LG290P_GNSS_Arduino_Library.git
    * cd SparkFun_LG290P_GNSS_Arduino_Library/examples/Firmware_Update/PC
    * make

    * Build the ESP32 application ESP32_Firmware_Update found under
      SparkFun_LG290P_GNSS_Arduino_Library/examples/Firmware_Update
    * Upload the ESP32 application into the ESP32

    Checking the firmware version using the CygWin shell window
    * ./Firmware_Update   /dev/ttyS<your COM port here>   <any file here>

    Uploading firmware version 1.03 to the LG290P using the CygWin shell window
    * ./Firmware_Update   --firmware-update-enabled   /dev/ttyS<your COM port here>   ~/SparkFun_RTK_Postcard/Firmware/Old/LG290P03AANR01A03S/LG290P03AANR01A03S.pkg

    Note: Wait until the firmware update is complete and displays the new
          firmware version.  Avoid losing power or resetting the firmware
          update while in progress!
*/

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

//----------------------------------------
// New types
//----------------------------------------

typedef struct _BAUDRATE_OPTION
{
    int _baudrate;
    speed_t _bValue;
} BAUDRATE_OPTION;

typedef struct _COMMAND_OPTION
{
    bool _displayHandshakeDiagram;
    bool _getBaudRate;
    bool * _optionBoolean;
    const char * _optionString;
    const char * _helpText;
} COMMAND_OPTION;

//----------------------------------------
// Constants
//----------------------------------------

#define MAX_PACKET_SIZE             (5 * 1024)

#define BAIL_WITH_SUCCESS           0x8000000

// Microprocessor GNSS firmware upload states
enum MICROPROCESSOR_FIRMWARE_UPLOAD_STATES
{
    FUS_HELLO = 0,
    FUS_RESET,
    FUS_DETECT_GNSS,
    FUS_FIRMWARE_VERSION,
    FUS_SYNC,
    FUS_BOOT_VERSION_1,
    FUS_BOOT_VERSION_2,
    FUS_FIRMWARE_INFO_1,
    FUS_FIRMWARE_INFO_2,
    FUS_FIRMWARE_ERASE_1,
    FUS_FIRMWARE_ERASE_2,
    FUS_FIRMWARE_UPLOAD_1,
    FUS_FIRMWARE_UPLOAD_2,
    FUS_FIRMWARE_RESET_1,
    FUS_FIRMWARE_RESET_2,
    // Add new states above this line
    FUS_MAX
};

enum DIRECT_CONNECT_FIRMWARE_UPLOAD_STATES
{
    DCFUS_FIRMWARE_VERSION = 0,
    DCFUS_RESET,
    DCFUS_POWER_ON,
    DCFUS_SYNC,
    DCFUS_BOOT_VERSION,
    DCFUS_FIRMWARE_INFO,
    DCFUS_FIRMWARE_ERASE,
    DCFUS_FIRMWARE_UPLOAD,
    DCFUS_FIRMWARE_RESET,

    // Add new states above this line
    DCFUS_MAX
};

#define nullptr                 NULL

const char * dashes = "---------------------------------------------";
const char * leftArrow = "<";
const char * rightArrow = ">";
const char * microprocessor       = "|                |";
const char * microprocessorBottom = "'----------------'";
const char * microprocessorLabel  = "| MicroProcessor |";
const char * microprocessorTop    = ".----------------.";
const char * pc       = "|          |";
const char * pcBottom = "'----------'";
const char * pcLabel  = "|    PC    |";
const char * pcTop    = ".----------.";
const char * spaces = "                                                                                ";

const BAUDRATE_OPTION baudrateTable[] =
{
    {50, B50},
    {110, B110},
    {134, B134},
    {150, B150},
    {200, B200},
    {300, B300},
    {600, B600},
    {1200, B1200},
    {1800, B1800},
    {2400, B2400},
    {4800, B4800},
    {9600, B9600},
    {19200, B19200},
    {38400, B38400},
#ifdef B57600
    {57600, B57600},
#endif
#ifdef B76800
    {76800, B76800},
#endif
#ifdef B115200
    {115200, B115200},
#endif
#ifdef B153600
    {153600, B153600},
#endif
#ifdef B230400
    {230400, B230400},
#endif
#ifdef B307200
    {307200, B307200},
#endif
#ifdef B460800
    {460800, B460800},
#endif
#ifdef B500000
    {500000, B500000},
#endif
#ifdef B576000
    {576000, B576000},
#endif
#ifdef B614400
    {614400, B614400},
#endif
#ifdef B921600
    {921600, B921600},
#endif
#ifdef B1000000
    {1000000, B1000000},
#endif
#ifdef B1152000
    {1152000, B1152000},
#endif
#ifdef B1500000
    {1500000, B1500000},
#endif
#ifdef B2000000
    {2000000, B2000000},
#endif
#ifdef B2500000
    {2500000, B2500000},
#endif
#ifdef B3000000
    {3000000, B3000000},
#endif
#ifdef B3500000
    {3500000, B3500000},
#endif
#ifdef B4000000
    {4000000, B4000000},
#endif
};
const int baudrateTableEntries = sizeof(baudrateTable) / sizeof(baudrateTable[0]);

const uint32_t crc32_table[256] =
{
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba,
    0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de,
    0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,
    0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940,
    0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116,
    0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a,
    0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818,
    0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c,
    0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2,
    0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086,
    0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4,
    0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8,
    0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe,
    0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252,
    0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60,
    0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04,
    0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a,
    0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e,
    0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c,
    0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0,
    0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6,
    0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

//----------------------------------------
// Globals
//----------------------------------------

speed_t baudrate = B460800;
bool baudrateSet;
size_t commandResponseLength;
int comPort;
bool displayBinaryCommand;
bool displayBinaryCommandSummary;
bool displayBinaryResponse;
bool displayBytesReceived;
bool displayCommand;
bool displayCommandResponse;
bool displayHandshakeDiagram;
bool eraseOnly;
int firmware;
uint8_t firmwareBuffer[MAX_PACKET_SIZE];
uint32_t firmwareCrc32;
off_t firmwareLength;
uint8_t * firmwarePackage;
bool firmwareUpdateEnabled;
bool firmwareVersionOnly;
const char * helloMicro = "Hello Micro";
int32_t packetCount;
int32_t packetNumber;
uint32_t pollTimeoutUsec;
uint8_t response[8192];
size_t responseLength;
bool skipVersionCheck;
int state;
int timeoutCount;
char * timeoutMessage;
bool useMicroprocessor;
bool waitForUart;

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
        printf("%s\r\n", line);

        // Set the next line of data
        buffer += bytes;
        offset += bytes;
    }
}

//----------------------------------------
// Configure the serial port talking with the microprocessor
//----------------------------------------
int configureComPort(speed_t baudRate)
{
    struct termios options;
    int status;

    do
    {
        // Get the COM Port settings
        status = tcgetattr(comPort, &options);
        if (status < 0)
        {
            status = errno;
            perror("ERROR: Failed to get COM Port settings\r\n");
            break;
        }

        // Set the baud rates
        cfsetispeed(&options, baudRate);
        cfsetospeed(&options, baudRate);

        // 8 bits, no parity, one stop bit
        options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;

        // Disable hardware flow control (CRTSCTS)
        options.c_cflag &= ~CRTSCTS;

        // Disable software flow control (IXON, IXOFF, IXANY)
        options.c_iflag &= ~(IXON | IXOFF | IXANY);

        options.c_cflag |= CREAD | CLOCAL;

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        options.c_oflag &= ~(OPOST | ONLCR);

        // Update the COM Port settings
        status = tcsetattr(comPort, TCSANOW, &options);
        if (status < 0)
        {
            status = errno;
            perror("ERROR: Failed to set COM Port settings\r\n");
            break;
        }
        status = 0;
    } while (0);

    // Return the configuration status
    return status;
}

//----------------------------------------
// Write some data to the microprocessor
//----------------------------------------
int writeData(const uint8_t * command, ssize_t length)
{
    size_t bytes;
    const size_t maxBytes = 4096;
    errno = 0;

    // Ensure that all of the data is sent to the microprocessor
    while (length)
    {
        // Send some data to the microprocessor
        bytes = length;
        if (bytes > maxBytes)
            bytes = maxBytes;
        int bytesWritten = write(comPort, command, bytes);

        // Handle write errors
        if (bytesWritten < 0)
            break;

        // Account for the data sent
        length -= bytesWritten;
        command += bytesWritten;
    }
    return errno;
}

//----------------------------------------
// Add a command to the handshake diagram
//----------------------------------------
void addCommandToHandshakeDiagram(const char * command)
{
    size_t length;
    size_t spacesAfter;
    size_t spacesBefore;

    // Display the command
    length = strlen(command);
    spacesAfter = strlen(dashes) - 1 - length - 1;
    spacesBefore = spacesAfter / 2;
    spacesAfter -= spacesBefore;
    spacesBefore = strlen(spaces) - spacesBefore;
    spacesAfter = strlen(spaces) - spacesAfter;
    printf("%s%s %s %s%s\r\n",
           pc,
           &spaces[spacesBefore],
           command,
           &spaces[spacesAfter],
           microprocessor);

    // Display the arrow
    printf("%s%s%s%s\r\n", pcLabel, &dashes[1], rightArrow, microprocessorLabel);
}

//----------------------------------------
// Send a command to the microprocessor
//----------------------------------------
int writeCommand(const char * command)
{
    // Display the command
    if (displayCommand)
        addCommandToHandshakeDiagram(command);

    // Send the command to the microprocessor
    if (writeData((const uint8_t *)command, strlen(command)))
        return errno;
    return writeData((const uint8_t *)"\r\n", 2);
}

//----------------------------------------
// Add a response to the handshake diagram
//----------------------------------------
void addResponseToHandshakeDiagram(const char * responseString)
{
    bool displayLabel;
    size_t length;
    size_t spacesAfter;
    size_t spacesBefore;

    // Display the arrow
    printf("%s%s%s%s\r\n", pc, leftArrow, &dashes[1], microprocessor);

    // Display the response
    displayLabel = ! (displayCommand | displayBinaryCommand);
    length = strlen(responseString);
    spacesAfter = strlen(dashes) - 1 - length - 1;
    spacesBefore = spacesAfter / 2;
    spacesAfter -= spacesBefore;
    spacesBefore = strlen(spaces) - spacesBefore;
    spacesAfter = strlen(spaces) - spacesAfter;
    printf("%s%s %s %s%s\r\n",
           displayLabel ? pcLabel : pc,
           &spaces[spacesBefore],
           responseString,
           &spaces[spacesAfter],
           displayLabel ? microprocessorLabel : microprocessor);

    // Display a couple of blank lines
    printf("%s%s%s\r\n", pc, &spaces[strlen(spaces) - strlen(dashes)], microprocessor);
    printf("%s%s%s\r\n", pc, &spaces[strlen(spaces) - strlen(dashes)], microprocessor);
}

//----------------------------------------
// Read a response from the microprocessor
//----------------------------------------
bool getResponse()
{
    char data;
    bool gotResponse;

    errno = 0;
    gotResponse = false;
    do
    {
        // Read data from the microprocessor
        ssize_t bytesRead = read(comPort, &response[responseLength], 1);

        // Handle the errors
        if (bytesRead <= 0)
            break;

        // Display the byte
        if (displayBytesReceived)
            printf("0x%02x\r\n", response[responseLength]);

        // Done when CR or LF received
        if ((response[responseLength] == '\r') || (response[responseLength] == '\n'))
        {
            gotResponse = (responseLength != 0);
            break;
        }

        // Buffer the response
        responseLength += 1;
    } while (0);

    // Zero terminate the string
    response[responseLength] = 0;

    // Start a new response if necessary
    if (gotResponse)
    {
        responseLength = 0;

        // Display the response
        if (displayCommandResponse)
        {
            if (strcmp((char *)response, "Command Done") == 0)
            {
                size_t length;
                size_t spacesAfter;
                size_t spacesBefore;

                // Display the response
                length = strlen((char *)response);
                spacesAfter = strlen(dashes) - 1 - length - 1;
                spacesBefore = spacesAfter / 2;
                spacesAfter -= spacesBefore;
                spacesBefore = strlen(spaces) - spacesBefore;
                spacesAfter = strlen(spaces) - spacesAfter;
                printf("%s%s %s %s%s\r\n",
                       pc,
                       &spaces[spacesBefore],
                       response,
                       &spaces[spacesAfter],
                       microprocessor);

                // Display the arrow
                printf("%s%s%s%s\r\n", pc, leftArrow, &dashes[1], microprocessor);
            }
            else
                addResponseToHandshakeDiagram((char *)response);
        }
    }

    // Tell the caller of the response
    return gotResponse;
}

//----------------------------------------
// Read a NMEA response from the GNSS device
//----------------------------------------
bool getNmeaResponse()
{
    char data;
    bool gotResponse;

    errno = 0;
    gotResponse = false;
    do
    {
        // Read data from the microprocessor
        ssize_t bytesRead = read(comPort, &response[responseLength], 1);

        // Handle the errors
        if (bytesRead <= 0)
            break;

        // Display the byte
        if (displayBytesReceived)
            printf("0x%02x\r\n", response[responseLength]);

        // NMEA sentence starts with a dollar sign ($)
        if ((responseLength == 0) && (response[0] != '$'))
            // Ignore this character
            break;

        // Done when CR or LF received
        if ((response[responseLength] == '\r') || (response[responseLength] == '\n'))
        {
            gotResponse = (responseLength != 0);
            break;
        }

        // Buffer the response
        responseLength += 1;
    } while (0);

    // Zero terminate the string
    response[responseLength] = 0;

    // Start a new response if necessary
    if (gotResponse)
    {
        responseLength = 0;

        // Display the response
        if (displayCommandResponse)
        {
            if (strcmp((char *)response, "Command Done") == 0)
            {
                size_t length;
                size_t spacesAfter;
                size_t spacesBefore;

                // Display the response
                length = strlen((char *)response);
                spacesAfter = strlen(dashes) - 1 - length - 1;
                spacesBefore = spacesAfter / 2;
                spacesAfter -= spacesBefore;
                spacesBefore = strlen(spaces) - spacesBefore;
                spacesAfter = strlen(spaces) - spacesAfter;
                printf("%s%s %s %s%s\r\n",
                       pc,
                       &spaces[spacesBefore],
                       response,
                       &spaces[spacesAfter],
                       microprocessor);

                // Display the arrow
                printf("%s%s%s%s\r\n", pc, leftArrow, &dashes[1], microprocessor);
            }
            else
                addResponseToHandshakeDiagram((char *)response);
        }
    }

    // Tell the caller of the response
    return gotResponse;
}

//----------------------------------------
// Extract the status from the command response
//----------------------------------------
uint16_t getCommandStatus(const uint8_t * data, const char ** message)
{
    uint16_t status;

    // Get the command status value
    status = (((uint16_t)data[0]) << 8) | data[1];

    // Return the status message
    if (message)
    {
        switch(status)
        {
            default:
                *message = "Unknown status value";
                break;

            case 0:
                *message = "Message received and executed successfully.";
                break;

            case 1:
                *message = "Unknow error.";
                break;

            case 2:
                *message = "CRC32 checksum error.";
                break;

            case 3:
                *message = "Timeout.";
                break;

            case 4:
                *message = "Unsupported message.";
                break;

            case 5:
                *message = "Message package error.";
                break;

            case 0x20:
                *message = "Firmware area erase error.";
                break;

            case 0x21:
                *message = "Firmware write to Flash error.";
                break;
        }
    }
    return status;
}

//----------------------------------------
// Get the binary command name
//----------------------------------------
const char * getBinaryCommandName(const uint8_t * data)
{
    const char * commandName;

    // Get the command name
    if ((data[0] == 2) && (data[1] == 0))
        commandName = "Query Bootloader Version";
    else if ((data[0] == 2) && (data[1] == 2))
        commandName = "Firmware Information";
    else if ((data[0] == 2) && (data[1] == 3))
        commandName = "Erase Firmware";
    else if ((data[0] == 2) && (data[1] == 4))
        commandName = "Firmware Packet";
    else if ((data[0] == 2) && (data[1] == 0x31))
        commandName = "Reset";
    else if ((data[0] == 2) && (data[1] == 0x71))
        commandName = "Query Bootloader Version";
    else
        commandName = "Unknown command";
    return commandName;
}

//----------------------------------------
// Read a command response from the microprocessor
//----------------------------------------
bool getCommandResponse()
{
    char data;
    bool displayLabel;
    bool gotResponse;
    char line[64];
    size_t messageBytes;
    uint16_t packetLength;

    errno = 0;
    gotResponse = false;
    do
    {
        // Read data from the microprocessor
        ssize_t bytesRead = read(comPort, &response[responseLength], 1);

        // Handle the errors
        if (bytesRead <= 0)
            break;

        // Display the byte
        if (displayBytesReceived)
            printf("0x%02x\r\n", response[responseLength]);

        // Wait for 0xAA to start the binary response
        if ((responseLength == 0) && (response[0] != 0xaa))
            break;

        // Buffer the response
        responseLength += 1;

        // Ignore the class and message ID values and upper portion of length
        if (responseLength <= 4)
            break;

        // Get the payload length
        packetLength = (((uint16_t)response[3]) << 8) | response[4];
        messageBytes = 1    // Head
                     + 1    // Class ID
                     + 1    // Message ID
                     + packetLength
                     + 4;   // CRC32

        // Skip over the payload and CRC
        if (responseLength <= messageBytes)
            break;

        // Done when 0x55 is in the buffer
        if (response[responseLength - 1] == 0x55)
            gotResponse = true;
    } while (0);

    // Start a new response if necessary
    if (gotResponse)
    {
        commandResponseLength = responseLength;
        responseLength = 0;

        // Display the response
        if (displayHandshakeDiagram)
        {
            const char * commandName;
            size_t length;
            const char * message;
            size_t spacesAfter;
            size_t spacesBefore;

            // Display the arrow
            displayLabel = ! (displayCommand
                              | displayCommandResponse
                              | displayBinaryCommand
                              | displayBinaryCommandSummary);
            printf("%s%s%s%s\r\n",
                   displayLabel ? pcLabel : pc,
                   leftArrow,
                   &dashes[1],
                   displayLabel ? microprocessorLabel : microprocessor);

            // Display the response
            commandName = getBinaryCommandName(&response[1 + 1 + 1 + 2]);
            if (packetNumber < 0)
                length = strlen(commandName);
            else
            {
                sprintf(line, "%s: Packet %d of %d", commandName, packetNumber, packetCount - 1);
                length = strlen(line);
            }
            spacesAfter = strlen(dashes) - 1 - length - 1;
            spacesBefore = spacesAfter / 2;
            spacesAfter -= spacesBefore;
            spacesBefore = strlen(spaces) - spacesBefore;
            spacesAfter = strlen(spaces) - spacesAfter;
            printf("%s%s %s %s%s\r\n",
                   pc,
                   &spaces[spacesBefore],
                   (packetNumber < 0) ? commandName : line,
                   &spaces[spacesAfter],
                   microprocessor);

            // Display the status
            getCommandStatus(&response[7], &message);
            length = strlen(message);
            spacesAfter = strlen(dashes) - 1 - length - 1;
            spacesBefore = spacesAfter / 2;
            spacesAfter -= spacesBefore;
            spacesBefore = strlen(spaces) - spacesBefore;
            spacesAfter = strlen(spaces) - spacesAfter;
            printf("%s%s %s %s%s\r\n",
                   pc,
                   &spaces[spacesBefore],
                   message,
                   &spaces[spacesAfter],
                   microprocessor);

            // Display the bootloader version
            if ((response[5] == 2) && (response[6] == 0x71))
            {
                char line[64];

                sprintf(line, "Bootloader version: %d.%d.%d", response[9], response[10], response[11]);
                length = strlen(line);
                spacesAfter = strlen(dashes) - 1 - length - 1;
                spacesBefore = spacesAfter / 2;
                spacesAfter -= spacesBefore;
                spacesBefore = strlen(spaces) - spacesBefore;
                spacesAfter = strlen(spaces) - spacesAfter;
                printf("%s%s %s %s%s\r\n",
                       pc,
                       &spaces[spacesBefore],
                       line,
                       &spaces[spacesAfter],
                       microprocessor);
            }

            // Display the binary data
            if (displayBinaryResponse)
                dumpBuffer(0, response, commandResponseLength);

            // Display a couple of blank lines
            printf("%s%s%s\r\n", pc, &spaces[strlen(spaces) - strlen(dashes)], microprocessor);
            printf("%s%s%s\r\n", pc, &spaces[strlen(spaces) - strlen(dashes)], microprocessor);
        }
    }

    // Tell the caller of the response
    return gotResponse;
}

//----------------------------------------
// Compute the CRC32 for the specified data region
//----------------------------------------

uint32_t computeCrc32(uint32_t initialValue, const uint8_t * data, size_t length)
{
    uint32_t crc;
    const uint8_t * end;

    // Compute the CRC for the data buffer
    crc = initialValue ^ 0xffffffff;
    end = &data[length];
    while (data < end)
        crc = crc32_table[(crc ^ *data++) & 0xFF] ^ (crc >> 8);
    crc ^= 0xffffffff;
    return crc;
}

//----------------------------------------
// Write a 32-bit value in big endian format
//----------------------------------------
void insertBigEndian(uint32_t value, uint8_t * data)
{
    data[0] = (value >> 24) & 0xff;
    data[1] = (value >> 16) & 0xff;
    data[2] = (value >> 8) & 0xff;
    data[3] = value & 0xff;
}

//----------------------------------------
// Write a 32-bit value in little endian format
//----------------------------------------
void insertLittleEndian(uint32_t value, uint8_t * data)
{
    data[0] = value & 0xff;
    data[1] = (value >> 8) & 0xff;
    data[2] = (value >> 16) & 0xff;
    data[3] = (value >> 24) & 0xff;
}

//----------------------------------------
// Reset the GNSS device and exchange synchronation words with the bootloader
//----------------------------------------
int resetAndSync()
{
    timeoutMessage = "ERROR: Timeout during GNSS synchronization!\r\n";
    pollTimeoutUsec = 2 * 1000 * 1000;
    state = FUS_SYNC;
    return writeCommand("Sync GNSS");
}

//----------------------------------------
// Display the binary command
//----------------------------------------
void addBinaryCommandToHandshakeDiagram(const uint8_t * command, size_t commandLength)
{
    const char * commandName;
    bool displayLabel;
    size_t length;
    char line[64];
    size_t spacesAfter;
    size_t spacesBefore;
    const size_t summaryLength = 0x10;
    size_t tailLength;
    size_t tailOffset;

    // Display the arrow
    displayLabel = ! displayCommand;
    if (useMicroprocessor)
        printf("%s%s%s%s\r\n",
               displayLabel ? pcLabel : pc,
               &dashes[1],
               rightArrow,
               displayLabel ? microprocessorLabel : microprocessor);

    // Get the command name
    commandName = getBinaryCommandName(&command[1]);

    // Display the command name
    if (packetNumber < 0)
        length = strlen(commandName);
    else
    {
        sprintf(line, "%s: Packet %d of %d", commandName, packetNumber, packetCount - 1);
        length = strlen(line);
    }
    spacesAfter = strlen(dashes) - 1 - length - 1;
    spacesBefore = spacesAfter / 2;
    spacesAfter -= spacesBefore;
    spacesBefore = strlen(spaces) - spacesBefore;
    spacesAfter = strlen(spaces) - spacesAfter;
    printf("%s%s %s %s%s\r\n",
           pc,
           &spaces[spacesBefore],
           (packetNumber < 0) ? commandName : line,
           &spaces[spacesAfter],
           microprocessor);

    if (displayBinaryCommand)
    {
        // Determine how much of the binary command to display
        if (displayBinaryCommandSummary && (command[2] == 4)
            && (commandLength > (summaryLength * 2)))
        {
            // Display a summary of the command
            dumpBuffer(0, command, summaryLength);

            // Display the command tail
            tailLength = commandLength & (summaryLength - 1);
            tailOffset = commandLength - tailLength;
            dumpBuffer(tailOffset, &command[tailOffset], tailLength);
        }
        else
            // Display the entire command
            dumpBuffer(0, command, commandLength);
    }

    // Display the arrow
    if (useMicroprocessor == false)
        printf("%s%s%s%s\r\n",
               displayLabel ? pcLabel : pc,
               &dashes[1],
               rightArrow,
               displayLabel ? microprocessorLabel : microprocessor);
    else
    {
        // Display a couple of blank lines
        printf("%s%s%s\r\n", pc, &spaces[strlen(spaces) - strlen(dashes)], microprocessor);
        printf("%s%s%s\r\n", pc, &spaces[strlen(spaces) - strlen(dashes)], microprocessor);
    }
}

//----------------------------------------
// Send the boot firmware version command to the microprocessor
//----------------------------------------
uint32_t getBootLoaderVersion()
{
    uint8_t command[10];
    uint32_t crc32;

    // Construct the command
    command[0] = 0xaa;
    command[1] = 2;
    command[2] = 0x71;
    command[3] = 0;
    command[4] = 0;
    command[9] = 0x55;

    // Compute the CRC
    crc32 = computeCrc32(0, &command[1], 4);
    insertBigEndian(crc32, &command[5]);

    // Display the command
    if (displayHandshakeDiagram)
        addBinaryCommandToHandshakeDiagram(command, sizeof(command));

    // Send the command to the microprocessor
    return writeData(command, sizeof(command));
}

//----------------------------------------
// Send the firmware erase command to the microprocessor
//----------------------------------------
uint32_t sendFirmwareErase()
{
    uint8_t command[10];
    uint32_t crc32;

    // Construct the command
    command[0] = 0xaa;      // Head
    command[1] = 2;         // Class ID
    command[2] = 3;         // Message ID
    command[3] = 0;         // Payload length (big endian)
    command[4] = 0;
    command[9] = 0x55;      // Tail

    // Compute the CRC
    crc32 = computeCrc32(0, &command[1], 4);
    insertBigEndian(crc32, &command[5]);

    // Display the command
    if (displayHandshakeDiagram)
        addBinaryCommandToHandshakeDiagram(command, sizeof(command));

    // Send the command to the microprocessor
    return writeData(command, sizeof(command));
}

//----------------------------------------
// Send the firmware infomation command to the microprocessor
//----------------------------------------
uint32_t sendFirmwareInfo()
{
    uint8_t command[26];
    uint32_t crc32;

    // Construct the command
    command[0] = 0xaa;  // Head
    command[1] = 2;     // Class ID
    command[2] = 2;     // Message ID
    command[3] = 0;     // Payload length (big endian)
    command[4] = 0x10;
    command[25] = 0x55; // Tail

    // Payload
    insertBigEndian(firmwareLength, &command[5]);   // Firmware length in bytes
    insertBigEndian(firmwareCrc32, &command[9]);    // Firmware CRC
    insertBigEndian(0, &command[13]);               // Base address
    insertBigEndian(0, &command[17]);               // Reversed

    // Compute the CRC
    crc32 = computeCrc32(0, &command[1], sizeof(command) - 1 - 4 - 1);
    insertBigEndian(crc32, &command[21]);

    // Display the command
    if (displayHandshakeDiagram)
        addBinaryCommandToHandshakeDiagram(command, sizeof(command));

    // Send the command to the microprocessor
    return writeData(command, sizeof(command));
}

//----------------------------------------
// Send a firmware packet to the microprocessor
//----------------------------------------
uint32_t sendFirmwarePacket()
{
    static uint8_t command[1 + 1 + 1 + 2 + 4 + MAX_PACKET_SIZE + 4 + 1];
    size_t commandLength;
    uint32_t crc32;
    uint8_t * firmwarePayload;
    size_t lengthFirmware;
    size_t lengthPayload;
    size_t offset;

    // Determine the payload length
    offset = packetNumber * MAX_PACKET_SIZE;
    firmwarePayload = &firmwarePackage[offset];
    lengthFirmware = firmwareLength - offset;
    if (lengthFirmware > MAX_PACKET_SIZE)
        lengthFirmware = MAX_PACKET_SIZE;
    lengthPayload = 4 + lengthFirmware;

    // Construct the command
    command[0] = 0xaa;      // Head
    command[1] = 2;         // Class ID
    command[2] = 4;         // Message ID
    command[3] = lengthPayload >> 8;        // Payload length (big endian)
    command[4] = lengthPayload & 0xff;
    command[5 + lengthPayload + 4] = 0x55;  // Tail

    // Add the payload
    insertBigEndian(packetNumber, &command[5]);             // Packet number (big endian)
    memcpy(&command[9], firmwarePayload, lengthFirmware);   // Next block of firmware data

    // Compute the CRC
    commandLength = 1 + 1 + 1 + 2 + lengthPayload + 4 + 1;
    crc32 = computeCrc32(0, &command[1], commandLength - 1 - 4 - 1);
    insertBigEndian(crc32, &command[commandLength - 4 - 1]);

    // Display the command
    if (displayHandshakeDiagram)
        addBinaryCommandToHandshakeDiagram(command, commandLength);

    // Send the command to the microprocessor
    return writeData(command, commandLength);
}

//----------------------------------------
// Send the firmware reset command to the microprocessor
//----------------------------------------
uint32_t sendFirmwareReset()
{
    uint8_t command[10];
    uint32_t crc32;

    // Construct the command
    command[0] = 0xaa;      // Head
    command[1] = 2;         // Class ID
    command[2] = 0x31;      // Message ID
    command[3] = 0;         // Payload length (big endian)
    command[4] = 0;
    command[9] = 0x55;      // Tail

    // Compute the CRC
    crc32 = computeCrc32(0, &command[1], 4);
    insertBigEndian(crc32, &command[5]);

    // Display the command
    if (displayHandshakeDiagram)
        addBinaryCommandToHandshakeDiagram(command, sizeof(command));

    // Send the command to the microprocessor
    return writeData(command, sizeof(command));
}

//----------------------------------------
// Upload the firmware image using a microprocessor
//
//      PC <---> Microprocessor <---> GNSS
//----------------------------------------
int uploadFirmware(bool timeout)
{
    static bool binaryResponse;
    uint16_t commandStatus;
    uint32_t crc;
    bool displayResponseSummary;
    int exitStatus;
    const char * firmwareVersionResponse = "Firmware Version: ";
    bool gotResponse;
    int length;
    const char * message;
    bool printResponse;
    bool responseType;

    // Handle timeouts
    if (timeout && timeoutMessage && (state < FUS_MAX))
    {
        printf("%s\r\n", timeoutMessage);
        timeoutMessage = nullptr;
        return -1;
    }

    // Determine if response summaries should be displayed
    displayResponseSummary = (displayHandshakeDiagram == false)
                             || ((displayHandshakeDiagram == true)
                                && (displayBinaryCommand == false)
                                && (displayBinaryCommandSummary == false)
                                && (displayBinaryResponse == false)
                                && displayCommand
                                && (displayCommandResponse == false));

    // Get the response
    exitStatus = 0;
    gotResponse = false;
    responseType = false;
    if (timeout == false)
    {
        // Use responseType to selet type of display for response
        responseType = binaryResponse;

        // Get the expected type of response
        if (binaryResponse)
            gotResponse = getCommandResponse();
        else
            gotResponse = getResponse();

        // Done getting binary command responses
        if (gotResponse)
            binaryResponse = false;

        // Handle the any response errors
        exitStatus = errno;
        if (exitStatus)
            return exitStatus;
    }

    // Process the state
    printResponse = gotResponse;
    switch (state)
    {
        // Default case for development
        default:
            // Display the error
            printf("ERROR: Unknown state %d\r\n", state);
            if (timeout)
                printf("Timeout!\r\n");
            exitStatus = -1;
            timeoutMessage = nullptr;
            break;

        // Establish a connection to the microprocessor
        case FUS_HELLO:
            if (timeout)
                exitStatus = writeCommand(helloMicro);

            // Determine if the microprocessor is connected
            if (gotResponse && strcmp((char *)response, "Hello PC") == 0)
            {
                if (skipVersionCheck)
                {
                    // Reset the GNSS and perform SYNC handshake
                    printResponse = false;
                    exitStatus = resetAndSync();
                }
                else
                {
                    // Send the reset command
                    printResponse = false;
                    exitStatus = writeCommand("Reset GNSS");
                    timeoutMessage = "ERROR: Timeout during GNSS reset!\r\n";
                    pollTimeoutUsec = 20 * 1000 * 1000;
                    state = FUS_RESET;
                }
            }
            break;

        // Wait for the reset to complete
        case FUS_RESET:
            // Determine if the microprocessor is reset
            if (gotResponse && strcmp((char *)response, "GNSS reset") == 0)
            {
                // Send the command to detect the GNSS device
                printResponse = false;
                exitStatus = writeCommand("Detect GNSS");
                timeoutMessage = "ERROR: Timeout during GNSS detection!\r\n";
                state = FUS_DETECT_GNSS;
            }
            break;

        // Wait to detect the GNSS device
        case FUS_DETECT_GNSS:
            // Determine if the GNSS is detected
            if (gotResponse && strcmp((char *)response, "GNSS detected") == 0)
            {
                // Send the command to detect the GNSS device
                printResponse = false;
                exitStatus = writeCommand("Get firmware version");
                timeoutMessage = "ERROR: Timeout while getting GNSS firmware version!\r\n";
                state = FUS_FIRMWARE_VERSION;
            }
            break;

        // Wait for the firmware version
        case FUS_FIRMWARE_VERSION:

            // Determine if the firmware version is available
            length = strlen(firmwareVersionResponse);
            if (gotResponse && strncmp((char *)response, firmwareVersionResponse, length) == 0)
            {
                // Done after displaying version number
                if (firmwareVersionOnly)
                {
                    exitStatus = BAIL_WITH_SUCCESS;
                    break;
                }

                // Reset the GNSS
                printResponse = displayResponseSummary;
                exitStatus = resetAndSync();
            }
            break;

        // Wait for the sync response
        case FUS_SYNC:
            // Determine if the GNSS received sync word 1
            if (gotResponse && strcmp((char *)response, "Sync OK") == 0)
            {
                // Send the boot version command
                printResponse = false;
                exitStatus = writeCommand("Command Packet");
                if (!exitStatus)
                    exitStatus = getBootLoaderVersion();
                timeoutMessage = "ERROR: Timeout sending bootloader version command!\r\n";
                state = FUS_BOOT_VERSION_1;
            }
            break;

        // Wait for the boot version command to complete
        case FUS_BOOT_VERSION_1:
            if (gotResponse && strcmp((char *)response, "Command Done") == 0)
            {
                // Get the boot loader response
                printResponse = false;
                binaryResponse = true;
                timeoutMessage = "ERROR: Timeout waiting for bootloader version response!\r\n";
                state = FUS_BOOT_VERSION_2;
            }
            break;

        // Wait for the boot loader version packet response
        case FUS_BOOT_VERSION_2:
            if (gotResponse)
            {
                // Process the response
                crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

                // Check for an error
                if ((response[0] != 0xaa)       // Head
                    || (response[1] != 2)       // Class ID
                    || (response[2] != 0)       // Message ID
                    || (response[3] != 0)       // Payload length (big endian)
                    || (response[4] != 7)
                    || (response[5] != 2)       // Class ID
                    || (response[6] != 0x71)    // Message ID
                    || (response[12] != ((crc >> 24) & 0xff))   // CRC (big endian)
                    || (response[13] != ((crc >> 16) & 0xff))
                    || (response[14] != ((crc >> 8) & 0xff))
                    || (response[15] != (crc & 0xff))
                    || (response[16] != 0x55))  // Tail
                {
                    dumpBuffer(0, response, commandResponseLength);
                    printf("ERROR: Boot loader version error\r\n");
                    exitStatus = -1;
                    break;
                }
                printResponse = false;

                // Verify the command status
                commandStatus = getCommandStatus(&response[7], &message);
                if (commandStatus)
                {
                    printf("ERROR: Boot loader version command returned: %s\r\n", message);
                    exitStatus = -1;
                    break;
                }

                // Display the boot loader version
                if (displayResponseSummary)
                    printf("Bootloader version: %d.%d.%d\r\n", response[9], response[10], response[11]);

                // Determine if firmware updates are enabled
                if (firmwareUpdateEnabled == false)
                {
                    exitStatus = BAIL_WITH_SUCCESS;
                    break;
                }

                // Send the firmware information
                exitStatus = writeCommand("Command Packet");
                if (!exitStatus)
                    exitStatus = sendFirmwareInfo();
                state = FUS_FIRMWARE_INFO_1;
            }
            break;

        // Wait for the firmware information command to complete
        case FUS_FIRMWARE_INFO_1:
            if (gotResponse && strcmp((char *)response, "Command Done") == 0)
            {
                // Get the boot loader response
                printResponse = false;
                binaryResponse = true;
                timeoutMessage = "ERROR: Timeout waiting for bootloader version response!\r\n";
                state = FUS_FIRMWARE_INFO_2;
            }
            break;

        // Wait for the firmware information response
        case FUS_FIRMWARE_INFO_2:
            if (gotResponse)
            {
                // Process the response
                crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

                // Check for an error
                if ((response[0] != 0xaa)       // Head
                    || (response[1] != 2)       // Class ID
                    || (response[2] != 0)       // Message ID
                    || (response[3] != 0)       // Payload length (big endian)
                    || (response[4] != 4)
                    || (response[5] != 2)       // Class ID
                    || ((response[6] != 2) && (response[6] != 0)) // Message ID
                    || (response[9] != ((crc >> 24) & 0xff))    // CRC (big endian)
                    || (response[10] != ((crc >> 16) & 0xff))
                    || (response[11] != ((crc >> 8) & 0xff))
                    || (response[12] != (crc & 0xff))
                    || (response[13] != 0x55))  // Tail
                {
                    printf("ERROR: Firmware information error\r\n");
                    exitStatus = -1;
                    break;
                }
                printResponse = false;

                // Verify the command status
                commandStatus = getCommandStatus(&response[7], &message);
                if (commandStatus)
                {
                    printf("ERROR: Firmware information command returned: %s\r\n", message);
                    exitStatus = -1;
                    break;
                }

                // Determine if firmware updates are enabled
                if (firmwareUpdateEnabled == false)
                {
                    exitStatus = BAIL_WITH_SUCCESS;
                    break;
                }

                // Start the firmware erase
                exitStatus = writeCommand("Command Packet");
                if (!exitStatus)
                    exitStatus = sendFirmwareErase();
                timeoutMessage = "ERROR: Timeout waiting for erase command response!\r\n";
                pollTimeoutUsec = 30 * 1000 * 1000;
                state = FUS_FIRMWARE_ERASE_1;
            }
            break;

        // Wait for the firmware erase command to complete
        case FUS_FIRMWARE_ERASE_1:
            if (gotResponse && strcmp((char *)response, "Command Done") == 0)
            {
                // Get the boot loader response
                printResponse = false;
                binaryResponse = true;
                timeoutMessage = "ERROR: Timeout waiting for firmware erase command status!\r\n";
                state = FUS_FIRMWARE_ERASE_2;
            }
            break;

        // Wait for the firmware erase response
        case FUS_FIRMWARE_ERASE_2:
            if (gotResponse)
            {
                // Process the response
                crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

                // Check for an error
                if ((response[0] != 0xaa)       // Head
                    || (response[1] != 2)       // Class ID
                    || (response[2] != 0)       // Message ID
                    || (response[3] != 0)       // Payload length (big endian)
                    || (response[4] != 4)
                    || (response[5] != 2)       // Class ID
                    || ((response[6] != 3) && (response[6] != 0)) // Message ID
                    || (response[9] != ((crc >> 24) & 0xff))    // CRC (big endian)
                    || (response[10] != ((crc >> 16) & 0xff))
                    || (response[11] != ((crc >> 8) & 0xff))
                    || (response[12] != (crc & 0xff))
                    || (response[13] != 0x55))  // Tail
                {
                    printf("ERROR: Firmware erase error\r\n");
                    exitStatus = -1;
                    break;
                }
                printResponse = false;

                // Verify the command status
                commandStatus = getCommandStatus(&response[7], &message);
                if (commandStatus)
                {
                    printf("ERROR: Firmware erase command returned: %s\r\n", message);
                    exitStatus = -1;
                    break;
                }

                // Display the erased message
                if (displayResponseSummary)
                    printf("Firmware erased\r\n");

                // Determine if just erasing the flash
                if (eraseOnly)
                {
                    exitStatus = BAIL_WITH_SUCCESS;
                    break;
                }

                // Start the firmware upload
                packetNumber = 0;
                timeoutMessage = "ERROR: Timeout waiting for packet write response!\r\n";
                exitStatus = writeCommand("Command Packet");
                if (!exitStatus)
                    exitStatus = sendFirmwarePacket();
                state = FUS_FIRMWARE_UPLOAD_1;
            }
            break;

        // Wait for the firmware information command to complete
        case FUS_FIRMWARE_UPLOAD_1:
            if (gotResponse && strcmp((char *)response, "Command Done") == 0)
            {
                // Get the boot loader response
                printResponse = false;
                binaryResponse = true;
                timeoutMessage = "ERROR: Timeout waiting for packet write status!\r\n";
                state = FUS_FIRMWARE_UPLOAD_2;
            }
            break;

        // Wait for the firmware upload response
        case FUS_FIRMWARE_UPLOAD_2:
            if (gotResponse)
            {
                // Process the response
                crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

                // Check for an error
                if ((response[0] != 0xaa)       // Head
                    || (response[1] != 2)       // Class ID
                    || (response[2] != 0)       // Message ID
                    || (response[3] != 0)       // Payload length (big endian)
                    || (response[4] != 4)
                    || (response[5] != 2)       // Class ID
                    || ((response[6] != 4) && (response[6] != 0)) // Message ID
                    || (response[9] != ((crc >> 24) & 0xff))    // CRC (big endian)
                    || (response[10] != ((crc >> 16) & 0xff))
                    || (response[11] != ((crc >> 8) & 0xff))
                    || (response[12] != (crc & 0xff))
                    || (response[13] != 0x55))  // Tail
                {
                    printf("ERROR: Firmware upload error, packet %d\r\n", packetNumber);
                    exitStatus = -1;
                    break;
                }
                printResponse = false;

                // Verify the command status
                commandStatus = getCommandStatus(&response[7], &message);
                if (commandStatus)
                {
                    printf("ERROR: Firmware upload command returned: %s\r\n", message);
                    exitStatus = -1;
                    break;
                }

                // Display the packet number and count
                if (displayResponseSummary)
                    printf("Packet %d of %d\r\n", packetNumber, packetCount - 1);

                // Account for this packet
                packetNumber += 1;

                // Determine if the firmware upload is complete
                if (packetNumber < packetCount)
                {
                    exitStatus = writeCommand("Command Packet");
                    if (!exitStatus)
                        exitStatus = sendFirmwarePacket();
                    timeoutMessage = "ERROR: Timeout waiting for packet write response!\r\n";
                    state = FUS_FIRMWARE_UPLOAD_1;
                    break;
                }

                // Firmware update complete
                if (displayResponseSummary)
                    printf("Firmware upload complete, resetting GNSS\r\n");

                // Send the GNSS reset command
                exitStatus = writeCommand("Command Packet");
                if (!exitStatus)
                    exitStatus = sendFirmwareReset();
                state = FUS_FIRMWARE_RESET_1;
            }
            break;

        // Wait for the reset command to complete
        case FUS_FIRMWARE_RESET_1:
            if (gotResponse && strcmp((char *)response, "Command Done") == 0)
            {
                // Get the boot loader response
                printResponse = false;
                binaryResponse = true;
                timeoutMessage = "ERROR: Timeout waiting for GNSS reset response!\r\n";
                state = FUS_FIRMWARE_RESET_2;
            }
            break;

        // Wait for the GNSS reset response
        case FUS_FIRMWARE_RESET_2:
            if (gotResponse)
            {
                // Process the response
                crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

                // Check for an error
                if ((response[0] != 0xaa)       // Head
                    || (response[1] != 2)       // Class ID
                    || (response[2] != 0)       // Message ID
                    || (response[3] != 0)       // Payload length (big endian)
                    || (response[4] != 4)
                    || (response[5] != 2)       // Class ID
                    || (response[6] != 0x31)    // Message ID
                    || (response[9] != ((crc >> 24) & 0xff))    // CRC (big endian)
                    || (response[10] != ((crc >> 16) & 0xff))
                    || (response[11] != ((crc >> 8) & 0xff))
                    || (response[12] != (crc & 0xff))
                    || (response[13] != 0x55))  // Tail
                {
                    printf("ERROR: Firmware information error\r\n");
                    exitStatus = -1;
                    break;
                }
                printResponse = false;

                // Verify the command status
                commandStatus = getCommandStatus(&response[7], &message);
                if (commandStatus)
                {
                    printf("ERROR: GNSS reset command returned: %s\r\n", message);
                    exitStatus = -1;
                    break;
                }

                // All done
                if (displayResponseSummary)
                    printf("GNSS reset\r\n");

                // Display the firmware version
                firmwareVersionOnly = true;
                exitStatus = writeCommand("Detect GNSS");
                timeoutMessage = "ERROR: Timeout during GNSS detection!\r\n";
                state = FUS_DETECT_GNSS;
            }
            break;
    }

    // Display the response
    if (printResponse)
    {
        if (responseType)
            dumpBuffer(0, response, commandResponseLength);
        else
            printf("%s\r\n", response);
    }
    return exitStatus;
}

//----------------------------------------
// Display the firmware version
void displayFirmwareVersion()
{
    const char * lg290pFirmware = "$PQTMVERNO,LG290P03AANR";
    size_t length;
    int major;
    int minor;

    // Determine if the firmware version was returned
    length = strlen(lg290pFirmware);
    if (strncmp((char *)response, lg290pFirmware, length) != 0)
        // Display the response upon error
        printf("ERROR: Firmware version response: %s\r\n", response);

    // Get the firmware version
    else if (sscanf((char *)&response[length], "%2dA%2dS", &major, &minor) != 2)
        // Display the error
        printf("ERROR: Unable to parse firmware version from %s\r\n", response);

    // Display the version number
    else
        printf("Current firmware version: %d.%d (%d)\r\n",
               major, minor, (major * 100) + minor);
}

//----------------------------------------
// Upload the firmware image through a directly connected UART
//
//      PC <---> GNSS
//----------------------------------------
int directFirmwareUpload(bool timeout)
{
    ssize_t bytesRead;
    uint16_t commandStatus;
    uint32_t crc;
    bool displayResponseSummary;
    int exitStatus;
    static bool getBinaryResponse;
    bool gotResponse;
    int length;
    const char * message;
    bool printResponse;
    const char * resetCommand = "$PQTMSRR*4B\r\n";

    // Handle timeouts
    if (timeout && timeoutMessage && (state < DCFUS_MAX))
    {
        printf("%s\r\n", timeoutMessage);
        timeoutMessage = nullptr;
        return -1;
    }

    // Determine if response summaries should be displayed
    displayResponseSummary = (displayHandshakeDiagram == false)
                             || ((displayHandshakeDiagram == true)
                                && (displayBinaryCommand == false)
                                && (displayBinaryCommandSummary == false)
                                && (displayBinaryResponse == false)
                                && (displayCommand || displayCommandResponse));

    // Get the response
    exitStatus = 0;
    gotResponse = false;
    if (getBinaryResponse && (timeout == false))
    {
        // Get the binary response
        gotResponse = getCommandResponse();

        // Handle the any response errors
        exitStatus = errno;
        if (exitStatus)
            return exitStatus;
    }

    // Process the state
    printResponse = gotResponse;
    switch (state)
    {
        // Default case for development
        default:
            // Display the error
            printf("ERROR: Unknown state %d\r\n", state);
            if (timeout)
                printf("Timeout!\r\n");
            exitStatus = -1;
            timeoutMessage = nullptr;
            break;

        // Display the firmware version if available
        case DCFUS_FIRMWARE_VERSION:
            // Attempt to get the firmware version response
            if (timeout == false)
                gotResponse = getNmeaResponse();
            if (gotResponse)
            {
                displayFirmwareVersion();
                exitStatus = writeData((const uint8_t *)resetCommand, strlen(resetCommand));
                state = DCFUS_POWER_ON;
            } else if (timeout)
            {
                exitStatus = writeData((const uint8_t *)resetCommand, strlen(resetCommand));
                state = DCFUS_POWER_ON;
            }
            break;

        // Wait for response to SYNC WORD 1
        case DCFUS_POWER_ON:
            // Resend the SYNC WORD 1 every 20 mSec
            if (timeout)
            {
                uint8_t data[4];

                // Account for this timeout
                timeoutCount += 1;

                // Exceeded 500 mSec
                if (timeoutCount > (500 / 20))
                {
                    // Display the timeout error
                    printf("ERROR: Response to SYNC_WORD1 not received in 500mSec!\r\n");
                    exitStatus = -1;
                    break;
                }

                // Send SYNC_WORD1 0x514C1309 (little endian) every 20 mSec
                data[0] = 0x09;
                data[1] = 0x13;
                data[2] = 0x4c;
                data[3] = 0x51;
                exitStatus = writeData(data, sizeof(data));

                // Start receiving a new response
                responseLength= 0;
            }
            else
            {
                // Read data from the GNSS
                bytesRead = read(comPort, &response[responseLength], 1);

                // Handle the errors
                if (bytesRead <= 0)
                {
                    exitStatus = errno;
                    break;
                }
                if (bytesRead)
                {
                    // Display the byte
                    if (displayBytesReceived)
                        printf("0x%02x\r\n", response[responseLength]);

                    // Account for this byte
                    responseLength += 1;
                }

                // Expecting RSP_WORD1 0xAAFC3A4D (little endian)
                printResponse = false;
                if (((responseLength >= 1) && (response[0] != 0x4d))
                    || ((responseLength >= 2) && (response[1] != 0x3a))
                    || ((responseLength >= 3) && (response[2] != 0xfc))
                    || ((responseLength == 4) && (response[3] != 0xaa)))
                {
                    // Error, discard any received data and try again
                    responseLength = 0;
                }

                // Sucessfully received RSP_WORD1 0xAAFC3A4D (little endian)
                else if (responseLength == 4)
                {
                    uint8_t data[4];

                    // Display the sync word and its response
                    if (displayHandshakeDiagram)
                        addCommandToHandshakeDiagram("SYNC_WORD1 0x514C1309");
                    if (displayHandshakeDiagram)
                        addResponseToHandshakeDiagram("RSP_WORD1 0xAAFC3A4D");

                    // Display a blank line
                    if (displayHandshakeDiagram)
                        printf("%s%s%s\r\n", pc, &spaces[strlen(spaces) - strlen(dashes)], microprocessor);

                    // Display the sync word
                    if (displayHandshakeDiagram)
                        addCommandToHandshakeDiagram("SYNC_WORD2 0x1203A504");

                    // Send SYNC_WORD2 0x1203A504 (little endian)
                    data[0] = 0x04;
                    data[1] = 0xa5;
                    data[2] = 0x03;
                    data[3] = 0x12;
                    exitStatus = writeData(data, sizeof(data));
                    responseLength = 0;
                    state = DCFUS_SYNC;
                }
            }
            break;

        // Waiting for RSP_WORD2 0x55FD5BA0 (little endian)
        case DCFUS_SYNC:
            // Read data from the GNSS
            bytesRead = read(comPort, &response[responseLength], 1);

            // Handle the errors
            if (bytesRead <= 0)
            {
                exitStatus = errno;
                break;
            }
            if (bytesRead)
            {
                // Display the byte
                if (displayBytesReceived)
                    printf("0x%02x\r\n", response[responseLength]);

                // Account for this byte
                responseLength += 1;
            }

            // Expecting RSP_WORD2 0x55FD5BA0 (little endian)
            printResponse = false;
            if (((responseLength >= 1) && (response[0] != 0xa0))
                || ((responseLength >= 2) && (response[1] != 0x5b))
                || ((responseLength >= 3) && (response[2] != 0xfd))
                || ((responseLength == 4) && (response[3] != 0x55)))
            {
                // Error, discard any received data and try again
                responseLength = 0;
                state = DCFUS_POWER_ON;
            }

            // Sucessfully received RSP_WORD2 0x55FD5BA0 (little endian)
            else if (responseLength == 4)
            {
                // Display the sync word response
                if (displayHandshakeDiagram)
                    addResponseToHandshakeDiagram("RSP_WORD2 0x55FD5BA0");

                // Display a blank line
                if (displayHandshakeDiagram)
                    printf("%s%s%s\r\n", pc, &spaces[strlen(spaces) - strlen(dashes)], microprocessor);

                // Send the boot version command
                exitStatus = getBootLoaderVersion();
                getBinaryResponse = true;
                responseLength = 0;
                pollTimeoutUsec = 500 * 1000;
                timeoutMessage = "ERROR: Timeout getting bootloader version command!\r\n";
                state = DCFUS_BOOT_VERSION;
            }
            break;

        case DCFUS_BOOT_VERSION:
            if (gotResponse)
            {
                // Process the response
                crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

                // Check for an error
                if ((response[0] != 0xaa)       // Head
                    || (response[1] != 2)       // Class ID
                    || (response[2] != 0)       // Message ID
                    || (response[3] != 0)       // Payload length (big endian)
                    || (response[4] != 7)
                    || (response[5] != 2)       // Class ID
                    || (response[6] != 0x71)    // Message ID
                    || (response[12] != ((crc >> 24) & 0xff))   // CRC (big endian)
                    || (response[13] != ((crc >> 16) & 0xff))
                    || (response[14] != ((crc >> 8) & 0xff))
                    || (response[15] != (crc & 0xff))
                    || (response[16] != 0x55))  // Tail
                {
                    printf("ERROR: Boot loader version error\r\n");
                    exitStatus = -1;
                    break;
                }
                printResponse = false;

                // Verify the command status
                commandStatus = getCommandStatus(&response[7], &message);
                if (commandStatus)
                {
                    printf("ERROR: Failed to get boot loader version!\r\n");
                    if (displayBinaryResponse == false)
                        printf("%s\r\n", message);
                    exitStatus = -1;
                    break;
                }

                // Display the boot loader version
                if (displayResponseSummary)
                    printf("Bootloader version: %d.%d.%d\r\n", response[9], response[10], response[11]);

                // Send the firmware information
                exitStatus = sendFirmwareInfo();
                state = DCFUS_FIRMWARE_INFO;
            }
            break;

        // Wait for the firmware information response
        case DCFUS_FIRMWARE_INFO:
            if (gotResponse)
            {
                // Process the response
                crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

                // Check for an error
                if ((response[0] != 0xaa)       // Head
                    || (response[1] != 2)       // Class ID
                    || (response[2] != 0)       // Message ID
                    || (response[3] != 0)       // Payload length (big endian)
                    || (response[4] != 4)
                    || (response[5] != 2)       // Class ID
                    || ((response[6] != 2) && (response[6] != 0)) // Message ID
                    || (response[9] != ((crc >> 24) & 0xff))    // CRC (big endian)
                    || (response[10] != ((crc >> 16) & 0xff))
                    || (response[11] != ((crc >> 8) & 0xff))
                    || (response[12] != (crc & 0xff))
                    || (response[13] != 0x55))  // Tail
                {
                    printf("ERROR: Firmware information error\r\n");
                    exitStatus = -1;
                    break;
                }
                printResponse = false;

                // Verify the command status
                commandStatus = getCommandStatus(&response[7], &message);
                if (commandStatus)
                {
                    printf("ERROR: Failed to set the firmware information!\r\n");
                    if (displayBinaryResponse == false)
                        printf("%s\r\n", message);
                    exitStatus = -1;
                    break;
                }

                // Determine if firmware updates are enabled
                if (firmwareUpdateEnabled == false)
                {
                    exitStatus = BAIL_WITH_SUCCESS;
                    break;
                }

                // Start the firmware erase
                exitStatus = sendFirmwareErase();
                pollTimeoutUsec = 30 * 1000 * 1000;
                timeoutMessage = "ERROR: Timeout waiting for erase command response!\r\n";
                state = DCFUS_FIRMWARE_ERASE;
            }
            break;

        // Wait for the firmware erase response
        case DCFUS_FIRMWARE_ERASE:
            if (gotResponse)
            {
                // Process the response
                crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

                // Check for an error
                if ((response[0] != 0xaa)       // Head
                    || (response[1] != 2)       // Class ID
                    || (response[2] != 0)       // Message ID
                    || (response[3] != 0)       // Payload length (big endian)
                    || (response[4] != 4)
                    || (response[5] != 2)       // Class ID
                    || ((response[6] != 3) && (response[6] != 0)) // Message ID
                    || (response[9] != ((crc >> 24) & 0xff))    // CRC (big endian)
                    || (response[10] != ((crc >> 16) & 0xff))
                    || (response[11] != ((crc >> 8) & 0xff))
                    || (response[12] != (crc & 0xff))
                    || (response[13] != 0x55))  // Tail
                {
                    printf("ERROR: Firmware erase error\r\n");
                    exitStatus = -1;
                    break;
                }
                printResponse = false;

                // Verify the command status
                commandStatus = getCommandStatus(&response[7], &message);
                if (commandStatus)
                {
                    printf("ERROR: Failed to erase the GNSS firmware!\r\n");
                    if (displayBinaryResponse == false)
                        printf("%s\r\n", message);
                    exitStatus = -1;
                    break;
                }

                // Display the erased message
                if (displayResponseSummary)
                    printf("Firmware erased\r\n");

                // Determine if just erasing the flash
                if (eraseOnly)
                {
                    exitStatus = BAIL_WITH_SUCCESS;
                    break;
                }

                // Start the firmware upload
                packetNumber = 0;
                timeoutMessage = "ERROR: Timeout waiting for firmware packet response!\r\n";
                exitStatus = writeCommand("Command Packet");
                if (!exitStatus)
                    exitStatus = sendFirmwarePacket();
                state = DCFUS_FIRMWARE_UPLOAD;
            }
            break;

        // Wait for the firmware upload response
        case DCFUS_FIRMWARE_UPLOAD:
            if (gotResponse)
            {
                // Process the response
                crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

                // Check for an error
                if ((response[0] != 0xaa)       // Head
                    || (response[1] != 2)       // Class ID
                    || (response[2] != 0)       // Message ID
                    || (response[3] != 0)       // Payload length (big endian)
                    || (response[4] != 4)
                    || (response[5] != 2)       // Class ID
                    || ((response[6] != 4) && (response[6] != 0)) // Message ID
                    || (response[9] != ((crc >> 24) & 0xff))    // CRC (big endian)
                    || (response[10] != ((crc >> 16) & 0xff))
                    || (response[11] != ((crc >> 8) & 0xff))
                    || (response[12] != (crc & 0xff))
                    || (response[13] != 0x55))  // Tail
                {
                    printf("ERROR: Firmware upload error, packet %d\r\n", packetNumber);
                    exitStatus = -1;
                    break;
                }
                printResponse = false;

                // Verify the command status
                commandStatus = getCommandStatus(&response[7], &message);
                if (commandStatus)
                {
                    printf("ERROR: Failed firmware upload at packet %d!\r\n", packetNumber);
                    if (displayBinaryResponse == false)
                        printf("%s\r\n", message);
                    exitStatus = -1;
                    break;
                }

                // Display the packet number and count
                if (displayResponseSummary)
                    printf("Packet %d of %d\r\n", packetNumber, packetCount - 1);

                // Account for this packet
                packetNumber += 1;

                // Determine if the firmware upload is complete
                if (packetNumber < packetCount)
                {
                    exitStatus = sendFirmwarePacket(packetNumber);
                    if ((packetNumber + 1) == packetCount)
                        pollTimeoutUsec = 30 * 1000 * 1000;
                    state = DCFUS_FIRMWARE_UPLOAD;
                    break;
                }

                // Firmware update complete
                if (displayResponseSummary)
                    printf("Firmware upload complete, resetting GNSS\r\n");

                // Send the GNSS reset command
                exitStatus = sendFirmwareReset();
                state = DCFUS_FIRMWARE_RESET;
            }
            break;

        // Wait for the GNSS reset response
        case DCFUS_FIRMWARE_RESET:
            if (gotResponse)
            {
                // Process the response
                crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

                // Check for an error
                if ((response[0] != 0xaa)       // Head
                    || (response[1] != 2)       // Class ID
                    || (response[2] != 0)       // Message ID
                    || (response[3] != 0)       // Payload length (big endian)
                    || (response[4] != 4)
                    || (response[5] != 2)       // Class ID
                    || (response[6] != 0x31)    // Message ID
                    || (response[9] != ((crc >> 24) & 0xff))    // CRC (big endian)
                    || (response[10] != ((crc >> 16) & 0xff))
                    || (response[11] != ((crc >> 8) & 0xff))
                    || (response[12] != (crc & 0xff))
                    || (response[13] != 0x55))  // Tail
                {
                    printf("ERROR: Firmware information error\r\n");
                    exitStatus = -1;
                    break;
                }
                printResponse = false;

                // Verify the command status
                commandStatus = getCommandStatus(&response[7], &message);
                if (commandStatus)
                {
                    printf("ERROR: Failed GNSS reset\r\n");
                    if (displayBinaryResponse == false)
                        printf("%s\r\n", message);
                    exitStatus = -1;
                    break;
                }

                // All done
                if (displayResponseSummary)
                    printf("GNSS reset\r\n");
                exitStatus = BAIL_WITH_SUCCESS;
            }
            break;
    }

    // Display the response
    if (printResponse)
        dumpBuffer(0, response, commandResponseLength);
    return exitStatus;
}

//----------------------------------------
// Handle data flow with the COM Port
//----------------------------------------
int handleComPort()
{
    fd_set currentfds;
    int exitStatus;
    int maxfds;
    int numfds;
    fd_set readfds;
    struct timeval timeout;
    const char * versionInfoCommand = "$PQTMVERNO*58\r\n";


    //Initialize the fd_sets
    FD_ZERO(&readfds);
    FD_SET(comPort, &readfds);
    maxfds = fileno(stdin);
    if (maxfds < comPort)
        maxfds = comPort;

    // Send the initial command
    timeoutMessage = nullptr;
    exitStatus = 0;
    if (useMicroprocessor)
        exitStatus = writeCommand(helloMicro);
    else
        exitStatus = writeData((const uint8_t *)versionInfoCommand, strlen(versionInfoCommand));

    // Wait for a response
    while (exitStatus == 0)
    {
        //Set the timeout
        timeout.tv_sec = pollTimeoutUsec / (1000 * 1000);
        timeout.tv_usec = pollTimeoutUsec % (10000 * 1000);

        //Wait for receive data or timeout
        memcpy((void *)&currentfds, (void *)&readfds, sizeof(readfds));
        numfds = select(maxfds + 1, &currentfds, NULL, NULL, &timeout);
        if (numfds < 0)
        {
            exitStatus = errno;
            perror("ERROR: select call failed!");
            break;
        }

        // Wait for power on to complete or connection to microprocessor
        else if (useMicroprocessor == false)
        {
            //Determine microprocessor output is available
            if (FD_ISSET(comPort, &currentfds))
                exitStatus = directFirmwareUpload(false);

            // Check for timeout
            else if (numfds == 0)
                exitStatus = directFirmwareUpload(true);
        }
        else
        {
            //Determine microprocessor output is available
            if (FD_ISSET(comPort, &currentfds))
                exitStatus = uploadFirmware(false);

            // Check for timeout
            else if (numfds == 0)
                exitStatus = uploadFirmware(true);
        }
    }
    return exitStatus;
}

//----------------------------------------
// Connect to the COM port
//----------------------------------------
int connectComPort(const char * portName)
{
    int exitStatus;

    exitStatus = 0;
    do
    {
        // Wait for the COM Port to become available
        printf("Waiting for GNSS power on and COM Port %s\r\n", portName);
        do
        {
            // Attempt to open the COM Port
            comPort = open(portName, O_RDWR, 0);
        } while (waitForUart && (comPort < 0));

        // Handle the connection error
        if (comPort < 0)
        {
            exitStatus = errno;
            printf("ERROR: Failed to open COM Port %s\r\n", portName);
            perror("");
            break;
        }

        // Configure the COM Port
        exitStatus = configureComPort(baudrate);
        if (exitStatus)
            break;

        // Display the headshake header
        if (displayHandshakeDiagram)
        {
            size_t spaceCount = strlen(spaces) - strlen(dashes);
            printf("%s%s%s\r\n", pcTop, &spaces[spaceCount], microprocessorTop);
            printf("%s%s%s\r\n", pcLabel, &spaces[spaceCount], microprocessorLabel);
        }
    } while (0);
    return exitStatus;
}

//----------------------------------------
// Translate the B-value into a baudrate
//----------------------------------------
int baudrateLookup()
{
    // Walk the baudrate table
    for (int index = 0; index < baudrateTableEntries; index++)
        if (baudrateTable[index]._bValue == baudrate)
            return baudrateTable[index]._baudrate;

    // Unknown baudrate value
    return 0;
}

//----------------------------------------
// Application to update the firmware on a Quectel GNSS device
//----------------------------------------
int main(int argc, char **argv)
{
    int argCount;
    int argOffset;
    size_t bytes;
    static bool displayArguments;
    static bool displayHandshake;
    int exitStatus;
    const char * fileName;
    int index;
    const COMMAND_OPTION options[] =
    {
        {0, 1, &baudrateSet,                "--baudrate", "Set the baudrate between the PC and microprocessor,\r\n                defaults to 115200, Example: --baudrate   19200"},
        {0, 0, &displayArguments,           "--display-arguments", "Display the command arguments"},
        {1, 0, &displayBinaryCommand,       "--display-binary-command", "Dump the binary command in hexadecimal and ASCII"},
        {1, 0, &displayBinaryResponse,      "--display-binary-response", "Dump the binary response in hexadecimal and ASCII"},
        {1, 0, &displayBinaryCommandSummary,"--display-binary-summary", "Dump a summary of the binary command in hexadecimal and ASCII"},
        {0, 0, &displayBytesReceived,       "--display-bytes-received", "Display each of the received bytes"},
        {1, 0, &displayCommand,             "--display-command", "Display the microprocessor commands"},
        {1, 0, &displayCommandResponse,     "--display-command-response", "Display the microprocessor command responses"},
        {1, 0, &displayHandshake,           "--display-handshake-diagram", "Display the handshake diagram"},
        {0, 0, &firmwareUpdateEnabled,      "--firmware-update-enabled", "Enable firmware updates"},
        {0, 0, &eraseOnly,                  "--erase-only", "Perform the flash erase and then exit"},
        {0, 0, &skipVersionCheck,           "--skip-version-check", "Don't display current firmware version"},
        {0, 0, &useMicroprocessor,          "--use-microprocessor", "Communicate with the GNSS through a microprocessor"},
        {0, 0, &waitForUart,                "--wait-for-uart", "From GNSS system power on, wait for the UART to appear"},
    };
    const int optionCount = sizeof(options) / sizeof(options[0]);
    const char * portName;
    bool validCommand;

    exitStatus = -1;
    do
    {
        // Get the options
        argCount = 0;
        argOffset = 1;
        portName = "";
        displayHandshakeDiagram = false;
        fileName = "";
        validCommand = true;
        while (argc - argOffset)
        {
            bool match;

            // Check for an option
            match = false;
            if (strncmp(argv[argOffset], "--", 2) == 0)
            {
                // Walk the list of options
                for (index = 0; index < optionCount; index++)
                {
                    match = (strcmp(argv[argOffset], options[index]._optionString) == 0);
                    if (match)
                    {
                        displayHandshakeDiagram |= options[index]._displayHandshakeDiagram;
                        *options[index]._optionBoolean = true;
                        argOffset += 1;
                        break;
                    }
                }

                // Check for an integer value
                if (match && options[index]._getBaudRate)
                {
                    int value;

                    // Verify that at least one more argument is present
                    if (argOffset >= argc)
                    {
                        printf("ERROR: Baudrate not specified\r\n");
                        validCommand = false;
                        break;
                    }

                    // Get the baudrate value
                    if (sscanf(argv[argOffset], "%d", &value) == 0)
                    {
                        printf("ERROR: Invalid baudrate value, %s\r\n", argv[argOffset]);
                        validCommand = false;
                        break;
                    }
                    else
                    {
                        // Validate the baudrate value
                        for (index = 0; index < baudrateTableEntries; index++)
                        {
                            if (baudrateTable[index]._baudrate == value)
                            {
                                baudrate = baudrateTable[index]._bValue;
                                argOffset += 1;
                                break;
                            }
                        }

                        // Display the baudrate value error
                        if (index >= baudrateTableEntries)
                        {
                            printf("ERROR: Invalid baudrate value, %d\r\n", value);
                            validCommand = false;
                            break;
                        }
                    }
                }
            }

            // Check for a valid argument
            else if (argCount < 2)
            {
                if (argCount == 0)
                    // Save the terminal port name
                    portName = argv[argOffset++];
                else
                    // Save the file name
                    fileName = argv[argOffset++];
                match = true;
                argCount += 1;
            }

            // Handle the error
            if (match == false)
            {
                // Display the help message
                printf("Invalid argument or option: %s\r\n", argv[argOffset]);
                break;
            }
        }

        // Check for a valid command
        if ((argCount < 2) || (argOffset != argc))
            validCommand = false;

        // Display the help text
        if (validCommand == false)
        {
            printf("%s   [options]   <COM_Port%s%s>   <Firmware_File>\r\n",
                   argv[0],
                   (argc == 2) ? ": " : "",
                   (argc == 2) ? argv[1] : ""
                   );
            printf("    COM_PORT: Example COM3 or /dev/ttyACM0\r\n");
            printf("    Firmware_File: Example LG290P03AANR02A01S.pkg\r\n");
            printf("Options:\r\n");
            for (index = 0; index < optionCount; index++)
                printf("    %s: %s\r\n", options[index]._optionString, options[index]._helpText);
            printf("\r\n");
            printf("Program to update the firmware on the Quectel GNSS device.\r\n");
            break;
        }

        // Display the options and arguments
        if (displayArguments)
        {
            // Display the options
            for (index = 1; index < optionCount; index++)
                printf("%s: %s\r\n", options[index]._optionString,
                       *options[index]._optionBoolean ? "true" : "false");

            // Display the command arguments
            printf("Port: %s\r\n", portName);
            printf("File name: %s\r\n", fileName);
        }

        // The GNSS bootloader is always using 460800 as the baudrate, see
        // Section 2.1 of the Quectel LG290P (03) Firmware Upgrade Guide
        if (useMicroprocessor == false)
            baudrate = B460800;

        // Determine the baudrate between the PC and microprocessor,
        // defaults to 115200
        else if ((baudrateSet == false) && useMicroprocessor)
            baudrate = B115200;

        // Determine if displaying the handshake diagram
        if (displayHandshake)
        {
            displayCommand = true;
            displayCommandResponse = true;
        }

        // Set display-binary-command if displaying a summary
        if (displayBinaryCommandSummary)
            displayBinaryCommand = true;

        // Attempt to open the firmware file
        firmware = open(fileName, O_RDONLY, 0);
        if (firmware < 0)
        {
            exitStatus = errno;
            printf("ERROR: Failed to open file %s\r\n", fileName);
            perror("");
            break;
        }

        // Determine the firmware length
        firmwareLength = lseek(firmware, 0, SEEK_END);
        if (firmwareLength == (off_t)-1)
        {
            exitStatus = errno;
            perror("ERROR: Failed to get the file length");
            break;
        }
        printf("Firmware length: %ld bytes\r\n", firmwareLength);

        // Start at the beginning of the file
        if (lseek(firmware, 0, SEEK_SET) == (off_t)-1)
        {
            exitStatus = errno;
            perror("ERROR: Failed to set the start of the file");
            break;
        }

        // Map the firmware image into memory
        firmwarePackage = (uint8_t *)mmap(NULL,
                                          (size_t)firmwareLength,
                                          PROT_READ,
                                          MAP_SHARED,   // MAP_PRIVATE | MAP_NORESERVE | MAP_POPULATE,
                                          firmware,
                                          0);
        if (firmwarePackage == MAP_FAILED)
        {
            exitStatus = errno;
            perror("ERROR: Failed to map the firmware into memory");
            break;
        }

        // Determine the packet count
        packetCount = (firmwareLength + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;

        // CRC test
        uint8_t crcTest[4];
        crcTest[0] = 2;
        crcTest[1] = 3;
        crcTest[2] = 0;
        crcTest[3] = 0;
        firmwareCrc32 = computeCrc32(0, crcTest, sizeof(crcTest));
        if (firmwareCrc32 != 0x890ba9ce)
        {
            printf("Test 1: Expecting 0x890ba9ce actual 0x%08x\r\n", firmwareCrc32);
            exitStatus = -1;
            break;
        }

        firmwareCrc32 = computeCrc32(0,             &crcTest[0], 1);
        firmwareCrc32 = computeCrc32(firmwareCrc32, &crcTest[1], 1);
        firmwareCrc32 = computeCrc32(firmwareCrc32, &crcTest[2], 1);
        firmwareCrc32 = computeCrc32(firmwareCrc32, &crcTest[3], 1);
        if (firmwareCrc32 != 0x890ba9ce)
        {
            printf("Test 2: Expecting 0x890ba9ce actual 0x%08x\r\n", firmwareCrc32);
            exitStatus = -1;
            break;
        }

        // Compute the firmware CRC
        uint8_t bytes[4];
        insertLittleEndian(firmwareLength, bytes);
        firmwareCrc32 = computeCrc32(0, bytes, sizeof(bytes));
        firmwareCrc32 = computeCrc32(firmwareCrc32, firmwarePackage, firmwareLength);
        printf("Firmware CRC32: 0x%08x\r\n", firmwareCrc32);

        // Attempt to connect to the COM port
        exitStatus = connectComPort(portName);
        if (exitStatus)
            break;

        // Attempt to upgrade the GNSS firmware
        packetNumber = -1;
        timeoutCount = 0;
        if (useMicroprocessor == false)
            pollTimeoutUsec = 20 * 1000;
        else
            pollTimeoutUsec = 500 * 1000;
        exitStatus = handleComPort();
    } while (0);

    // Done with the COM port
    if (comPort >= 0)
        close(comPort);

    // Release the mapped firmware file
    if (firmwarePackage)
        munmap(firmwarePackage, firmwareLength);

    // Done with the firmware file
    if (firmware >= 0)
        close(firmware);

    // Convert the exitStatus value if necessary
    if (exitStatus == BAIL_WITH_SUCCESS)
    {
        if (displayHandshakeDiagram)
        {
            size_t spaceCount = strlen(spaces) - strlen(dashes);
            printf("%s%s%s\r\n", pcLabel, &spaces[spaceCount], microprocessorLabel);
            printf("%s%s%s\r\n", pcBottom, &spaces[spaceCount], microprocessorBottom);
        }
        exitStatus = 0;
    }
    return exitStatus;
}
