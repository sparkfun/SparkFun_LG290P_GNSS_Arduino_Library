/*
  This is a library to control Quectel GNSS receivers, with
  a focus on the LG290P QuadBand receiver.

  Development environment specifics:
  Arduino IDE 1.8.x

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include "SparkFun_LG290P_GNSS.h"

//----------------------------------------
// Constants
//----------------------------------------

// parserTable index values
#define LG290P_NMEA_PARSER_INDEX 0
#define LG290P_RTCM_PARSER_INDEX 1

// Build the table listing all of the parsers
static const SEMP_PARSER_DESCRIPTION *lg290pParserTable[] = {
    &sempNmeaParserDescription,
    &sempRtcmParserDescription,
};
static const int lg290pParserCount = sizeof(lg290pParserTable) / sizeof(lg290pParserTable[0]);

// Account for the largest message
#define BUFFER_LENGTH 3000

//----------------------------------------
// Globals
//----------------------------------------

LG290P *ptrLG290P = nullptr; // Global pointer for external parser access into library class

//----------------------------------------
// Parser support routines
//----------------------------------------

// Enable the display of bad checksum messages from the parser
void LG290P::enablePrintBadChecksums()
{
    _printBadChecksum = true;
}

// Disable the display of bad checksum messages from the parser
void LG290P::disablePrintBadChecksums()
{
    _printBadChecksum = false;
}

// Translate the state value into an ASCII state name
const char *LG290PGetStateName(SEMP_PARSE_STATE *parse)
{
    return sempGetStateName(parse);
}

// Disable debug output from the parser
void LG290P::disableParserDebug()
{
    sempDebugOutputDisable(_sempParse);
}

// Enable debug output from the parser
void LG290P::enableParserDebug(SEMP_OUTPUT output)
{
    sempDebugOutputEnable(_sempParse, output);
}

// Disable debug output from the parser
void LG290P::disableParserErrors()
{
    sempErrorOutputDisable(_sempParse);
}

// Enable debug output from the parser
void LG290P::enableParserErrors(SEMP_OUTPUT output)
{
    sempErrorOutputEnable(_sempParse, output);
}

// Print the LG290P parser configuration
void LG290P::printParserConfiguration(SEMP_OUTPUT output)
{
    sempPrintParserConfiguration(_sempParse, output);
}

bool LG290P::badNmeaChecksum(SEMP_PARSE_STATE *parse)
{
    return false;
}

//----------------------------------------
// LG290P support routines
//----------------------------------------

bool LG290P::begin(HardwareSerial &serialPort, const char *parserName, SEMP_OUTPUT errorOutput /* = nullptr */,
                   Print *parserDebug /* = nullptr */, SEMP_OUTPUT debugOutput /* = nullptr */)
{
    ptrLG290P = this;
    _hwSerialPort = &serialPort;
    _debugPort = parserDebug;

    // Initialize the parser
    size_t bufferLength = sempGetBufferLength(lg290pParserTable, lg290pParserCount, BUFFER_LENGTH);
    uint8_t *buffer = (uint8_t *)malloc(bufferLength);
    _sempParse = sempBeginParser(parserName, lg290pParserTable, lg290pParserCount, buffer, bufferLength,
                                 LG290PProcessMessage, errorOutput, debugOutput, badNmeaChecksum);
    if (!_sempParse)
    {
        debugPrintf("LG290P Lib: Failed to initialize the parser!");
        return false;
    }

    bool ok = isConnected();
    ok = ok && getMode(devState.mode);
    ok = ok && scanForMsgsEnabled();

    int firmwareVersionMajor = 0;
    int firmwareVersionMinor = 0;
    ok = ok && getFirmwareVersionMajor(firmwareVersionMajor);
    ok = ok && getFirmwareVersionMinor(firmwareVersionMinor);
    firmwareVersionInt = (firmwareVersionMajor * 100) + firmwareVersionMinor; // v2.16 becomes 216

    if (ok)
    {
        debugPrintf("Firmware version is %d.%d %s", firmwareVersionMajor, firmwareVersionMinor,
                    firmwareVersionInt == 0 ? " (Unknown)" : "");
        debugPrintf("Starting with %s mode, GGA %d RMC %d EPE %d PVT %d PL %d SVIN %d GSV %d GST %d PPPNAV %d",
                    devState.mode == BASEMODE ? "BASE" : "ROVER", devState.ggaRate, devState.rmcRate, devState.epeRate,
                    devState.pvtRate, devState.plRate, devState.svinstatusRate, devState.gsvRate, devState.gstRate,
                    devState.pppnavRate);
    }
    else
    {
        debugPrintf("begin() failed.");
        _sempParse = nullptr;
        free(buffer);
    }

    return ok;
}

bool LG290P::beginAutoBaudDetect(HardwareSerial &serialPort, int rxPin, int txPin, const char *parserName,
                                 SEMP_OUTPUT errorOutput /* = nullptr */, Print *parserDebug /* = nullptr */,
                                 SEMP_OUTPUT debugOutput /* = nullptr */)
{
    serialPort.setRxBufferSize(4096);
    _debugPort = parserDebug;

    for (int baud : {460800, 921600, 230400, 115200, 9600})
    {
        debugPrintf("Trying baud rate %d...", baud);
        serialPort.begin(baud, SERIAL_8N1, rxPin, txPin);
        if (begin(serialPort, parserName, errorOutput, parserDebug, debugOutput))
            return true;
    }
    return false;
}

// Query the device with 'UNIQID', expect OK response
// Device may be booting and outputting other messages (ie, PQTMVER)
// Try a few times
bool LG290P::isConnected()
{
    // Try up to 10 seconds
    for (unsigned long start = millis(); millis() - start < 10000;)
    {
        if (sendOkCommand("PQTMUNIQID"))
            return true;
    }
    return false;
}

// If another task outside of this library is accessing the same Serial hardware, it can
// check to see if this library currently needs exclusive read/write access for a short period.
// If isBlocking is true, external consumers should not read/write to the Serial hardware
bool LG290P::isBlocking()
{
    return lg290PLibrarySemaphoreBlock;
}

// Calling this function with nothing sets the debug port to Serial
// You can also call it with other streams like Serial1, SerialUSB, etc.
void LG290P::enableDebugging(Print &debugPort)
{
    _debugPort = &debugPort;
}

void LG290P::disableDebugging()
{
    _debugPort = nullptr;
}

// Check for new data until there is no more
bool LG290P::update()
{
    bool newData = false;

    lg290PLibrarySemaphoreBlock = true; // Allow external tasks to control serial hardware

    newData = serialAvailable(); // Check if new data needs parsing

    while (serialAvailable())
        update(serialRead()); // Pass character from Serial stream to parser

    lg290PLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware

    return newData;
}

// Process a buffer of bytes. Allows a stream outside of library to feed the library.
bool LG290P::update(uint8_t *incomingBuffer, uint16_t bufferLength)
{
    bool newData = false;

    lg290PLibrarySemaphoreBlock = true; // Allow external tasks to control serial hardware

    newData = serialAvailable(); // Check if new data needs parsing

    for (int x = 0; x < bufferLength; x++)
        update(incomingBuffer[x]);

    lg290PLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware

    return newData;
}

// Process a given byte into the SEMP
// LG290PProcessMessage() is called once the parser completes on a line
bool LG290P::update(byte incoming)
{
    const char *endName;
    const char *startName = nullptr;
    SEMP_PARSE_ROUTINE startState;

    // Get the current state and state name
    if (_printParserTransitions)
    {
        startState = _sempParse->state;
        startName = LG290PGetStateName(_sempParse);
    }

    // Update the parser state based on the incoming byte
    sempParseNextByte(_sempParse, incoming);

    // Get the current state name
    if (_printParserTransitions)
    {
        endName = LG290PGetStateName(_sempParse);

        // Display the parser state transition
        debugPrintf("LG290P Lib: 0x%02x (%c), crc: 0x%08x, state: %s --> %s", incoming,
                    ((incoming >= ' ') && (incoming < 0x7f)) ? incoming : '.', _sempParse->crc, startName, endName);
    }

    (void)startState; // Fix pesky warning-as-error

    return true;
}

// Enable the display of parser transitions
void LG290P::enablePrintParserTransitions()
{
    _printParserTransitions = true;
}

// Enable the display of parser transitions
void LG290P::disablePrintParserTransitions()
{
    _printParserTransitions = false;
}

// Display the contents of a buffer
void LG290P::dumpBuffer(const uint8_t *buffer, uint16_t length)
{
    int bytes;
    const uint8_t *end;
    int index;
    char line[128];
    uint16_t offset;

    end = &buffer[length];
    offset = 0;
    while (buffer < end)
    {
        // Determine the number of bytes to display on the line
        bytes = end - buffer;
        if (bytes > (16 - (offset & 0xf)))
            bytes = 16 - (offset & 0xf);

        // Display the offset
        sprintf(line, "0x%08lx: ", (long unsigned int)offset);

        // Skip leading bytes
        for (index = 0; index < (offset & 0xf); index++)
            sprintf(&line[strlen(line)], "   ");

        // Display the data bytes
        for (index = 0; index < bytes; index++)
            sprintf(&line[strlen(line)], "%02x ", buffer[index]);

        // Separate the data bytes from the ASCII
        for (; index < (16 - (offset & 0xf)); index++)
            sprintf(&line[strlen(line)], "   ");
        sprintf(&line[strlen(line)], " ");

        // Skip leading bytes
        for (index = 0; index < (offset & 0xf); index++)
            sprintf(&line[strlen(line)], " ");

        // Display the ASCII values
        for (index = 0; index < bytes; index++)
            sprintf(&line[strlen(line)], "%c",
                    ((buffer[index] < ' ') || (buffer[index] >= 0x7f)) ? '.' : buffer[index]);
        debugPrintf("%s", line);

        // Set the next line of data
        buffer += bytes;
        offset += bytes;
    }
}

// Enable the display of received messages
void LG290P::enablePrintRxMessages()
{
    _printRxMessages = true;
}

// Disable the display of received messages
void LG290P::disablePrintRxMessages()
{
    _printRxMessages = false;
}

// Enable the hex dump of received messages
void LG290P::enableRxMessageDump()
{
    _dumpRxMessages = true;
}

// Disable the hex dump of received messages
void LG290P::disableRxMessageDump()
{
    _dumpRxMessages = false;
}

// Static callback from within parser, for end of message
// Process a complete message incoming from parser
void LG290P::LG290PProcessMessage(SEMP_PARSE_STATE *parse, uint16_t type)
{
    if (ptrLG290P->_printRxMessages)
    {
        // Display the raw message
        ptrLG290P->debugPrintf("");
        switch (type)
        {
        case LG290P_NMEA_PARSER_INDEX:
            ptrLG290P->debugPrintf("LG290P Lib: Valid NMEA Sentence: %s, 0x%04x (%d) bytes",
                                   sempNmeaGetSentenceName(parse), parse->length, parse->length);
            break;

        case LG290P_RTCM_PARSER_INDEX:
            ptrLG290P->debugPrintf("LG290P Lib: Valid RTCM message: 0x%04x (%d) bytes", parse->length, parse->length);
            break;
        }
    }

    // Dump the contents of the parsed messages
    if (ptrLG290P->_dumpRxMessages)
        ptrLG290P->dumpBuffer(parse->buffer, parse->length);

    // Process the message
    switch (type)
    {
    case LG290P_RTCM_PARSER_INDEX:
        ptrLG290P->rtcmHandler(parse);
        break;
    case LG290P_NMEA_PARSER_INDEX:
        ptrLG290P->nmeaHandler(parse);
        break;
    }
}

// Commands
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Statistics (counting packets of various types that arrived)
int LG290P::getNmeaCount(const char *sentenceId /* = nullptr */)
{
    if (sentenceId != nullptr)
    {
        auto find = nmeaCounters.find(sentenceId);
        return find == nmeaCounters.end() ? 0 : find->second;
    }
    int sum = 0;
    for (auto &item : nmeaCounters)
        sum += item.second;
    return sum;
}

int LG290P::getRtcmCount(int packetType /* = -1 */)
{
    if (packetType != -1)
    {
        auto find = rtcmCounters.find(packetType);
        return find == rtcmCounters.end() ? 0 : find->second;
    }
    int sum = 0;
    for (auto &item : rtcmCounters)
        sum += item.second;
    return sum;
}

bool LG290P::setModeBase(bool resetAfter /* = true */)
{
    bool ret = sendOkCommand("PQTMCFGRCVRMODE", ",W,2");
    if (resetAfter)
        ret = ret && save() && reset();
    if (ret)
        devState.mode = BASEMODE;
    return ret;
}

bool LG290P::setModeRover(bool resetAfter)
{
    bool ret = sendOkCommand("PQTMCFGRCVRMODE", ",W,1");
    if (resetAfter)
        ret = ret && save() && reset();
    if (ret)
    {
        // In Rover mode the SVINSTATUS messages don't work, so clear the domain.
        svinStatusDomain.clear();
        devState.mode = ROVERMODE;
    }
    return ret;
}

bool LG290P::getMode(int &mode)
{
    bool ret = sendOkCommand("PQTMCFGRCVRMODE", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        mode = atoi(packet[2].c_str());
    }
    return ret;
}

bool LG290P::setPortBaudrate(int port, uint32_t newBaud, uint16_t maxWaitMs)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d,%d", port, (int)newBaud);
    return sendOkCommand("PQTMCFGUART", parms, maxWaitMs);
}

// Set baud rate on current port
bool LG290P::setBaudrate(uint32_t newBaud, uint16_t maxWaitMs)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d", (int)newBaud);
    return sendOkCommand("PQTMCFGUART", parms, maxWaitMs);
}

bool LG290P::getPortInfo(int port, uint32_t &newBaud, uint8_t &databits, uint8_t &parity, uint8_t &stop,
                         uint8_t &flowControl, uint16_t maxWaitMs)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",R,%d", port);
    bool ret = sendCommand("PQTMCFGUART", parms, maxWaitMs);
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        newBaud = atol(packet[3].c_str());
        databits = atoi(packet[4].c_str());
        parity = atoi(packet[5].c_str());
        stop = atoi(packet[6].c_str());
        flowControl = atoi(packet[7].c_str());
    }
    return ret;
}

bool LG290P::setPortInputProtocols(int port, uint8_t newFlags)
{
    uint8_t inputFlags, outputFlags;
    if (!getPortProtocols(port, inputFlags, outputFlags))
        return false;

    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,%d,%d,%d", port, newFlags, outputFlags);
    return sendOkCommand("PQTMCFGPROT", parms);
}

bool LG290P::setPortOutputProtocols(int port, uint8_t newFlags)
{
    uint8_t inputFlags, outputFlags;
    if (!getPortProtocols(port, inputFlags, outputFlags))
        return false;

    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,%d,%d,%d", port, inputFlags, newFlags);
    return sendOkCommand("PQTMCFGPROT", parms);
}

bool LG290P::getPortProtocols(int port, uint8_t &inputFlags, uint8_t &outputFlags)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",R,1,%d", port);
    bool ok = sendOkCommand("PQTMCFGPROT", parms);
    if (ok)
    {
        auto packet = getCommandResponse();
        ok = packet[2] == "1";
        inputFlags = atoi(packet[4].c_str());
        outputFlags = atoi(packet[5].c_str());
    }
    return ok;
}

bool LG290P::setPPS(uint16_t duration, bool alwaysOutput, bool positivePolarity)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,1,%d,%d,%d,0", duration, alwaysOutput ? 1 : 2, positivePolarity);
    return sendOkCommand("PQTMCFGPPS", parms);
}

bool LG290P::disablePPS()
{
    return sendOkCommand("PQTMCFGPPS", ",W,1,0");
}

bool LG290P::getPPS(bool &enabled, uint16_t &duration, bool &alwaysOutput, bool &positivePolarity)
{
    bool ret = sendCommand("PQTMCFGPPS", ",R,1");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK" && packet[2] == "1";
        enabled = packet[3] == "1";
        duration = atoi(packet[4].c_str());
        alwaysOutput = packet[5] == "1";
        positivePolarity = packet[6] == "1";
    }
    return ret;
}

bool LG290P::getConstellations(bool &enableGPS, bool &enableGLONASS, bool &enableGalileo, bool &enableBDS,
                               bool &enableQZSS, bool &enableNavIC)
{
    bool ret = sendCommand("PQTMCFGCNST", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        enableGPS = packet[2] == "1";
        enableGLONASS = packet[3] == "1";
        enableGalileo = packet[4] == "1";
        enableBDS = packet[5] == "1";
        enableQZSS = packet[6] == "1";
        enableNavIC = packet[7] == "1";
    }
    return ret;
}

bool LG290P::getSerialNumber(std::string &serial)
{
    bool ret = sendCommand("PQTMUNIQID");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        serial = packet[3];
    }
    return ret;
}

bool LG290P::getVersionInfo(std::string &version, std::string &buildDate, std::string &buildTime)
{
    bool ret = sendCommand("PQTMVERNO");
    if (ret)
    {
        auto packet = getCommandResponse();
        version = packet[1];
        buildDate = packet[2];
        buildTime = packet[3];
    }
    return ret;
}

bool LG290P::getFirmwareVersion(int &version)
{
    bool ok = true;
    int firmwareVersionMajor = 0;
    int firmwareVersionMinor = 0;
    ok &= getFirmwareVersionMajor(firmwareVersionMajor);
    ok &= getFirmwareVersionMinor(firmwareVersionMinor);

    version = (firmwareVersionMajor * 100) + firmwareVersionMinor; // v2.16 becomes 216

    return (ok);
}

bool LG290P::getFirmwareVersionMajor(int &majorVersion)
{
    majorVersion = 0; // Unknown

    std::string ver, buildDate, buildTime;
    bool ret = getVersionInfo(ver, buildDate, buildTime);
    if (ret && (ver.length() > strlen(firmwareVersionPrefix)))
    {
        char *spot = strstr(ver.c_str(), firmwareVersionPrefix);
        if (spot != NULL)
        {
            spot += strlen(firmwareVersionPrefix);
            majorVersion = atoi(spot);
            return (majorVersion > 0);
        }
    }

    return false;
}

bool LG290P::getFirmwareVersionMinor(int &minorVersion)
{
    minorVersion = 0; // Unknown

    std::string ver, buildDate, buildTime;
    bool ret = getVersionInfo(ver, buildDate, buildTime);
    if (ret && (ver.length() > strlen(firmwareVersionPrefix)))
    {
        char *spot = strstr(ver.c_str(), firmwareVersionPrefix);
        if (spot != NULL)
        {
            // LG290P03AANR##A?? - move 3 more than the prefix to get to the minor version after 'A'
            spot += (strlen(firmwareVersionPrefix) + 3);
            minorVersion = atoi(spot);
            return (minorVersion > 0);
        }
    }

    return false;
}

bool LG290P::getFixInterval(uint16_t &fixInterval)
{
    bool ret = sendCommand("PQTMCFGFIXRATE", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        fixInterval = atoi(packet[2].c_str());
    }
    return ret;
}

bool LG290P::setFixInterval(uint16_t fixInterval, bool resetAfter /* = true */)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d", fixInterval);
    bool ret = sendOkCommand("PQTMCFGFIXRATE", parms);
    if (resetAfter)
        ret = ret && save() && reset(); // Needs reset, not hotStart
    return ret;
}

bool LG290P::setMessageRate(const char *msgName, int rate, int msgVer)
{
    char parms[50];
    snprintf(parms, sizeof parms, msgVer == -1 ? ",W,%s,%d" : ",W,%s,%d,%d", msgName, rate, msgVer);
    bool ret = sendOkCommand("PQTMCFGMSGRATE", parms);

    // We internally track whether certain important sentences are enabled
    if (ret)
    {
        std::string str = msgName;
        if (str == "GGA")
            devState.ggaRate = rate;
        else if (str == "RMC")
            devState.rmcRate = rate;
        else if (str == "PQTMPVT")
            devState.pvtRate = rate;
        else if (str == "PQTMPL")
            devState.plRate = rate;
        else if (str == "PQTMSVINSTATUS")
            devState.svinstatusRate = rate;
        else if (str == "PQTMEPE")
            devState.epeRate = rate;
        else if (str == "GSV")
            devState.gsvRate = rate;
        else if (str == "GST")
            devState.gstRate = rate;
        else if (str == "PQTMPPPNAV")
            devState.pppnavRate = rate;
    }
    return ret;
}

// Set a message on a given port. Available in v1.4 and above.
// $PQTMCFGMSGRATE,W,1,2,GGA,1* - <PortType>,<PortID>,<MsgName>,<Rate>[,MsgVersion/Offset]
// Set port type (1 = UART), UART number (2 = UART2), message, rate
bool LG290P::setMessageRateOnPort(const char *msgName, int rate, int portNumber, int msgVer)
{
    char parms[50];
    snprintf(parms, sizeof parms, msgVer == -1 ? ",W,1,%d,%s,%d" : ",W,1,%d,%s,%d,%d", portNumber, msgName, rate,
             msgVer);
    bool ret = sendOkCommand("PQTMCFGMSGRATE", parms);

    // We internally track whether certain important sentences are enabled
    if (ret)
    {
        std::string str = msgName;
        if (str == "GGA")
            devState.ggaRate = rate;
        else if (str == "RMC")
            devState.rmcRate = rate;
        else if (str == "PQTMPVT")
            devState.pvtRate = rate;
        else if (str == "PQTMPL")
            devState.plRate = rate;
        else if (str == "PQTMSVINSTATUS")
            devState.svinstatusRate = rate;
        else if (str == "PQTMEPE")
            devState.epeRate = rate;
        else if (str == "GSV")
            devState.gsvRate = rate;
        else if (str == "GST")
            devState.gstRate = rate;
        else if (str == "PQTMPPPNAV")
            devState.pppnavRate = rate;
    }
    return ret;
}

bool LG290P::getMessageRate(const char *msgName, int &rate, int msgVer)
{
    char parms[50];
    snprintf(parms, sizeof parms, msgVer == -1 ? ",R,%s" : ",R,%s,%d", msgName, msgVer);
    bool ret = sendOkCommand("PQTMCFGMSGRATE", parms);
    if (ret)
    {
        auto packet = getCommandResponse();
        rate = atoi(packet[3].c_str());
    }
    return ret;
}

// Configures the elevation threshold for position engine. Available in v1.5 and above.
bool LG290P::setElevationAngle(int elevationAngle)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d", elevationAngle);
    return sendOkCommand("PQTMCFGELETHD", parms) && hotStart();
}

bool LG290P::getElevationAngle(int &elevationAngle)
{
    bool ret = sendCommand("PQTMCFGELETHD", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        elevationAngle = atoi(packet[2].c_str());
    }
    return ret;
}

// Configures the CNR threshold for position engine. Available in v1.5 and above.
bool LG290P::setCNR(float cnr)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%0.2f", cnr);
    return sendOkCommand("PQTMCFGCNRTHD", parms) && hotStart();
}

bool LG290P::getCNR(float &cnr)
{
    bool ret = sendCommand("PQTMCFGCNRTHD", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        cnr = atof(packet[2].c_str());
    }
    return ret;
}

// Configures the max differential age of RTK fix before the device will drop back to RTK float.
// Note: We assume Auto mode and Absolute mode when setting the timeout.
bool LG290P::setRtkDifferentialAge(uint16_t timeout)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,1,%d", timeout);
    return sendOkCommand("PQTMCFGRTK", parms) && hotStart();
}

bool LG290P::getRtkDifferentialAge(uint16_t &timeout)
{
    bool ret = sendCommand("PQTMCFGRTK", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        timeout = atoi(packet[4].c_str());
    }
    return ret;
}

// Configures the RTK differential source type.
// 0 = Auto, 1 = Normal, 2 = Wide Lane. Default is Auto.
bool LG290P::setRtkDifferentialSourceType(uint16_t srcType)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d", srcType);
    return sendOkCommand("PQTMCFGRTKSRCTYPE", parms) && hotStart();
}

bool LG290P::getRtkDifferentialSourceType(uint16_t &srcType)
{
    bool ret = sendCommand("PQTMCFGRTKSRCTYPE", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        srcType = atoi(packet[2].c_str());
    }
    return ret;
}

bool LG290P::setPppSettings(int mode, int datum, int timeout, float horstd, float verstd)
{
    // CFGPPP does not require reset to take effect
    bool ret = true;
    if (mode == 0)
        ret = sendOkCommand("PQTMCFGPPP", ",W,0"); // $PQTMCFGPPP,W,0,0,0* throws error if other params sent
    else
    {
        char parms[100];
        snprintf(parms, sizeof parms, ",W,%d,%d,%d,%0.2f,%0.2f", mode, datum, timeout, horstd, verstd);
        ret = sendOkCommand("PQTMCFGPPP", parms);
    }
    return ret;
}

bool LG290P::getPppSettings(int &mode, int &datum, int &timeout, float &horstd, float &verstd)
{
    bool ret = sendCommand("PQTMCFGPPP", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        if (ret)
        {
            mode = atoi(packet[2].c_str());
            datum = atoi(packet[3].c_str());
            timeout = atoi(packet[4].c_str());
            horstd = atof(packet[5].c_str());
            verstd = atof(packet[6].c_str());
        }
    }
    return ret;
}

bool LG290P::scanForMsgsEnabled()
{
    bool ok = getMessageRate("GGA", devState.ggaRate);
    ok = ok && getMessageRate("RMC", devState.rmcRate);
    ok = ok && getMessageRate("PQTMEPE", devState.epeRate, 2);
    ok = ok && getMessageRate("PQTMPVT", devState.pvtRate, 1);
    ok = ok && getMessageRate("PQTMPL", devState.plRate, 1);
    ok = ok && getMessageRate("GSV", devState.gsvRate);

    // PPPNAV not available on firmware < 201
    if (firmwareVersionInt >= 201)
        ok = ok && getMessageRate("PQTMPPPNAV", devState.pppnavRate, 1);

    // GST is not available on firmware < 104
    if (firmwareVersionInt >= 104)
        ok = ok && getMessageRate("GST", devState.gstRate);

    // This is a special message. getMessageRate might fail if in ROVER mode
    getMessageRate("PQTMSVINSTATUS", devState.svinstatusRate, 1);
    return ok;
}

void LG290P::ensureMsgEnabled(bool enabled, const char *msg, int msgVer /* = -1 */)
{
    if (!enabled)
    {
        debugPrintf("Forcing enable of %s sentence", msg);
        setMessageRate(msg, 1, msgVer);
    }
}

bool LG290P::nmeaSubscribe(const char *msgName, nmeaCallback callback)
{
    nmeaSubscriptions[msgName] = callback;
    return true;
}

bool LG290P::nmeaUnsubscribe(const char *msgName)
{
    nmeaSubscriptions.erase(msgName);
    return true;
}

bool LG290P::nmeaSubscribeAll(nmeaCallback callback)
{
    nmeaAllSubscribe = callback;
    return true;
}

bool LG290P::nmeaUnsubscribeAll()
{
    nmeaAllSubscribe = nullptr;
    return true;
}

bool LG290P::rtcmSubscribe(uint16_t type, rtcmCallback callback)
{
    rtcmSubscriptions[type] = callback;
    return true;
}

bool LG290P::rtcmUnsubscribe(uint16_t type)
{
    rtcmSubscriptions.erase(type);
    return true;
}

bool LG290P::rtcmSubscribeAll(rtcmCallback callback)
{
    rtcmAllSubscribe = callback;
    return true;
}

bool LG290P::rtcmUnsubscribeAll()
{
    rtcmAllSubscribe = nullptr;
    return true;
}

void LG290P::clearAll()
{
    pvtDomain.clear();
    satelliteDomain.clear();
    nmeaCounters.clear();
    rtcmCounters.clear();
    epeDomain.clear();
    plDomain.clear();
    svinStatusDomain.clear();
}

bool LG290P::genericReset(const char *resetCmd)
{
    clearAll();

    // Do a software reset, wait for reconnection, then rescan which messages are enabled
    return sendCommandNoResponse(resetCmd) && isConnected() && scanForMsgsEnabled();
}

bool LG290P::setConstellations(bool enableGPS, bool enableGLONASS, bool enableGalileo, bool enableBDS, bool enableQZSS,
                               bool enableNavIC)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d,%d,%d,%d,%d,%d", enableGPS, enableGLONASS, enableGalileo, enableBDS,
             enableQZSS, enableNavIC);
    return sendOkCommand("PQTMCFGCNST", parms) && save() && hotStart();
}

bool LG290P::disableEngine()
{
    return sendOkCommand("PQTMGNSSSTOP");
}

bool LG290P::enableEngine()
{
    return sendOkCommand("PQTMGNSSSTART");
}

bool LG290P::save()
{
    return sendOkCommand("PQTMSAVEPAR");
}

bool LG290P::factoryRestore()
{
    return sendOkCommand("PQTMRESTOREPAR");
}

// Abstraction of the serial interface
// Useful if we ever need to support SoftwareSerial
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Enable printfs to various endpoints
// https://stackoverflow.com/questions/42131753/wrapper-for-printf
void LG290P::debugPrintf(const char *format, ...)
{
    if (_debugPort == nullptr)
        return;

    va_list args;
    va_start(args, format);

    va_list args2;
    va_copy(args2, args);
    char buf[vsnprintf(nullptr, 0, format, args) + sizeof("\r\n")];

    vsnprintf(buf, sizeof buf, format, args2);

    // Add CR+LF
    buf[sizeof(buf) - 3] = '\r';
    buf[sizeof(buf) - 2] = '\n';
    buf[sizeof(buf) - 1] = '\0';

    _debugPort->write(buf, strlen(buf));

    va_end(args);
    va_end(args2);
}

// Discards any characters sitting in RX buffer
void LG290P::clearBuffer()
{
    while (serialAvailable())
        serialRead();
}

uint16_t LG290P::serialAvailable()
{
    if (_hwSerialPort != nullptr)
    {
        return (_hwSerialPort->available());
    }
    return (0);
}

uint8_t LG290P::serialRead()
{
    if (_hwSerialPort != nullptr)
    {
        return (_hwSerialPort->read());
    }
    return (0);
}

// Correctly format command $PQTM with $ and checksum,
// then send it!
bool LG290P::transmit(const char *command, const char *parms)
{
    if (!_hwSerialPort || command == nullptr)
        return false;

    std::string cmd = command;
    cmd += parms;
    if (cmd.empty())
        return false;

    // Prepend $ if lacking
    if (cmd[0] != '$')
        cmd = "$" + cmd;

    // Calculate NMEA checksum
    uint8_t chk = 0;
    for (int i = 1; i < cmd.length(); ++i)
        chk ^= cmd[i];

    // Append checksum
    char buf[32];
    sprintf(buf, "*%02X", chk);
    cmd += buf;

    // send with newlines
    debugPrintf("...transmit '%s'", cmd.c_str());
    _hwSerialPort->print(cmd.c_str());
    _hwSerialPort->print("\r\n");

    return true;
}

// Send a $PQTM query string to the LG290P and wait for response
// If you want to see the response string, you can get it from getCommandResponse()
bool LG290P::sendCommand(const char *command, const char *parms, uint16_t maxWaitMs, bool waitForResponse)
{
    if (lg290PLibrarySemaphoreBlock)
        return false;

    bool success = false;

    debugPrintf("sendCommand(\"%s\", \"%s\")", command, parms);
    commandName = command[0] == '$' ? command + 1 : command;
    commandResponse = LG290P_RESULT_RESPONSE_COMMAND_WAITING; // Reset

    lg290PLibrarySemaphoreBlock = true; // Prevent external tasks from harvesting serial data

    // add $ and checksum and transmit
    if (!transmit(command, parms))
        return false;

    if (waitForResponse)
        debugPrintf("...sendCommand: waiting for response for %s", command);

    // Feed the parser until we see a response to the command
    for (unsigned long start = millis(); millis() - start < maxWaitMs;)
    {
        update(); // Will call LG290PProcessMessage()

        if (commandResponse == LG290P_RESULT_RESPONSE_COMMAND_OK)
        {
            debugPrintf("...sendCommand: Matching Response received");
            success = true;
            break;
        }

        if (commandResponse == LG290P_RESULT_RESPONSE_COMMAND_ERROR)
        {
            debugPrintf("...sendCommand: Matching response NOT received");
            break;
        }
    }

    if (commandResponse == LG290P_RESULT_RESPONSE_COMMAND_WAITING)
    {
        if (waitForResponse)
            debugPrintf("...sendCommand: TIMEOUT: no response received");
        else
            success = true;
    }
    commandName.clear();
    lg290PLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
    return success;
}

bool LG290P::sendCommandNoResponse(const char *command, uint16_t maxWaitMs)
{
    return sendCommand(command, "", maxWaitMs, false);
}

bool LG290P::sendCommandLine(const char *commandline, uint16_t maxWaitMs)
{
    std::string cmdline = commandline;
    size_t commaPos = cmdline.find(',');
    std::string cmd = commaPos == cmdline.npos ? cmdline : cmdline.substr(0, commaPos);
    std::string parms = commaPos == cmdline.npos ? "" : cmdline.substr(commaPos);

    return sendCommand(cmd.c_str(), parms.c_str(), maxWaitMs);
}

bool LG290P::sendOkCommand(const char *command, const char *parms, uint16_t maxWaitMs)
{
    if (!sendCommand(command, parms, maxWaitMs))
        return false;

    NmeaPacket &response = getCommandResponse();
    bool okFound = response[1] == "OK";

    debugPrintf("...sendOkCommand, OK %sfound", okFound ? "" : "NOT ");

    return okFound;
}

std::list<LG290P::satinfo> LG290P::getVisibleSats(const char *talker /* = nullptr */)
{
    std::list<LG290P::satinfo> ret;

    ensureGsvEnabled();

    // Get all the satellites visible?
    if (talker == nullptr)
    {
        for (auto &item : satelliteDomain)
            ret.insert(ret.end(), item.second.begin(), item.second.end());
    }

    // ... or just the ones from a specific Talker?
    else
    {
        auto item = satelliteDomain.find(talker);
        if (item != satelliteDomain.end())
            ret.insert(ret.end(), item->second.begin(), item->second.end());
    }
    return ret;
}

bool LG290P::getSurveyDetails(int &mode, int &positionTimes, double &accuracyLimit, double &ecefX, double &ecefY,
                              double &ecefZ)
{
    bool ret = sendOkCommand("PQTMCFGSVIN", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        mode = atoi(packet[2].c_str());
        positionTimes = atof(packet[3].c_str());
        accuracyLimit = atof(packet[4].c_str());
        ecefX = atof(packet[5].c_str());
        ecefY = atof(packet[6].c_str());
        ecefZ = atof(packet[7].c_str());
    }
    return ret;
}

uint8_t LG290P::getSurveyMode()
{
    if (sendOkCommand("PQTMCFGSVIN", ",R"))
    {
        auto packet = getCommandResponse();
        return (uint8_t)atoi(packet[2].c_str());
    }
    return SURVEYDISABLED;
}

bool LG290P::setSurveyInMode(int positionTimes, double accuracyLimit /* = 0 */, bool resetAfter /* = true */)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,%d,%f,0.00,0.00,0.00", positionTimes, accuracyLimit);
    bool ok = sendOkCommand("PQTMCFGSVIN", parms);
    if (resetAfter)
        ok = ok && save() && reset();
    return ok;
}

bool LG290P::setSurveyFixedMode(double ecefX, double ecefY, double ecefZ, bool resetAfter /* = true */)
{
    char parms[100];
    snprintf(parms, sizeof parms, ",W,2,0,0,%f,%f,%f", ecefX, ecefY, ecefZ);
    bool ok = sendOkCommand("PQTMCFGSVIN", parms);
    if (resetAfter)
        ok = ok && save() && reset();
    return ok;
}

bool LG290P::disableSurveyInMode(bool resetAfter /* = true */)
{
    bool ok = sendOkCommand("PQTMCFGSVIN", ",W,0,0,0,0,0,0");
    if (resetAfter)
        ok = ok && save() && reset();
    return ok;
}

bool LG290P::setNavMode(uint16_t mode, bool resetAfter /* = true */)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d", mode);
    bool ret = sendOkCommand("PQTMCFGNAVMODE", parms);
    if (resetAfter)
        ret = ret && save() && reset();
    return ret;
}

bool LG290P::getNavMode(uint16_t &mode)
{
    bool ret = sendCommand("PQTMCFGNAVMODE", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        if (ret)
            mode = atoi(packet[2].c_str());
    }
    return ret;
}

// Main handler and RAM inits
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Cracks an NMEA sentence into the applicable container
// The sentence might be a normal NMEA one like $GPGGA or a custom one like $PQTMVER. In
// the former case, the first term is cracked into the "Talker Id" (GP) and "Sentence Id" (GGA)
void LG290P::nmeaHandler(SEMP_PARSE_STATE *parse)
{
    // Is this a command response?
    std::string sentence = (const char *)parse->buffer;
    if (sentence.length() >= 2 && sentence.substr(sentence.length() - 2) == "\r\n")
        // if (sentence.ends_with("\r\n"))
        sentence.erase(sentence.length() - 2);

    NmeaPacket nmea = NmeaPacket::FromString(sentence);
    auto id = nmea.SentenceId();

    // Do $PQTM strings later
    if (sentence.substr(0, 2) != "$P")
    {
        if (nmea.IsValid())
        {
            if (nmea.ChecksumValid())
            {
                if (id == "RMC")
                {
                    lastUpdatePvtDomain = millis(); // Update stale marker
                    nmea.processRMC(ptrLG290P->pvtDomain);
                }

                else if (id == "GGA")
                {
                    lastUpdatePvtDomain = millis(); // Update stale marker
                    nmea.processGGA(ptrLG290P->pvtDomain);
                }

                else if (id == "GST")
                {
                    lastUpdatePvtDomain = millis(); // Update stale marker
                    PvtDomain &pvt = ptrLG290P->pvtDomain;
                    pvt.rmsPseudorangeResidual = strtod(nmea[2].c_str(), NULL);
                    pvt.latitudeError = strtod(nmea[6].c_str(), NULL);
                    pvt.longitudeError = strtod(nmea[7].c_str(), NULL);
                    pvt.heightError = strtod(nmea[8].c_str(), NULL);
                }

                else if (id == "GSV")
                {
                    uint16_t msgNo = (uint16_t)strtoul(nmea[2].c_str(), NULL, 10);
                    uint16_t svsInView = (uint16_t)strtoul(nmea[3].c_str(), NULL, 10);
                    std::string talker = nmea.TalkerId();
                    unsigned long now = millis();
                    auto &thisSet = satelliteStaging[talker];

                    // should we publish the latest batch of satellites for this talker id?
                    if (msgNo == 1 && now - satelliteUpdateTime[talker] > 900)
                    {
                        hasNewSatellites = true;
                        satelliteDomain[talker] = thisSet;
                        thisSet.clear();
                        satelliteUpdateTime[talker] = now;
                    }

                    // log_d("GSV: count: %d no: %d in view: %d", msgCount, msgNo, svsInView);
                    for (int i = 0; i < 4 && 4 * (msgNo - 1) + i < svsInView; ++i)
                    {
                        satinfo sat;
                        sat.prn = (uint16_t)strtoul(nmea[4 + 4 * i].c_str(), NULL, 10);
                        sat.elev = (uint16_t)strtoul(nmea[5 + 4 * i].c_str(), NULL, 10);
                        sat.azimuth = (uint16_t)strtoul(nmea[6 + 4 * i].c_str(), NULL, 10);
                        sat.snr = (uint16_t)strtoul(nmea[7 + 4 * i].c_str(), NULL, 10);
                        strncpy(sat.talker, talker.substr(0, 2).c_str(), sizeof sat.talker);
                        thisSet.insert(sat);
                    }
                }
            }
            else
            {
                ptrLG290P->debugPrintf("BAD CHECKSUM: %s\r\n", sentence.c_str());
            }
        }
        else
        {
            ptrLG290P->debugPrintf("NOT VALID: %s\r\n", sentence.c_str());
        }
    }

    // Handle $PQTM response messages
    else
    {
        // Is this the response to a command we issued with sendCommand?
        if (id == ptrLG290P->commandName)
        {
            // Display the command response
            ptrLG290P->debugPrintf("...sendCommand: (main loop) Expected command response found %s", parse->buffer);
            ptrLG290P->pqtmResponse = nmea;
            ptrLG290P->commandResponse = LG290P_RESULT_RESPONSE_COMMAND_OK;
        }

        else if (id == "PQTMPVT")
        {
            lastUpdatePvtDomain = millis(); // Update stale marker
            PvtDomain &pvt = ptrLG290P->pvtDomain;
            pvt.timeOfWeek = atoi(nmea[2].c_str());
            uint32_t d = atoi(nmea[3].c_str());
            pvt.year = d / 10000;
            pvt.month = (d / 100) % 100;
            pvt.day = d % 100;
            NmeaPacket::parseTime(nmea[4], pvt.hour, pvt.minute, pvt.second, pvt.nanosecond);
            // 5 is reserved
            // 6 is fix type -- maybe don't use because more limited than GGA?
            // pvt.quality = nmea[6].empty() ? '0' : nmea[6][0];
            pvt.satellitesUsed = atoi(nmea[7].c_str());
            pvt.leapSeconds = atoi(nmea[8].c_str());
            pvt.latitude = atof(nmea[9].c_str());
            pvt.longitude = atof(nmea[10].c_str());
            pvt.altitude = atof(nmea[11].c_str());
            pvt.geoidalSeparation = atof(nmea[12].c_str());
            pvt.nvelocity = atof(nmea[13].c_str());
            pvt.evelocity = atof(nmea[14].c_str());
            pvt.dvelocity = atof(nmea[15].c_str());
            pvt.groundSpeed = atof(nmea[16].c_str());
            pvt.course = atof(nmea[17].c_str());
            pvt.hdop = atof(nmea[18].c_str());
            pvt.pdop = atof(nmea[19].c_str());
            pvtDomain.newDataAvailable = true;
        }

        else if (id == "PQTMSVINSTATUS")
        {
            svinStatusDomain.validity = atoi(nmea[3].c_str());
            svinStatusDomain.observations = atoi(nmea[6].c_str());
            svinStatusDomain.cfgDur = atoi(nmea[7].c_str());
            svinStatusDomain.meanX = atof(nmea[8].c_str());
            svinStatusDomain.meanY = atof(nmea[9].c_str());
            svinStatusDomain.meanZ = atof(nmea[10].c_str());
            svinStatusDomain.meanAcc = atof(nmea[11].c_str());
        }

        else if (id == "PQTMEPE")
        {
            // handle 5 components of Horizontal Position Accuracy
            epeDomain.errorNorth = atof(nmea[2].c_str());
            epeDomain.errorEast = atof(nmea[3].c_str());
            epeDomain.errorDown = atof(nmea[4].c_str());
            epeDomain.error2D = atof(nmea[5].c_str());
            epeDomain.error3D = atof(nmea[6].c_str());
        }

        else if (id == "PQTMPL")
        {
            plDomain.probUncertainty = atof(nmea[3].c_str());
            plDomain.protectionLevelNorth = atof(nmea[6].c_str());
            plDomain.protectionLevelEast = atof(nmea[7].c_str());
            plDomain.protectionLevelDown = atof(nmea[8].c_str());
            plDomain.protectionLevelNorthVelocity = atof(nmea[9].c_str());
            plDomain.protectionLevelEastVelocity = atof(nmea[10].c_str());
            plDomain.protectionLevelDownVelocity = atof(nmea[11].c_str());
            plDomain.protectionLevelTime = atof(nmea[14].c_str());
        }

        else if (id == "PQTMPPPNAV")
        {
            pppNavDomain.datumId = atoi(nmea[9].c_str());
            pppNavDomain.solType = atoi(nmea[11].c_str());
            pppNavDomain.diffId = atoi(nmea[24].c_str());
            pppNavDomain.diffAge = atoi(nmea[25].c_str());
        }

#if false
        else if (!ptrLG290P->commandName.empty())
        {
            // Display the command response
            ptrLG290P->debugPrintf("LG290P Lib: Unknown command response: %s", parse->buffer);
            ptrLG290P->debugPrintf("LG290P Lib: Looking for command: %s", ptrLG290P->commandName.c_str());
        }
#endif
    }

    if (id != "")
    {
        nmeaCounters[id]++;

        // Is there a callback registered for this id?
        auto iterator = nmeaSubscriptions.find(id);
        if (iterator != nmeaSubscriptions.end())
            (*iterator->second)(nmea); // call it!
    }

    // Is there a general callback registered?
    if (nmeaAllSubscribe != nullptr)
        nmeaAllSubscribe(nmea); // call it!
}

int64_t RtcmPacket::extract_38bit_signed(int bit_offset)
{
    // Extract 38-bit value starting from the given bit_offset
    int64_t value = 0;
    int byte_offset = bit_offset / 8;
    int bit_in_byte = bit_offset % 8;

    // We need to grab up to 6 bytes and mask out the unused 2 bits.
    for (int i = 0; i < 6; ++i)
    {
        value <<= 8;
        value |= (int64_t)(payload[byte_offset + i]);
    }

    // Shift right to discard the unwanted bits and align to 38-bit value
    value >>= (10 - bit_in_byte);

    // Mask to keep only 38 bits
    value &= 0x3FFFFFFFFFLL; // 38-bit mask (0x3FFFFFFFFF is 38 ones in binary)

    // Check if the sign bit (38th bit) is set and extend the sign for 64-bit int
    if (value & (1LL << 37))
    {
        value |= ~((1LL << 38) - 1); // Extend the sign
    }

    return value;
}

/* static */
bool RtcmPacket::FromBuffer(uint8_t *buffer, size_t bufferLen, RtcmPacket &result)
{
    bool good = bufferLen > 6 && buffer[0] == 0xD3;
    if (good)
    {
        result.payloadLen = (buffer[1] << 8) | buffer[2];
        good = result.payloadLen + 6 == bufferLen;
    }

    if (good)
    {
        result.type = (buffer[3] << 4) | (buffer[4] >> 4);
        result.buffer = buffer;
        result.bufferlen = bufferLen;
        result.payload = buffer + 3;
    }
    return good;
}

// Cracks an RTCM packet into the applicable container
void LG290P::rtcmHandler(SEMP_PARSE_STATE *parse)
{
    RtcmPacket packet;
    if (RtcmPacket::FromBuffer(parse->buffer, parse->length, packet))
    {
        rtcmCounters[packet.type]++;

        if (packet.type == 1005)
        {
            rtcmDomain.ecefX = packet.getEcefX();
            rtcmDomain.ecefY = packet.getEcefY();
            rtcmDomain.ecefZ = packet.getEcefZ();
            lastUpdateEcef = millis();
        }

        // handle specific and general callbacks
        // If user is subscribed for this packet, call the callback
        if (rtcmSubscriptions.count(packet.type) > 0)
            rtcmSubscriptions[packet.type](packet);
        if (rtcmAllSubscribe != nullptr)
            rtcmAllSubscribe(packet);
    }
}

// All the general gets and sets
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

bool LG290P::isNewSnapshotAvailable()
{
    ensurePvtEnabled();
    bool r = pvtDomain.newDataAvailable;
    pvtDomain.newDataAvailable = false;
    return r;
}

bool LG290P::isNewSatelliteInfoAvailable()
{
    ensureGsvEnabled();
    bool r = hasNewSatellites;
    hasNewSatellites = false;
    return r;
}

uint16_t LG290P::getSatellitesInViewCount()
{
    uint16_t count = 0;
    for (auto &item : satelliteDomain)
        count += item.second.size();
    return count;
}

// Return the number of millis since last update
uint32_t LG290P::getPVTDomainAgeMs()
{
    return millis() - lastUpdatePvtDomain;
}

/* static */
void LG290P::geodeticToEcef(double lat, double lon, double alt, double &xOut, double &yOut, double &zOut)
{
    const double WGS84_A = 6378137; // https://geographiclib.sourceforge.io/html/Constants_8hpp_source.html
    const double WGS84_E =
        0.081819190842622; // http://docs.ros.org/en/hydro/api/gps_common/html/namespacegps__common.html
                           // and https://gist.github.com/uhho/63750c4b54c7f90f37f958cc8af0c718
    double clat = cos(lat * DEG_TO_RAD);
    double slat = sin(lat * DEG_TO_RAD);
    double clon = cos(lon * DEG_TO_RAD);
    double slon = sin(lon * DEG_TO_RAD);

    double N = WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * slat * slat);

    xOut = (N + alt) * clat * clon;
    yOut = (N + alt) * clat * slon;
    zOut = (N * (1.0 - WGS84_E * WGS84_E) + alt) * slat;
}

// Convert ECEF to LLH (geodetic)
// From: https://danceswithcode.net/engineeringnotes/geodetic_to_ecef/geodetic_to_ecef.html
/* static */
void LG290P::ecefToGeodetic(double x, double y, double z, double &latOut, double &lonOut, double &altOut)
{
    double a = 6378137.0;              // WGS-84 semi-major axis
    double e2 = 6.6943799901377997e-3; // WGS-84 first eccentricity squared
    double a1 = 4.2697672707157535e+4; // a1 = a*e2
    double a2 = 1.8230912546075455e+9; // a2 = a1*a1
    double a3 = 1.4291722289812413e+2; // a3 = a1*e2/2
    double a4 = 4.5577281365188637e+9; // a4 = 2.5*a2
    double a5 = 4.2840589930055659e+4; // a5 = a1+a3
    double a6 = 9.9330562000986220e-1; // a6 = 1-e2

    double zp, w2, w, r2, r, s2, c2, s, c, ss;
    double g, rg, rf, u, v, m, f, p;

    zp = abs(z);
    w2 = x * x + y * y;
    w = sqrt(w2);
    r2 = w2 + z * z;
    r = sqrt(r2);
    lonOut = atan2(y, x); // Lon (final)

    s2 = z * z / r2;
    c2 = w2 / r2;
    u = a2 / r;
    v = a3 - a4 / r;
    if (c2 > 0.3)
    {
        s = (zp / r) * (1.0 + c2 * (a1 + u + s2 * v) / r);
        latOut = asin(s); // Lat
        ss = s * s;
        c = sqrt(1.0 - ss);
    }
    else
    {
        c = (w / r) * (1.0 - s2 * (a5 - u - c2 * v) / r);
        latOut = acos(c); // Lat
        ss = 1.0 - c * c;
        s = sqrt(ss);
    }

    g = 1.0 - e2 * ss;
    rg = a / sqrt(g);
    rf = a6 * rg;
    u = w - rg * c;
    v = zp - rf * s;
    f = c * u + s * v;
    m = c * v - s * u;
    p = m / (rf / g + f);
    latOut = latOut + p;      // Lat
    altOut = f + m * p / 2.0; // Altitude
    if (z < 0.0)
    {
        latOut *= -1.0; // Lat
    }

    latOut *= RAD_TO_DEG; // Convert to degrees
    lonOut *= RAD_TO_DEG;
}

NmeaPacket NmeaPacket::FromString(const std::string &str)
{
    NmeaPacket s;
    s.charCount = str.length();
    size_t start = 0;
    size_t delimiterPos = str.find_first_of(",*");
    unsigned long calculatedChksum = 0;
    // log_d("Parsing sentence %s", str.c_str());

    while (true)
    {
        std::string token =
            delimiterPos != std::string::npos ? str.substr(start, delimiterPos - start) : str.substr(start);
        s.fields.push_back(token);
        for (auto c : token.substr(s.fields.size() == 1 && s.fields[0].substr(0, 1) == "$" ? 1 : 0))
            calculatedChksum ^= c;

        if (delimiterPos == std::string::npos)
        {
            s.hasChecksum = s.checksumValid = false;
            // log_d("Doesn't have any * checksum");
            break;
        }

        if (str[delimiterPos] == '*')
        {
            token = str.substr(delimiterPos + 1);
            s.fields.push_back(token);
            if (token.find_first_not_of("0123456789ABCDEFabcdef") != std::string::npos)
            {
                s.hasChecksum = s.checksumValid = false;
                // log_e("Bad format checksum: '%s'", token.c_str());
                // log_e("Sentence was '%s'", str.c_str());
            }
            else
            {
                unsigned long suppliedChecksum = strtoul(token.c_str(), NULL, 16);
                s.hasChecksum = true;
                s.checksumValid = suppliedChecksum == calculatedChksum;
            }
            break;
        }

        calculatedChksum ^= ',';
        start = delimiterPos + 1;
        delimiterPos = str.find_first_of(",*", start);
    }

    return s;
}

std::string NmeaPacket::ToString() const
{
    std::string str;
    for (int i = 0; i < fields.size(); ++i)
    {
        str += fields[i];
        if (i != fields.size() - 1)
            str += i == fields.size() - 2 ? '*' : ',';
    }
    return str;
}

std::string NmeaPacket::TalkerId() const
{
    if (!IsValid() || fields[0].empty() || fields[0][0] != '$')
        return "";

    // The Talker Id is the first two letters following the $ (four if PQTM)
    return fields[0].substr(0, 5) == "$PQTM" ? "" : fields[0].substr(1, 2);
}

std::string NmeaPacket::SentenceId() const
{
    if (!IsValid() || fields[0].empty() || fields[0][0] != '$')
        return "";

    // The Sentence Id is everything after the Talker Id, unless it's a PQTM
    return fields[0].substr(0, 5) == "$PQTM" ? fields[0].substr(1) : fields[0].length() >= 3 ? fields[0].substr(3) : "";
}

void NmeaPacket::processGGA(PvtDomain &snapshot)
{
    if (fields.size() >= 10)
    {
        NmeaPacket::parseTime(fields[1], snapshot.hour, snapshot.minute, snapshot.second, snapshot.nanosecond);
        NmeaPacket::parseLocation(fields[2], fields[3], fields[4], fields[5], snapshot.latitude, snapshot.longitude);
        NmeaPacket::parseQuality(fields[6], snapshot.quality);
        NmeaPacket::parseSatelliteCount(fields[7], snapshot.satellitesUsed);
        NmeaPacket::parseHdop(fields[8].c_str(), snapshot.hdop);
        NmeaPacket::parseAltitude(fields[9].c_str(), snapshot.altitude);
        snapshot.newDataAvailable = true;
    }
}

void NmeaPacket::processRMC(PvtDomain &snapshot)
{
    if (fields.size() >= 10)
    {
        NmeaPacket::parseTime(fields[1], snapshot.hour, snapshot.minute, snapshot.second, snapshot.nanosecond);
        NmeaPacket::parseFixStatus(fields[2], snapshot.fixStatus);
        NmeaPacket::parseLocation(fields[3], fields[4], fields[5], fields[6], snapshot.latitude, snapshot.longitude);
        NmeaPacket::parseSpeed(fields[7].c_str(), snapshot.groundSpeed);
        NmeaPacket::parseCourse(fields[8].c_str(), snapshot.course);
        NmeaPacket::parseDate(fields[9], snapshot.year, snapshot.month, snapshot.day);
        snapshot.newDataAvailable = true;
    }
}

void NmeaPacket::parseTime(const std::string &term, uint8_t &hour, uint8_t &minute, uint8_t &second, uint32_t &nanos)
{
    int32_t v = parseDecimal(term);
    hour = v / 1000000;
    minute = (v / 10000) % 100;
    second = (v / 100) % 100;
    nanos = 10000000 * (v % 100);
}

int32_t NmeaPacket::parseDecimal(const std::string &term)
{
    const char *t = term.c_str();
    bool negative = *t == '-';
    if (negative)
        ++t;
    int32_t ret = 100 * (int32_t)atol(t);
    while (isdigit(*t))
        ++t;
    if (*t == '.' && isdigit(t[1]))
    {
        ret += 10 * (t[1] - '0');
        if (isdigit(t[2]))
            ret += t[2] - '0';
    }
    return negative ? -ret : ret;
}

void NmeaPacket::parseDate(const std::string &term, uint16_t &year, uint8_t &month, uint8_t &day)
{
    uint32_t v = atol(term.c_str());
    year = 2000 + v % 100;
    month = (v / 100) % 100;
    day = v / 10000;
}

void NmeaPacket::parseLocation(const std::string &termLat, const std::string &termNS, const std::string &termLong,
                               const std::string &termEW, double &latitude, double &longitude)
{
    parseDegrees(termLat.c_str(), termNS.c_str(), latitude);
    parseDegrees(termLong.c_str(), termEW.c_str(), longitude);
}

void NmeaPacket::parseDegrees(const char *degTerm, const char *nsewTerm, double &angle)
{
    uint32_t leftOfDecimal = (uint32_t)atol(degTerm);
    uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
    uint32_t multiplier = 10000000UL;
    uint32_t tenMillionthsOfMinutes = minutes * multiplier;
    uint16_t deg = (int16_t)(leftOfDecimal / 100);
    uint32_t billionths = 0;
    bool negative = false;

    while (isdigit(*degTerm))
        ++degTerm;

    if (*degTerm == '.')
        while (isdigit(*++degTerm))
        {
            multiplier /= 10;
            tenMillionthsOfMinutes += (*degTerm - '0') * multiplier;
        }

    billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
    negative = *nsewTerm == 'W' || *nsewTerm == 'S';
    angle = deg + billionths / 1000000000.0;
    if (negative)
        angle = -angle;
}

void NmeaPacket::parseQuality(const std::string &term, char &quality)
{
    quality = term.empty() ? '0' : term[0];
}

void NmeaPacket::parseSatelliteCount(const std::string &term, uint8_t &used_satellites)
{
    used_satellites = atoi(term.c_str());
}

void NmeaPacket::parseFixStatus(const std::string &term, char &fixStatus)
{
    fixStatus = term.empty() ? 'N' : term[0];
}

void NmeaPacket::parseSpeed(const std::string &term, double &speed)
{
    static constexpr double _GPS_MPS_PER_KNOT = 0.51444444;
    speed = _GPS_MPS_PER_KNOT * parseDecimal(term) / 100.0;
}

void NmeaPacket::parseCourse(const std::string &term, double &course)
{
    course = parseDecimal(term) / 100.0;
}

void NmeaPacket::parseHdop(const std::string &term, double &hdop)
{
    hdop = parseDecimal(term) / 100.0;
}

void NmeaPacket::parseAltitude(const std::string &term, double &altitude)
{
    altitude = parseDecimal(term) / 100.0;
}

//----------------------------------------
// Firmware Update Support
//----------------------------------------

const uint32_t LG290P::_fwCrc32Table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3, 0x0edb8832,
    0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7, 0x136c9856, 0x646ba8c0, 0xfd62f97a,
    0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3,
    0x45df5c75, 0xdcd60dcf, 0xabd13d59, 0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab,
    0xb6662d3d, 0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01, 0x6b6b51f4,
    0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65, 0x4db26158, 0x3ab551ce, 0xa3bc0074,
    0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525,
    0x206f85b3, 0xb966d409, 0xce61e49f, 0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615,
    0x73dc1683, 0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7, 0xfed41b76,
    0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b, 0xd80d2bda, 0xaf0a1b4c, 0x36034af6,
    0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7,
    0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d, 0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7,
    0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45, 0xa00ae278,
    0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9, 0xbdbdf21c, 0xcabac28a, 0x53b39330,
    0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
};

// Seed a CRC32 computation with the 4-byte little-endian firmware size prefix required by the bootloader protocol
uint32_t LG290P::initFirmwareCrc32(uint32_t firmwareSize)
{
    uint8_t buf[4] = {(uint8_t)firmwareSize, (uint8_t)(firmwareSize >> 8), (uint8_t)(firmwareSize >> 16),
                      (uint8_t)(firmwareSize >> 24)};
    return computeFirmwareCrc32(0, buf, 4);
}

// Chainable CRC32; pass previous result as prev to continue over more data, or 0 to start fresh
uint32_t LG290P::computeFirmwareCrc32(uint32_t prev, const uint8_t *data, size_t length)
{
    uint32_t crc = prev ^ 0xffffffffUL;
    while (length--)
        crc = _fwCrc32Table[(crc ^ *data++) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xffffffffUL;
}

// Free heap buffers allocated for an active firmware update and zero all update state
void LG290P::fwCleanup()
{
    if (_fw.accumBuf)
    {
        free(_fw.accumBuf);
        _fw.accumBuf = nullptr;
    }
    if (_fw.response)
    {
        free(_fw.response);
        _fw.response = nullptr;
    }
    _fw.firmwareSize = 0;
    _fw.firmwareCrc = 0;
    _fw.packetNumber = 0;
    _fw.packetCount = 0;
    _fw.accumLen = 0;
    _fw.responseLen = 0;
    _fw.cmdResponseLen = 0;
    _fw.peekAvail = false;
    _fw.peekByte = 0;
}

// Block up to timeoutMs waiting for one byte; returns 1 on success, 0 on timeout
int LG290P::fwSerialWaitByte(uint8_t *b, uint32_t timeoutMs)
{
    uint32_t start = millis();
    while (millis() - start < timeoutMs)
    {
        if (serialAvailable())
        {
            *b = serialRead();
            return 1;
        }
    }
    return 0;
}

// Read one byte, consuming the peek-back buffer first if it holds a byte
int LG290P::fwReadByte(uint8_t *b)
{
    if (_fw.peekAvail)
    {
        _fw.peekAvail = false;
        *b = _fw.peekByte;
        return 1;
    }
    return fwSerialWaitByte(b, 250);
}

// Push one byte back so the next fwReadByte() returns it without hitting the serial port
void LG290P::fwPushBack(uint8_t b)
{
    _fw.peekAvail = true;
    _fw.peekByte = b;
}

// Accumulate bytes until a complete 0xAA...0x55 bootloader packet is received.
bool LG290P::fwGetResponse(uint32_t timeoutMs)
{
    _fw.responseLen = 0;
    uint32_t deadline = millis() + timeoutMs;

    while (millis() < deadline)
    {
        uint8_t b;
        uint32_t remaining = deadline - millis();
        if (remaining == 0)
            break;
        if (fwSerialWaitByte(&b, remaining < 250 ? remaining : 250) <= 0)
            continue;

        if (_fw.responseLen == 0 && b != 0xAA)
            continue;

        _fw.response[_fw.responseLen++] = b;

        if (_fw.responseLen < 5)
            continue;

        uint16_t payloadLen = (((uint16_t)_fw.response[3]) << 8) | _fw.response[4];
        size_t messageBytes = 1 + 1 + 1 + payloadLen + 4;

        if (_fw.responseLen <= messageBytes)
            continue;

        if (_fw.response[_fw.responseLen - 1] == 0x55)
        {
            _fw.cmdResponseLen = _fw.responseLen;
            _fw.responseLen = 0;
            return true;
        }

        if (_fw.responseLen >= 256)
            _fw.responseLen = 0; // safety reset
    }
    return false;
}

// Extract the 2-byte big-endian status field from a response payload; 0 = success
uint16_t LG290P::fwGetCommandStatus(const uint8_t *data)
{
    return (((uint16_t)data[0]) << 8) | data[1];
}

// Write a 32-bit value big-endian into a 4-byte buffer
void LG290P::fwInsertBigEndian(uint32_t val, uint8_t *buf)
{
    buf[0] = (val >> 24) & 0xff;
    buf[1] = (val >> 16) & 0xff;
    buf[2] = (val >> 8) & 0xff;
    buf[3] = val & 0xff;
}

// Send bootloader version query (class 0x02, msg 0x71)
int LG290P::fwSendGetVersion()
{
    uint8_t cmd[10];
    cmd[0] = 0xAA;
    cmd[1] = 0x02;
    cmd[2] = 0x71;
    cmd[3] = 0;
    cmd[4] = 0;
    cmd[9] = 0x55;
    uint32_t crc = computeFirmwareCrc32(0, &cmd[1], 4);
    fwInsertBigEndian(crc, &cmd[5]);
    _hwSerialPort->write(cmd, 10);
    return 0;
}

// Send firmware metadata — total length and CRC32 — so the device can validate after upload (class 0x02, msg 0x02)
int LG290P::fwSendFirmwareInfo()
{
    uint8_t cmd[26];
    cmd[0] = 0xAA;
    cmd[1] = 0x02;
    cmd[2] = 0x02;
    cmd[3] = 0;
    cmd[4] = 0x10;
    cmd[25] = 0x55;
    fwInsertBigEndian((uint32_t)_fw.firmwareSize, &cmd[5]);
    fwInsertBigEndian(_fw.firmwareCrc, &cmd[9]);
    fwInsertBigEndian(0, &cmd[13]);
    fwInsertBigEndian(0, &cmd[17]);
    uint32_t crc = computeFirmwareCrc32(0, &cmd[1], sizeof(cmd) - 1 - 4 - 1);
    fwInsertBigEndian(crc, &cmd[21]);
    _hwSerialPort->write(cmd, sizeof(cmd));
    return 0;
}

// Send flash erase command (class 0x02, msg 0x03); device may take up to 30 s to respond
int LG290P::fwSendErase()
{
    uint8_t cmd[10];
    cmd[0] = 0xAA;
    cmd[1] = 0x02;
    cmd[2] = 0x03;
    cmd[3] = 0;
    cmd[4] = 0;
    cmd[9] = 0x55;
    uint32_t crc = computeFirmwareCrc32(0, &cmd[1], 4);
    fwInsertBigEndian(crc, &cmd[5]);
    _hwSerialPort->write(cmd, 10);
    return 0;
}

// Send a firmware data packet; CRC computed in two chained passes to avoid a large stack buffer.
int LG290P::fwSendPacket(const uint8_t *data, size_t len, int32_t packetNum)
{
    size_t lengthPayload = 4 + len;
    //size_t commandLength = 1 + 1 + 1 + 2 + lengthPayload + 4 + 1;

    // Build 9-byte frame header: header + class + id + length[2] + packetNum[4]
    uint8_t frame[9];
    frame[0] = 0xAA;
    frame[1] = 0x02;
    frame[2] = 0x04;
    frame[3] = (uint8_t)(lengthPayload >> 8);
    frame[4] = (uint8_t)(lengthPayload & 0xff);
    fwInsertBigEndian((uint32_t)packetNum, &frame[5]);

    // CRC covers: class + id + length[2] + packetNum[4] + data[len] (= commandLength - 6)
    uint32_t crc = computeFirmwareCrc32(0, &frame[1], 8);
    crc = computeFirmwareCrc32(crc, data, len);

    uint8_t trailer[5];
    fwInsertBigEndian(crc, trailer);
    trailer[4] = 0x55;

    _hwSerialPort->write(frame, 9);
    _hwSerialPort->write(data, len);
    _hwSerialPort->write(trailer, 5);
    return 0;
}

// Send reset command to boot into the newly flashed firmware (class 0x02, msg 0x31)
int LG290P::fwSendReset()
{
    uint8_t cmd[10];
    cmd[0] = 0xAA;
    cmd[1] = 0x02;
    cmd[2] = 0x31;
    cmd[3] = 0;
    cmd[4] = 0;
    cmd[9] = 0x55;
    uint32_t crc = computeFirmwareCrc32(0, &cmd[1], 4);
    fwInsertBigEndian(crc, &cmd[5]);
    _hwSerialPort->write(cmd, 10);
    return 0;
}

// Reboot module into bootloader mode, negotiate sync words, validate bootloader version,
// send firmware metadata, and erase flash; returns true when device is ready for data packets
bool LG290P::updateFirmwareBegin(size_t firmwareSize, uint32_t firmwareCrc32, bool skipSoftwareReset)
{
    fwCleanup();

    _fw.accumBuf = (uint8_t *)malloc(4096);
    _fw.response = (uint8_t *)malloc(256);
    if (!_fw.accumBuf || !_fw.response)
    {
        fwCleanup();
        return false;
    }

    _fw.firmwareSize = firmwareSize;
    _fw.firmwareCrc = firmwareCrc32;
    _fw.packetCount = (int32_t)((firmwareSize + 4095) / 4096);

    if (!skipSoftwareReset) {
        // Reboot module into bootloader mode
        sendCommandNoResponse("PQTMSRR", 0); // Zero maxWaitMs
        //delay(500); // Extra delay here seems to cause problems?

        // Drain any reboot acknowledgment bytes for 250 ms
        // Tune and test this carefully!
        // 250ms is OK for LG290P03AANR01A06S with a PQTMSRR software reset
        // 250ms is OK for LG290P03AANR02A01S with a PQTMSRR software reset
        uint32_t drainDeadline = millis() + 250;
        while (millis() < drainDeadline)
        {
            if (serialAvailable())
                serialRead();
        }
    }

    // POWER_ON: send SYNC_WORD1 repeatedly until RSP_WORD1 is received
    static const uint8_t syncWord1[4] = {0x09, 0x13, 0x4C, 0x51};
    static const uint8_t rspWord1[4] = {0x4D, 0x3A, 0xFC, 0xAA};
    bool gotRsp1 = false;
    for (int attempt = 0; attempt < 8 && !gotRsp1; attempt++)
    {
        _hwSerialPort->write(syncWord1, 4);
        uint8_t matchIdx = 0;
        uint32_t deadline = millis() + 250;
        while (millis() < deadline && !gotRsp1)
        {
            uint8_t b;
            if (fwSerialWaitByte(&b, 10) <= 0)
                continue;
            if (b == rspWord1[matchIdx])
            {
                if (++matchIdx == 4)
                    gotRsp1 = true;
            }
            else
            {
                matchIdx = (b == rspWord1[0]) ? 1 : 0;
            }
        }
    }
    if (!gotRsp1)
    {
        fwCleanup();
        return false;
    }

    // SYNC: send SYNC_WORD2, wait for RSP_WORD2
    static const uint8_t syncWord2[4] = {0x04, 0xA5, 0x03, 0x12};
    static const uint8_t rspWord2[4] = {0xA0, 0x5B, 0xFD, 0x55};
    _hwSerialPort->write(syncWord2, 4);
    {
        uint8_t matchIdx = 0;
        bool gotRsp2 = false;
        uint32_t deadline = millis() + 500;
        while (millis() < deadline && !gotRsp2)
        {
            uint8_t b;
            if (fwSerialWaitByte(&b, 50) <= 0)
                continue;
            if (b == rspWord2[matchIdx])
            {
                if (++matchIdx == 4)
                    gotRsp2 = true;
            }
            else
            {
                matchIdx = (b == rspWord2[0]) ? 1 : 0;
            }
        }
        if (!gotRsp2)
        {
            fwCleanup();
            return false;
        }
    }

    // BOOT_VERSION: query and validate
    fwSendGetVersion();
    if (!fwGetResponse(500))
    {
        fwCleanup();
        return false;
    }
    {
        uint8_t *r = _fw.response;

        uint32_t crc = computeFirmwareCrc32(0, &r[1], _fw.cmdResponseLen - 1 - 4 - 1);
        uint32_t rxCrc = ((uint32_t)r[12] << 24) | ((uint32_t)r[13] << 16) | ((uint32_t)r[14] << 8) | r[15];
        if (r[0] != 0xAA || 
            r[1] != 2 || 
            r[2] != 0 || 
            r[3] != 0 || 
            r[4] != 7 || 
            r[5] != 2 || 
            r[6] != 0x71 ||
            rxCrc != crc || 
            r[16] != 0x55)
        {
            fwCleanup();
            return false;
        }
        if (fwGetCommandStatus(&r[7]) != 0)
        {
            fwCleanup();
            return false;
        }
    }

    // FIRMWARE_INFO: send metadata, validate ACK
    fwSendFirmwareInfo();
    if (!fwGetResponse(500))
    {
        fwCleanup();
        return false;
    }
    {
        uint8_t *r = _fw.response;
        uint32_t crc = computeFirmwareCrc32(0, &r[1], _fw.cmdResponseLen - 1 - 4 - 1);
        uint32_t rxCrc = ((uint32_t)r[9] << 24) | ((uint32_t)r[10] << 16) | ((uint32_t)r[11] << 8) | r[12];
        if (r[0] != 0xAA || r[1] != 0x02 || r[2] != 0 || r[3] != 0 || r[4] != 4 || r[5] != 2 ||
            (r[6] != 2 && r[6] != 0) || rxCrc != crc || r[13] != 0x55)
        {
            fwCleanup();
            return false;
        }
        if (fwGetCommandStatus(&r[7]) != 0)
        {
            fwCleanup();
            return false;
        }
    }

    // FIRMWARE_ERASE: erase flash, wait up to 30 s
    fwSendErase();
    if (!fwGetResponse(30000))
    {
        fwCleanup();
        return false;
    }
    {
        uint8_t *r = _fw.response;
        uint32_t crc = computeFirmwareCrc32(0, &r[1], _fw.cmdResponseLen - 1 - 4 - 1);
        uint32_t rxCrc = ((uint32_t)r[9] << 24) | ((uint32_t)r[10] << 16) | ((uint32_t)r[11] << 8) | r[12];
        if (r[0] != 0xAA || r[1] != 0x02 || r[2] != 0 || r[3] != 0 || r[4] != 4 || r[5] != 2 ||
            (r[6] != 3 && r[6] != 0) || rxCrc != crc || r[13] != 0x55)
        {
            fwCleanup();
            return false;
        }
        if (fwGetCommandStatus(&r[7]) != 0)
        {
            fwCleanup();
            return false;
        }
    }

    return true;
}

// Accept a chunk of firmware bytes; accumulates into 4096-byte packets and sends each with ACK
bool LG290P::updateFirmware(const uint8_t *data, size_t bytesToWrite)
{
    if (!_fw.accumBuf || !_fw.response)
        return false;

    size_t dataIdx = 0;
    while (dataIdx < bytesToWrite)
    {
        size_t space = 4096 - _fw.accumLen;
        size_t copyLen = bytesToWrite - dataIdx;
        if (copyLen > space)
            copyLen = space;

        memcpy(_fw.accumBuf + _fw.accumLen, data + dataIdx, copyLen);
        _fw.accumLen += copyLen;
        dataIdx += copyLen;

        if (_fw.accumLen == 4096)
        {
            fwSendPacket(_fw.accumBuf, 4096, _fw.packetNumber);
            if (!fwGetResponse(500))
                return false;

            {
                uint8_t *r = _fw.response;
                uint32_t crc = computeFirmwareCrc32(0, &r[1], _fw.cmdResponseLen - 1 - 4 - 1);
                uint32_t rxCrc = ((uint32_t)r[9] << 24) | ((uint32_t)r[10] << 16) | ((uint32_t)r[11] << 8) | r[12];
                if (r[0] != 0xAA || r[1] != 0x02 || r[2] != 0 || r[3] != 0 || r[4] != 4 || r[5] != 2 ||
                    (r[6] != 4 && r[6] != 0) || rxCrc != crc || r[13] != 0x55)
                    return false;
                if (fwGetCommandStatus(&r[7]) != 0)
                    return false;
            }
            _fw.packetNumber++;
            _fw.accumLen = 0;
        }
    }
    return true;
}

// Flush any remaining partial packet, free update buffers; returns true if all bytes were acknowledged
bool LG290P::updateFirmwareEnd()
{
    bool ok = true;
    if (_fw.accumLen > 0)
    {
        fwSendPacket(_fw.accumBuf, _fw.accumLen, _fw.packetNumber);

        // After sending the final packet, the bootloader may take up to 30 s to validate the full firmware and respond

        if (!fwGetResponse(30000))
        {
            ok = false;
        }
        else
        {
            {
                uint8_t *r = _fw.response;
                uint32_t crc = computeFirmwareCrc32(0, &r[1], _fw.cmdResponseLen - 1 - 4 - 1);
                uint32_t rxCrc = ((uint32_t)r[9] << 24) | ((uint32_t)r[10] << 16) | ((uint32_t)r[11] << 8) | r[12];
                if (r[0] != 0xAA || r[1] != 0x02 || r[2] != 0 || r[3] != 0 || r[4] != 4 || r[5] != 2 ||
                    (r[6] != 4 && r[6] != 0) || rxCrc != crc || r[13] != 0x55)
                    ok = false;
                else if (fwGetCommandStatus(&r[7]) != 0)
                    ok = false;
                else
                    _fw.packetNumber++;
            }
        }
    }

    fwCleanup();
    return ok;
}

// Send bootloader reset command then poll for up to maxWaitSeconds for the new firmware to respond
bool LG290P::updateFirmwareIsFinished(uint8_t maxWaitSeconds)
{
    fwSendReset();

    // Drain serial for 1 s while device reboots
    uint32_t drainDeadline = millis() + 1000;
    while (millis() < drainDeadline)
    {
        if (serialAvailable())
            serialRead();
    }

    // Poll for up to maxWaitSeconds for the new firmware to answer commands
    for (unsigned long start = millis(); millis() - start < maxWaitSeconds * 1000UL;)
    {
        if (sendOkCommand("PQTMUNIQID"))
            return true;
    }

    return false;
}
