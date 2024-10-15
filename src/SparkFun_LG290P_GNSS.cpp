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
SEMP_PARSE_ROUTINE const parserTable[] = {
    sempNmeaPreamble,
    sempRtcmPreamble,
};
const int parserCount = sizeof(parserTable) / sizeof(parserTable[0]);

const char *const parserNames[] = {
    "LG290P NMEA Parser",
    "LG290P RTCM Parser",
};
const int parserNameCount = sizeof(parserNames) / sizeof(parserNames[0]);

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
    const char *name;
    if ((name = sempNmeaGetStateName(parse)) != nullptr)
        return name;
    if ((name = sempRtcmGetStateName(parse)) != nullptr)
        return name;
    return sempGetStateName(parse);
}

// Disable debug output from the parser
void LG290P::disableParserDebug()
{
    sempDisableDebugOutput(_sempParse);
}

// Enable debug output from the parser
void LG290P::enableParserDebug(Print *print)
{
    sempEnableDebugOutput(_sempParse, print);
}

// Disable debug output from the parser
void LG290P::disableParserErrors()
{
    sempDisableDebugOutput(_sempParse);
}

// Enable debug output from the parser
void LG290P::enableParserErrors(Print *print)
{
    sempEnableErrorOutput(_sempParse, print);
}

// Print the LG290P parser configuration
void LG290P::printParserConfiguration(Print *print)
{
    sempPrintParserConfiguration(_sempParse, print);
}


bool LG290P::badNmeaChecksum(SEMP_PARSE_STATE *parse)
{
    return false;
}

//----------------------------------------
// LG290P support routines
//----------------------------------------

bool LG290P::begin(HardwareSerial &serialPort, Print *parserDebug, Print *parserError)
{
    ptrLG290P = this;
    _hwSerialPort = &serialPort;

    // Initialize the parser
    _sempParse =
        sempBeginParser(parserTable, parserCount, parserNames, parserNameCount, 0, BUFFER_LENGTH, LG290PProcessMessage,
                        "SFE_LG290P_GNSS_Library", parserError, parserDebug, badNmeaChecksum);
    if (!_sempParse)
    {
        debugPrintf("LG290P Lib: Failed to initialize the parser!");
        return false;
    }

    // We assume the user has started the serial port with proper pins and baud rate prior to calling begin()
    if (!isConnected())
    {
        sempStopParser(&_sempParse);
        return false;
    }
    return true;
}

// Query the device with 'UNIQID', expect OK response
// Device may be booting and outputting other messages (ie, $devicename,COM3*65)
// Try a few times
bool LG290P::isConnected()
{
    // Try up to 10 seconds
    for (unsigned long start = millis(); millis() - start < 10000;)
    {
        if (sendOkCommand("PQTMUNIQID"))
            return true;
        debugPrintf("LG290P failed to connect. Trying again.");
        delay(5);
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

    while (serialAvailable())
        newData = updateOnce();

    lg290PLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware

    return newData;
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

// Checks for new data once
// LG290PProcessMessage() is called once the parser completes on a line
bool LG290P::updateOnce()
{
    const char *endName;
    const char *startName = nullptr;
    SEMP_PARSE_ROUTINE startState;

    if (serialAvailable())
    {
        uint8_t incoming = serialRead();

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
        return true;
    }

    (void)startState; // Fix pesky warning-as-error

    return false;
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

bool LG290P::setModeBase()
{
    return sendOkCommand("PQTMCFGRCVRMODE", ",W,2");
}

bool LG290P::setModeRover()
{
    return sendOkCommand("PQTMCFGRCVRMODE", ",W,1");
}

bool LG290P::setPortBaudrate(int port, uint32_t newBaud)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d,%lu", port, newBaud);
    return sendOkCommand("PQTMCFGUART", parms);
}

bool LG290P::setBaudrate(uint32_t newBaud)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%lu", newBaud);
    return sendOkCommand("PQTMCFGUART", parms);
}

bool LG290P::getPortInfo(int port, uint32_t &newBaud, uint8_t &databits, uint8_t &parity, uint8_t &stop, uint8_t &flowControl)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",R,%d", port);
    bool ret = sendCommand("PQTMCFGUART", parms);
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = ret && packet[1] == "OK";
        newBaud = atol(packet[3].c_str());
        databits = atoi(packet[4].c_str());
        parity = atoi(packet[5].c_str());
        stop = atoi(packet[6].c_str());
        flowControl = atoi(packet[7].c_str());
    }
    return ret;
}

bool LG290P::enablePPS(uint16_t duration, bool alwaysOutput, bool positivePolarity)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,1,%d,%d,%d,0", duration, alwaysOutput ? 1 : 2, positivePolarity);
    return sendOkCommand("PQTMCFGPPS", parms);
}

bool LG290P::disablePPS()
{
    return sendOkCommand("PQTMCFGPPS", ",W,1,0");
}

bool LG290P::getPPSInfo(bool &enabled, uint16_t &duration, bool &alwaysOutput, bool &positivePolarity)
{
    bool ret = sendCommand("PQTMCFGPPS", ",R,1");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = ret && packet[1] == "OK" && packet[2] == "1";
        enabled = packet[3] == "1";
        duration = atoi(packet[4].c_str());
        alwaysOutput = packet[5] == "1";
        positivePolarity = packet[6] == "1";
    }
    return ret;
}

bool LG290P::getConstellationInfo(bool &enableGPS, bool &enableGLONASS, bool &enableGalileo, bool &enableBDS,
      bool &enableQZSS, bool &enableNavIC)
{
    bool ret = sendCommand("PQTMCFGCNST", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
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
        ret = ret && packet[1] == "OK";
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

bool LG290P::getFixInterval(uint16_t &fixInterval)
{
    bool ret = sendCommand("PQTMCFGFIXRATE", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = ret && packet[1] == "OK";
        fixInterval = atoi(packet[2].c_str());
    }
    return ret;
}

bool LG290P::setFixInterval(uint16_t fixInterval)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d", fixInterval);
    return sendOkCommand("PQTMCFGFIXRATE", parms);
}

bool LG290P::setMessageRate(const char *msgName, int rate, int msgVer)
{
    char parms[50];
    snprintf(parms, sizeof parms, msgVer == -1 ? ",W,%s,%d" : ",W,%s,%d,%d", msgName, rate, msgVer);
    return sendOkCommand("PQTMCFGMSGRATE", parms);
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

bool LG290P::softwareReset()
{
    return sendCommand("PQTMSRR");
}

bool LG290P::coldReset()
{
    return sendCommand("PQTMCOLD");
}

bool LG290P::warmReset()
{
    return sendCommand("PQTMWARM");
}

bool LG290P::hotReset()
{
    return sendCommand("PQTMHOT");
}

bool LG290P::configureConstellation(bool enableGPS, bool enableGLONASS, bool enableGalileo, bool enableBDS,
      bool enableQZSS, bool enableNavIC)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d,%d,%d,%d,%d,%d", enableGPS, enableGLONASS, 
        enableGalileo, enableBDS, enableQZSS, enableNavIC);
    bool ret = sendOkCommand("PQTMCFGCNST", parms);

    // Clear the reported satellites because removing a constellation might create orphans
    if (ret) satelliteReporting.clear();
    return ret;
}

bool LG290P::disableEngine()
{
    return sendOkCommand("PQTMGNSSSTOP");
}

bool LG290P::enableEngine()
{
    return sendOkCommand("PQTMGNSSSTART");
}

bool LG290P::saveParameters()
{
    return sendOkCommand("PQTMSAVEPAR");
}

bool LG290P::restoreParameters()
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
    for (int i=1; i<cmd.length(); ++i)
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
bool LG290P::sendCommand(const char *command, const char *parms, uint16_t maxWaitMs)
{
    if (lg290PLibrarySemaphoreBlock)
        return false;

    bool success = false;
    // clearBuffer(); // Not necessary?

    debugPrintf("sendCommand(\"%s\", \"%s\")", command, parms);
    commandName = command[0] == '$' ? command + 1 : command;
    commandResponse = LG290P_RESULT_RESPONSE_COMMAND_WAITING; // Reset

    lg290PLibrarySemaphoreBlock = true; // Prevent external tasks from harvesting serial data

    // add $ and checksum and transmit
    if (!transmit(command, parms))
        return false;

    debugPrintf("...sendCommand: waiting for response for %s", command);

    // Feed the parser until we see a response to the command
    for (unsigned long start = millis(); millis() - start < maxWaitMs;)
    {
        updateOnce(); // Will call LG290PProcessMessage()

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
        debugPrintf("...sendCommand: TIMEOUT: no response received");
    commandName.clear();
    lg290PLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
    return success;
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

    // Get all the satellites visible?
    if (talker == nullptr)
    {
        for (auto &item : satelliteReporting)
            ret.insert(ret.end(), item.second.begin(), item.second.end());
    }
    else 
    {
        auto item = satelliteReporting.find(talker);
        if (item != satelliteReporting.end())
            ret = item->second;
    }
    return ret;
}

bool LG290P::getSurveyMode(int &mode, int &positionTimes, double &accuracyLimit, double &ecefX, double &ecefY, double &ecefZ)
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

bool LG290P::setSurveyInMode(int positionTimes, double accuracyLimit /* = 0 */)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,%d,%f,0.00,0.00,0.00", positionTimes, accuracyLimit);
    return sendOkCommand("PQTMCFGSVIN", parms);
    return true;
}

bool LG290P::setSurveyFixedMode(double ecefX, double ecefY, double ecefZ)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,2,0,0,%f,%f,%f", ecefX, ecefY, ecefZ);
    return sendOkCommand("PQTMCFGSVIN", parms);
}

// Main handler and RAM inits
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define CHECK_POINTER_BOOL(packetPointer, initPointer)                                                                 \
    {                                                                                                                  \
        if (packetPointer == nullptr)                                                                                  \
            initPointer();                                                                                             \
        if (packetPointer == nullptr)                                                                                  \
            return false;                                                                                              \
    }

#define CHECK_POINTER_VOID(packetPointer, initPointer)                                                                 \
    {                                                                                                                  \
        if (packetPointer == nullptr)                                                                                  \
            initPointer();                                                                                             \
        if (packetPointer == nullptr)                                                                                  \
            return;                                                                                                    \
    }

// Cracks an NMEA sentence into the applicable container
// The sentence might be a normal NMEA one like $GPGGA or a custom one like $PQTMVER. In
// the former case, the first term is cracked into the "Talker Id" (GP) and "Sentence Id" (GGA)
void LG290P::nmeaHandler(SEMP_PARSE_STATE *parse)
{
    // Is this a command response?
    std::string sentence = (const char *)parse->buffer;
    if (sentence.substr(sentence.length() - 2) == "\r\n")
    // if (sentence.ends_with("\r\n"))
        sentence.erase(sentence.size() - 2);

    NmeaPacket nmea = NmeaPacket::FromString(sentence);

    // Do $PQTM strings later
    if (sentence.substr(0, 2) != "$P")
    {
        std::string strName = nmea[0];
        if (nmea.IsValid())
        {
            if (nmea.ChecksumValid())
            {
                std::string id = nmea.SentenceId();

                if (id == "RMC")
                {
                    CHECK_POINTER_VOID(ptrLG290P->snapshot, ptrLG290P->initSnapshot); // Check that RAM has been allocated
                    lastUpdateGeodetic = millis(); // Update stale marker
                    nmea.processRMC(ptrLG290P->snapshot);
                }
                
                if (id == "GGA")
                {
                    CHECK_POINTER_VOID(ptrLG290P->snapshot, ptrLG290P->initSnapshot); // Check that RAM has been allocated
                    lastUpdateGeodetic = millis(); // Update stale marker
                    nmea.processGGA(ptrLG290P->snapshot);
                }

                if (id == "GSV")
                {
                    uint16_t msgCount = (uint16_t)strtoul(nmea[1].c_str(), NULL, 10);
                    uint16_t msgNo = (uint16_t)strtoul(nmea[2].c_str(), NULL, 10);
                    uint16_t svsInView = (uint16_t)strtoul(nmea[3].c_str(), NULL, 10);
                    std::string talker = nmea.TalkerId();

                    if (msgNo == 1)
                    {
                        satelliteStaging[talker].clear();
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
                        satelliteStaging[talker].push_back(sat);
                    }
                    
                    if (msgNo == msgCount)
                    {
                        satelliteReporting[talker] = satelliteStaging[talker];
                        hasNewSatellites = true;
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
        if (nmea[0].substr(1) == ptrLG290P->commandName)
        {
            // Display the command response
            ptrLG290P->debugPrintf("...sendCommand: (main loop) Expected command response found %s", parse->buffer);
            ptrLG290P->pqtmResponse = nmea;
            ptrLG290P->commandResponse = LG290P_RESULT_RESPONSE_COMMAND_OK;
        }

        else if (!ptrLG290P->commandName.empty())
        {
            // Display the command response
            ptrLG290P->debugPrintf("LG290P Lib: Unknown command response: %s", parse->buffer);
            ptrLG290P->debugPrintf("LG290P Lib: Looking for command: %s", ptrLG290P->commandName.c_str());
        }
    }

    std::string id = nmea.SentenceId();
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
        nmeaAllSubscribe(nmea); //call it!
}

// Cracks an NMEA into the applicable container
void LG290P::rtcmHandler(SEMP_PARSE_STATE *parse)
{
    bool good = parse->length > 6 && parse->buffer[0] == 0xD3;
    RtcmPacket packet;
    if (good)
    {
        packet.payloadLen = (parse->buffer[1] << 8) | parse->buffer[2];
        good = packet.payloadLen + 6 == parse->length;
    }

    if (good)
    {
        packet.type = (parse->buffer[3] << 4) | (parse->buffer[4] >> 4);
        packet.buffer = parse->buffer;
        packet.bufferlen = parse->length;
        rtcmCounters[packet.type]++;

        // handle specific and general callbacks
        // If user is subscribed for this packet, call the callback
        if (rtcmSubscriptions.count(packet.type) > 0)
            rtcmSubscriptions[packet.type](packet);
        if (rtcmAllSubscribe != nullptr)
            rtcmAllSubscribe(packet);
    }
}

// Allocate RAM for NMEA snapshot and initialize it
bool LG290P::initSnapshot()
{
    snapshot = new NmeaSnapshot;
    if (snapshot == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return false;
    }

    return true;
}

// All the general gets and sets
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

bool LG290P::isNewSnapshotAvailable() 
{
    if (snapshot == nullptr)
        return false;
    bool r = snapshot->newDataAvailable;
    snapshot->newDataAvailable = false;
    return r;
}

bool LG290P::isNewSatelliteInfoAvailable()
{
    bool r = hasNewSatellites;
    hasNewSatellites = false;
    return r;
}

double LG290P::getLatitude()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->latitude;
}

double LG290P::getLongitude()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->longitude;
}

double LG290P::getAltitude()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->altitude;
}

double LG290P::getHorizontalSpeed()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->horizontalSpeed;
}

double LG290P::getVerticalSpeed()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->verticalSpeed;
}

uint16_t LG290P::getSatellitesTracked()
{
    uint16_t count = 0;
    for (auto &item : satelliteReporting)
        count += item.second.size();
    return count;
}

uint16_t LG290P::getSatellitesUsed()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->satellitesUsed;
}

// Return the number of millis since last update
uint32_t LG290P::getFixAgeMilliseconds()
{
    return millis() - lastUpdateGeodetic;
}

uint16_t LG290P::getYear()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->year;
}

uint8_t LG290P::getMonth()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->month;
}

uint8_t LG290P::getDay()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->day;
}

uint8_t LG290P::getHour()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->hour;
}

uint8_t LG290P::getMinute()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->minute;
}

uint8_t LG290P::getSecond()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->second;
}

uint16_t LG290P::getMillisecond()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return (uint16_t)(snapshot->nanosecond / 1000000);
}

NmeaPacket NmeaPacket::FromString(const std::string &str)
{
    NmeaPacket s;
    s.charCount = str.length();
    size_t start = 0;
    size_t delimiterPos = str.find_first_of(",*");
    unsigned long calculatedChksum = 0;
    log_d("Parsing sentence %s", str.c_str());

    while (true)
    {
        std::string token = delimiterPos != std::string::npos ? str.substr(start, delimiterPos - start) : str.substr(start);
        s.fields.push_back(token);
        for (auto c : token.substr(s.fields.size() == 1 && s.fields[0].substr(0, 1) == "$" ? 1 : 0))
            calculatedChksum ^= c;

        if (delimiterPos == std::string::npos)
        {
            s.hasChecksum = s.checksumValid = false;
            log_d("Doesn't have any * checksum");
            break;
        }

        if (str[delimiterPos] == '*')
        {
            token = str.substr(delimiterPos + 1);
            s.fields.push_back(token);
            if (token.find_first_not_of("0123456789ABCDEFabcdef") != std::string::npos)
            {
                s.hasChecksum = s.checksumValid = false;
                log_e("Bad format checksum: '%s'", token.c_str());
                log_e("Sentence was '%s'", str.c_str());
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
    if (!IsValid()) return "";

    // If proper NMEA, separate Talker and Sentence Ids
    return fields[0].substr(0, 2) == "$G" ? fields[0].substr(1, 2) : "";
}

std::string NmeaPacket::SentenceId() const
{
    if (!IsValid()) return "";

    // If proper NMEA, separate Talker and Sentence Ids
    return fields[0].substr(0, 2) == "$G" ? fields[0].substr(3) : fields[0].substr(1);
}

void NmeaPacket::processGGA(NmeaSnapshot *snapshot)
{
    if (fields.size() >= 10)
    {
        NmeaPacket::parseTime(fields[1], snapshot->hour, snapshot->minute, snapshot->second, snapshot->nanosecond);
        NmeaPacket::parseLocation(fields[2], fields[3], fields[4], fields[5], snapshot->latitude, snapshot->longitude);
        NmeaPacket::parseQuality(fields[6], snapshot->quality);
        NmeaPacket::parseSatelliteCount(fields[7], snapshot->satellitesUsed);
        NmeaPacket::parseHdop(fields[8].c_str(), snapshot->hdop);
        NmeaPacket::parseAltitude(fields[9].c_str(), snapshot->altitude);
        snapshot->newDataAvailable = true;
    }
}

void NmeaPacket::processRMC(NmeaSnapshot *snapshot)
{
    if (fields.size() >= 10)
    {
        NmeaPacket::parseTime(fields[1], snapshot->hour, snapshot->minute, snapshot->second, snapshot->nanosecond);
        NmeaPacket::parseFixStatus(fields[2], snapshot->fixStatus);
        NmeaPacket::parseLocation(fields[3], fields[4], fields[5], fields[6], snapshot->latitude, snapshot->longitude);
        NmeaPacket::parseSpeed(fields[7].c_str(), snapshot->horizontalSpeed);
        NmeaPacket::parseCourse(fields[8].c_str(), snapshot->course);
        NmeaPacket::parseDate(fields[9], snapshot->year, snapshot->month, snapshot->day);
        snapshot->newDataAvailable = true;
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

void NmeaPacket::parseLocation(const std::string &termLat, const std::string &termNS, const std::string &termLong, const std::string &termEW, double &latitude, double &longitude)
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

#if false
void LG290P::serialPrintln(const char *command)
{
    if (_hwSerialPort != nullptr)
    {
        _hwSerialPort->println(command);
    }
}

// Cracks a given binary message into the applicable container
void LG290P::unicoreHandler(uint8_t *response, uint16_t length)
{
    uint16_t messageID = ((uint16_t)response[offsetHeaderMessageId + 1] << 8) | response[offsetHeaderMessageId];

    if (messageID == messageIdBestnav)
    {
        debugPrintf("BestNav Handler");
        CHECK_POINTER_VOID(packetBESTNAV, initBestnav); // Check that RAM has been allocated

        lastUpdateGeodetic = millis(); // Update stale marker

        uint8_t *data = &response[LG290PHeaderLength]; // Point at the start of the data fields

        // Move data into given containers

        // 0 = Solution computed, 1 = Insufficient observation, 3 = No convergence, 4 = Covariance trace
        memcpy(&packetBESTNAV->data.solutionStatus, &data[offsetBestnavPsolStatus], sizeof(uint8_t));

        // 0 = None, 1 = FixedPos, 8 = DopplerVelocity, 16 = Single, ...
        memcpy(&packetBESTNAV->data.positionType, &data[offsetBestnavPosType], sizeof(uint8_t));
        memcpy(&packetBESTNAV->data.velocityType, &data[offsetBestnavVelType], sizeof(uint8_t));

        memcpy(&packetBESTNAV->data.latitude, &data[offsetBestnavLat], sizeof(double));
        memcpy(&packetBESTNAV->data.longitude, &data[offsetBestnavLon], sizeof(double));
        memcpy(&packetBESTNAV->data.altitude, &data[offsetBestnavHgt], sizeof(double));
        memcpy(&packetBESTNAV->data.horizontalSpeed, &data[offsetBestnavHorSpd], sizeof(double));
        memcpy(&packetBESTNAV->data.verticalSpeed, &data[offsetBestnavVertSpd], sizeof(double));
        memcpy(&packetBESTNAV->data.trackGround, &data[offsetBestnavTrkGnd], sizeof(double));

        memcpy(&packetBESTNAV->data.latitudeDeviation, &data[offsetBestnavLatDeviation], sizeof(float));
        memcpy(&packetBESTNAV->data.longitudeDeviation, &data[offsetBestnavLonDeviation], sizeof(float));
        memcpy(&packetBESTNAV->data.heightDeviation, &data[offsetBestnavHgtDeviation], sizeof(float));

        memcpy(&packetBESTNAV->data.horizontalSpeedDeviation, &data[offsetBestnavHorspdStd], sizeof(float));
        memcpy(&packetBESTNAV->data.verticalSpeedDeviation, &data[offsetBestnavVerspdStd], sizeof(float));

        memcpy(&packetBESTNAV->data.satellitesTracked, &data[offsetBestnavSatsTracked], sizeof(uint8_t));
        memcpy(&packetBESTNAV->data.satellitesUsed, &data[offsetBestnavSatsUsed], sizeof(uint8_t));

        uint8_t extSolStat;
        memcpy(&extSolStat, &data[offsetBestnavExtSolStat], sizeof(uint8_t));
        packetBESTNAV->data.rtkSolution = extSolStat & 0x01;                   // 0 = unchecked, 1 = checked
        packetBESTNAV->data.pseudorangeCorrection = (extSolStat >> 1) & 0b111; // Limit to three bits
    }
    else if (messageID == messageIdRectime)
    {
        debugPrintf("RecTime Handler");
        CHECK_POINTER_VOID(packetRECTIME, initRectime); // Check that RAM has been allocated

        lastUpdateDateTime = millis();

        uint8_t *data = &response[LG290PHeaderLength]; // Point at the start of the data fields

        // Move data into given containers
        memcpy(&packetRECTIME->data.timeStatus, &data[offsetRectimeClockStatus], sizeof(uint8_t));
        memcpy(&packetRECTIME->data.timeOffset, &data[offsetRectimeOffset], sizeof(double));
        memcpy(&packetRECTIME->data.timeDeviation, &data[offsetRectimeOffsetStd], sizeof(double));
        memcpy(&packetRECTIME->data.year, &data[offsetRectimeUtcYear], sizeof(uint16_t));
        memcpy(&packetRECTIME->data.month, &data[offsetRectimeUtcMonth], sizeof(uint8_t));
        memcpy(&packetRECTIME->data.day, &data[offsetRectimeUtcDay], sizeof(uint8_t));
        memcpy(&packetRECTIME->data.hour, &data[offsetRectimeUtcHour], sizeof(uint8_t));
        memcpy(&packetRECTIME->data.minute, &data[offsetRectimeUtcMinute], sizeof(uint8_t));

        memcpy(&packetRECTIME->data.millisecond, &data[offsetRectimeUtcMillisecond], sizeof(uint32_t));
        packetRECTIME->data.second = round(packetRECTIME->data.millisecond / 1000.0);
        packetRECTIME->data.millisecond -= (packetRECTIME->data.second * 1000); // Remove seconds from milliseconds

        memcpy(&packetRECTIME->data.dateStatus, &data[offsetRectimeUtcStatus], sizeof(uint8_t));
    }
    else if (messageID == messageIdBestnavXyz)
    {
        debugPrintf("BestNavXyz Handler");
        CHECK_POINTER_VOID(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated

        lastUpdateEcef = millis(); // Update stale marker

        uint8_t *data = &response[LG290PHeaderLength]; // Point at the start of the data fields

        // Move data into given containers
        memcpy(&packetBESTNAVXYZ->data.ecefX, &data[offsetBestnavXyzPX], sizeof(double));
        memcpy(&packetBESTNAVXYZ->data.ecefY, &data[offsetBestnavXyzPY], sizeof(double));
        memcpy(&packetBESTNAVXYZ->data.ecefZ, &data[offsetBestnavXyzPZ], sizeof(double));

        memcpy(&packetBESTNAVXYZ->data.ecefXDeviation, &data[offsetBestnavXyzPXDeviation], sizeof(float));
        memcpy(&packetBESTNAVXYZ->data.ecefYDeviation, &data[offsetBestnavXyzPYDeviation], sizeof(float));
        memcpy(&packetBESTNAVXYZ->data.ecefZDeviation, &data[offsetBestnavXyzPZDeviation], sizeof(float));
    }
    else if (messageID == messageIdVersion)
    {
        debugPrintf("Version Handler");
        CHECK_POINTER_VOID(packetVERSION, initVersion); // Check that RAM has been allocated

        lastUpdateVersion = millis(); // Update stale marker

        uint8_t *data = &response[LG290PHeaderLength]; // Point at the start of the data fields

        // Move data into given containers
        memcpy(&packetVERSION->data.modelType, &data[offsetVersionModuleType], sizeof(packetVERSION->data.modelType));
        memcpy(&packetVERSION->data.swVersion, &data[offsetVersionFirmwareVersion],
               sizeof(packetVERSION->data.swVersion));
        memcpy(&packetVERSION->data.efuseID, &data[offsetVersionEfuseID], sizeof(packetVERSION->data.efuseID));
        memcpy(&packetVERSION->data.compileTime, &data[offsetVersionCompTime], sizeof(packetVERSION->data.compileTime));
    }
    else
    {
        // Is this a NMEA sentence?
        if (response[0] == '$')
        {
            response[length] = '\0'; // Force terminator because strncasestr does not exist

            // The LG290P does not respond to binary requests when there is no GNSS reception.
            // Block BestNavB, etc commands if there is no fix.
            // Look for GNGGA NMEA then extract GNSS position status (spot 6).
            // $GNGGA,181535.00,,,,,0,00,9999.0,,,,,,*43
            char *responsePointer = strcasestr((char *)response, "GNGGA");
            if (responsePointer != nullptr) // Found
            {
                char gngga[100];
                strncpy(gngga, (const char *)response, length - 1); // Make copy before strtok

                debugPrintf("LG290P Lib: GNGGA message: %s\r\n", gngga);

                char *pt;
                pt = strtok(gngga, ",");
                int counter = 0;
                while (pt != NULL)
                {
                    int spotValue = atoi(pt);
                    if (counter++ == 6)
                        nmeaPositionStatus = spotValue;
                    pt = strtok(NULL, ",");
                }
            }
            else
            {
                // Unhandled NMEA message
                // debugPrintf("LG290P Lib: Unhandled NMEA sentence (%d bytes): %s\r\n", length, (char *)response);
            }
        }
        else
        {
            debugPrintf("LG290P Lib: Unknown message id: %d\r\n", messageID);
        }
    }
}
// Allocate RAM for packetVERSION and initialize it
bool LG290P::initVersion()
{
    packetVERSION = new UNICORE_VERSION_t; // Allocate RAM for the main struct
    if (packetVERSION == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return (false);
    }
    //   packetVERSION->callbackPointerPtr = nullptr;
    //   packetVERSION->callbackData = nullptr;

    // Send command for single query
    if (sendOkCommand("VERSIONB") == false)
    {
        delete packetVERSION;
        packetVERSION = nullptr; // Remove pointer so we will re-init next check
        return (false);
    }

    debugPrintf("VERSION started");

    // Wait until response is received
    lastUpdateVersion = 0;
    uint16_t maxWait = 1000; // Wait for one response to come in
    unsigned long startTime = millis();
    while (1)
    {
        update(); // Call parser
        if (lastUpdateVersion > 0)
            break;
        if (millis() - startTime > maxWait)
        {
            debugPrintf("GNSS: Failed to get response from VERSION start");
            delete packetVERSION;
            packetVERSION = nullptr;
            return (false);
        }
    }

    return (true);
}

double LG290P::getTrackGround()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->trackGround;
}

double LG290P::getCourse()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->course;
}

float LG290P::getLatitudeDeviation()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->latitudeDeviation;
}
float LG290P::getLongitudeDeviation()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->longitudeDeviation;
}
float LG290P::getAltitudeDeviation()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->heightDeviation;
}
float LG290P::getHorizontalSpeedDeviation()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->horizontalSpeedDeviation;
}
float LG290P::getVerticalSpeedDeviation()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->verticalSpeedDeviation;
}

// 0 = Solution computed, 1 = Insufficient observation, 3 = No convergence, 4 = Covariance trace
uint8_t LG290P::getSolutionStatus()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->solutionStatus;
}

// 0 = no fix, 1 = dead reckoning only, 2 = 2D-fix, 3 = 3D-fix, 4 = GNSS + dead reckoning combined, 5 = time only
// fix
uint8_t LG290P::getPositionType()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->positionType;
}
uint8_t LG290P::getVelocityType()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->velocityType;
}

uint8_t LG290P::getRTKSolution()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->rtkSolution;
}

uint8_t LG290P::getPseudorangeCorrection()
{
    CHECK_POINTER_BOOL(snapshot, initSnapshot); // Check that RAM has been allocated
    return snapshot->pseudorangeCorrection;
}

double LG290P::getEcefX()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefX);
}
double LG290P::getEcefY()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefY);
}
double LG290P::getEcefZ()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefZ);
}
float LG290P::getEcefXDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefXDeviation);
}
float LG290P::getEcefYDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefYDeviation);
}
float LG290P::getEcefZDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefZDeviation);
}

// Allocate RAM for packetBESTNAV and initialize it
bool LG290P::initBestnav(uint8_t rate)
{
    if ((startBinaryBeforeFix == false) && (isNmeaFixed() == false))
    {
        debugPrintf("LG290P Lib: BestNav no fix");
        return (false);
    }

    packetBESTNAV = new UNICORE_BESTNAV_t; // Allocate RAM for the main struct
    if (packetBESTNAV == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return (false);
    }
    //   packetBESTNAV->callbackPointerPtr = nullptr;
    //   packetBESTNAV->callbackData = nullptr;

    // Start outputting BESTNAV in Binary on this COM port
    // Wait until first report is available
    char command[50];
    snprintf(command, sizeof(command), "BESTNAVB %d", rate);
    if (sendOkCommand(command) == false)
    {
        delete packetBESTNAV;
        packetBESTNAV = nullptr; // Remove pointer so we will re-init next check
        return (false);
    }

    debugPrintf("BestNav started");

    lastUpdateGeodetic = 0;
    uint16_t maxWait = (1000 / rate) + 100; // Wait for one response to come in
    unsigned long startTime = millis();
    while (1)
    {
        update(); // Call parser
        if (lastUpdateGeodetic > 0)
            break;
        if (millis() - startTime > maxWait)
        {
            debugPrintf("GNSS: Failed to get response from BestNav start");
            delete packetBESTNAV;
            packetBESTNAV = nullptr;
            return (false);
        }
    }
    return (true);
}

// Allocate RAM for packetBESTNAVXYZ and initialize it
bool LG290P::initBestnavXyz(uint8_t rate)
{
    if ((startBinaryBeforeFix == false) && (isNmeaFixed() == false))
    {
        debugPrintf("LG290P Lib: BestNavXyz no fix");
        return (false);
    }

    packetBESTNAVXYZ = new UNICORE_BESTNAVXYZ_t; // Allocate RAM for the main struct
    if (packetBESTNAVXYZ == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return (false);
    }
    //   packetBESTNAVXYZ->callbackPointerPtr = nullptr;
    //   packetBESTNAVXYZ->callbackData = nullptr;

    // Start outputting BESTNAVXYZ in Binary on this COM port
    char command[50];
    snprintf(command, sizeof(command), "BESTNAVXYZB %d", rate);
    if (sendOkCommand(command) == false)
    {
        delete packetBESTNAVXYZ;
        packetBESTNAVXYZ = nullptr; // Remove pointer so we will re-init next check
        return (false);
    }

    debugPrintf("BestNavXYZB started");

    // Wait until first report is available
    lastUpdateEcef = 0;
    uint16_t maxWait = (1000 / rate) + 100; // Wait for one response to come in
    unsigned long startTime = millis();
    while (1)
    {
        update(); // Call parser
        if (lastUpdateEcef > 0)
            break;
        if (millis() - startTime > maxWait)
        {
            debugPrintf("GNSS: Failed to get response from BestNavXyz start");
            delete packetBESTNAVXYZ;
            packetBESTNAVXYZ = nullptr;
            return (false);
        }
    }

    return (true);
}

// Allocate RAM for packetRECTIME and initialize it
bool LG290P::initRectime(uint8_t rate)
{
    if ((startBinaryBeforeFix == false) && (isNmeaFixed() == false))
    {
        debugPrintf("LG290P Lib: RecTime no fix");
        return (false);
    }

    packetRECTIME = new UNICORE_RECTIME_t; // Allocate RAM for the main struct
    if (packetRECTIME == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return (false);
    }
    //   packetRECTIME->callbackPointerPtr = nullptr;
    //   packetRECTIME->callbackData = nullptr;

    debugPrintf("RecTime started");

    // Start outputting RECTIME in Binary on this COM port
    char command[50];
    snprintf(command, sizeof(command), "RECTIMEB %d", rate);
    if (sendOkCommand(command) == false)
    {
        delete packetRECTIME;
        packetRECTIME = nullptr; // Remove pointer so we will re-init next check
        return (false);
    }

    debugPrintf("RecTimeB started");

    // Wait until first report is available
    lastUpdateDateTime = 0;
    uint16_t maxWait = (1000 / rate) + 100; // Wait for one response to come in
    unsigned long startTime = millis();
    while (1)
    {
        update(); // Call parser
        if (lastUpdateDateTime > 0)
            break;
        if (millis() - startTime > maxWait)
        {
            debugPrintf("GNSS: Failed to get response from RecTime start");
            delete packetRECTIME;
            packetRECTIME = nullptr;
            return (false);
        }
    }

    return (true);
}

uint8_t LG290P::getTimeStatus()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.timeStatus);
}
uint8_t LG290P::getDateStatus()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.dateStatus);
}

// Receiver clock offset relative to GPS time, s.
// Positive indicates that the receiver clock is ahead of GPS time. To calculate the GPS time, use the formula
// below: GPS time = receiver time - clock offset
double LG290P::getTimeOffset()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.timeOffset);
}

// Standard deviation of the receiver clock offset, s.
double LG290P::getTimeOffsetDeviation()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.timeDeviation);
}

uint8_t LG290P::getModelType()
{
    CHECK_POINTER_BOOL(packetVERSION, initVersion); // Check that RAM has been allocated
    return (packetVERSION->data.modelType);
}
char *LG290P::getVersion()
{
    CHECK_POINTER_CHAR(packetVERSION, initVersion); // Check that RAM has been allocated
    return (packetVERSION->data.swVersion);
}
char *LG290P::getID()
{
    CHECK_POINTER_CHAR(packetVERSION, initVersion); // Check that RAM has been allocated
    return (packetVERSION->data.efuseID);
}
char *LG290P::getCompileTime()
{
    CHECK_POINTER_CHAR(packetVERSION, initVersion); // Check that RAM has been allocated
    return (packetVERSION->data.compileTime);
}

// Returns pointer to terminated response.
//$command,VERSION,response: OK*04
// #VERSION,92,GPS,FINE,2289,167126600,0,0,18,155;LG290P,R4.10Build7923,HRPT00-S10C-P,2310415000001-MD22B1224961040,ff3bd496fd7ca68b,2022/09/28*45d62771
char *LG290P::getVersionFull(uint16_t maxWaitMs)
{
    // Issue the version command
    LG290PResult result = sendQuery("VERSION");

    // Process the response
    if (result == LG290P_RESULT_OK)
        // Response sitting in buffer. Return pointer to buffer.
        return ((char *)_sempParse->buffer);
    else if (result == LG290P_RESULT_TIMEOUT_RESPONSE)
        return ((char *)"Timeout");
    else if (result == LG290P_RESULT_RESPONSE_COMMAND_ERROR)
        return ((char *)"Error1");
    return ((char *)"Error2");
}

// Cracks a given CONFIG response into settings
void LG290P::configHandler(uint8_t *response, uint16_t length)
{
    // We've received a response such as $CONFIG,COM3,CONFIG COM3 115200*23
    // See if it is the one we want
    char *responsePointer = strcasestr((char *)response, configStringToFind);
    if (responsePointer != nullptr) // Found
    {
        configStringFound = true;
    }
    else
    {
        // This config response was not what we were looking for
    }
}

// Given a string such as "CONFIG COM3 115200", query the device's config settings
// If the given string is in the CONFIG response, return true
// Send a CONFIG command and see if a specific string exists in the responses
// $command,config,response: OK*54
// $CONFIG,ANTENNA,CONFIG ANTENNA POWERON*7A
// $CONFIG,NMEAVERSION,CONFIG NMEAVERSION V410*47
// $CONFIG,RTK,CONFIG RTK TIMEOUT 120*6C
// $CONFIG,RTK,CONFIG RTK RELIABILITY 3 1*76
// $CONFIG,PPP,CONFIG PPP TIMEOUT 120*6C
// $CONFIG,DGPS,CONFIG DGPS TIMEOUT 300*6C
// $CONFIG,RTCMB1CB2A,CONFIG RTCMB1CB2A ENABLE*25
// $CONFIG,ANTENNADELTAHEN,CONFIG ANTENNADELTAHEN 0.0000 0.0000 0.0000*3A
// $CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 500000 1000 0 0*6E
// $CONFIG,SIGNALGROUP,CONFIG SIGNALGROUP 2*16
// $CONFIG,ANTIJAM,CONFIG ANTIJAM AUTO*2B
// $CONFIG,AGNSS,CONFIG AGNSS DISABLE*70
// $CONFIG,BASEOBSFILTER,CONFIG BASEOBSFILTER DISABLE*70
// $CONFIG,COM1,CONFIG COM1 115200*23
// $CONFIG,COM2,CONFIG COM2 115200*23
// $CONFIG,COM3,CONFIG COM3 115200*23
bool LG290P::isConfigurationPresent(const char *stringToFind, uint16_t maxWaitMs)
{
    LG290PResult result;

    clearBuffer();

    // Send command and check for OK response
    result = sendString("CONFIG", maxWaitMs);
    if (result != LG290P_RESULT_OK)
        // return (result);
        return (false);

    // Setup configStringToFind so configHandler() knows what to look for
    strncpy(configStringToFind, stringToFind, sizeof(configStringToFind));

    configStringFound = false; // configHandler() sets true if we find the intended string

    commandResponse = LG290P_RESULT_RESPONSE_COMMAND_WAITING; // Reset

    unicoreLibrarySemaphoreBlock = true; // Prevent external tasks from harvesting serial data

    // Feed the parser until we see a response to the command
    int wait = 0;
    while (1)
    {
        if (wait++ == maxWaitMs)
        {
            debugPrintf("LG290P Lib: isConfigPresent Response timeout");
            unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
            // return (LG290P_RESULT_TIMEOUT_RESPONSE);
            return (false);
        }

        updateOnce(); // Will call LG290PProcessMessage() and configHandler()

        if (configStringFound == true)
        {
            // return (LG290P_RESULT_CONFIG_PRESENT);
            return (true);
        }

        if (commandResponse == LG290P_RESULT_RESPONSE_COMMAND_ERROR)
        {
            debugPrintf("LG290P Lib: Query failure");
            unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
            // return (LG290P_RESULT_RESPONSE_COMMAND_ERROR);
            return (false);
        }

        delay(1);
    }

    unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware

    // return (LG290P_RESULT_OK);
    return (false);
}

// Returns true when GNGGA NMEA reports position status >= 1
bool LG290P::isNmeaFixed()
{
    if (nmeaPositionStatus >= 1)
        return (true);
    return (false);
}
// By default, library will attempt to start RECTIME and BESTNAV regardless of GNSS fix
// This may lead to command timeouts as the LG290P does not appear to respond to BESTNAVB commands if 3D fix is not
// achieved. Set startBinartBeforeFix = false via disableBinaryBeforeFix() to block binary commands before a fix is
// achieved
void LG290P::enableBinaryBeforeFix()
{
    startBinaryBeforeFix = true;
}
void LG290P::disableBinaryBeforeFix()
{
    startBinaryBeforeFix = false;
}
#endif
