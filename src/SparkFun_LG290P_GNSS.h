/*
  This is a library to control Quectel GNSS receivers, with
  a focus on the LG290P QuadBand receiver.

  Development environment specifics:
  Arduino IDE 2.3.x

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef _SPARKFUN_LG290P_GNSS_ARDUINO_LIBRARY_H
#define _SPARKFUN_LG290P_GNSS_ARDUINO_LIBRARY_H

#include "Arduino.h"

#if __has_include("SoftwareSerial.h")
#include <SoftwareSerial.h>
#endif

#include "LG290P_structs.h"
#include <SparkFun_Extensible_Message_Parser.h>
#include <map>

typedef enum
{
    LG290P_RESULT_OK = 0,
    LG290P_RESULT_TIMEOUT_START_BYTE,
    LG290P_RESULT_TIMEOUT_DATA_BYTE,
    LG290P_RESULT_TIMEOUT_END_BYTE,
    LG290P_RESULT_TIMEOUT_RESPONSE,
    LG290P_RESULT_WRONG_COMMAND,
    LG290P_RESULT_WRONG_MESSAGE_ID,
    LG290P_RESULT_BAD_START_BYTE,
    LG290P_RESULT_BAD_CHECKSUM,
    LG290P_RESULT_BAD_CRC,
    LG290P_RESULT_MISSING_CRC,
    LG290P_RESULT_TIMEOUT,
    LG290P_RESULT_RESPONSE_OVERFLOW,
    LG290P_RESULT_RESPONSE_COMMAND_OK,
    LG290P_RESULT_RESPONSE_COMMAND_ERROR,
    LG290P_RESULT_RESPONSE_COMMAND_WAITING,
    LG290P_RESULT_RESPONSE_COMMAND_CONFIG,
    LG290P_RESULT_CONFIG_PRESENT,
} LG290PResult;

#if false
#define LG290PBinarySyncA ((uint8_t)0xAA)
#define LG290PBinarySyncB ((uint8_t)0x44)
#define LG290PBinarySyncC ((uint8_t)0xB5)
#define LG290PASCIISyncEnd ((uint8_t)'\n')

#define LG290PHeaderLength ((uint16_t)24)
#define offsetHeaderSyncA ((uint16_t)0)
#define offsetHeaderSyncB ((uint16_t)1)
#define offsetHeaderSyncC ((uint16_t)2)
#define offsetHeaderCpuIdle ((uint16_t)3)
#define offsetHeaderMessageId ((uint16_t)4)
#define offsetHeaderMessageLength ((uint16_t)6)
#define offsetHeaderReferenceTime ((uint16_t)8)
#define offsetHeaderTimeStatus ((uint16_t)9)
#define offsetHeaderWeekNumber ((uint16_t)10)
#define offsetHeaderSecondsOfWeek ((uint16_t)12)
#define offsetHeaderReleaseVersion ((uint16_t)20)
#define offsetHeaderLeapSecond ((uint16_t)21)
#define offsetHeaderOutputDelay ((uint16_t)22)

// VERSIONB
#define messageIdVersion ((uint16_t)37)
#define offsetVersionModuleType ((uint16_t)0)
#define offsetVersionFirmwareVersion ((uint16_t)4)
#define offsetVersionAuth ((uint16_t)37)
#define offsetVersionPsn ((uint16_t)166)
#define offsetVersionEfuseID ((uint16_t)232)
#define offsetVersionCompTime ((uint16_t)265)

// BESTNAVB contains HPA, sats tracked/used, lat/long, RTK status, fix status
#define messageIdBestnav ((uint16_t)2118)
#define offsetBestnavPsolStatus ((uint16_t)0)
#define offsetBestnavPosType ((uint16_t)4)
#define offsetBestnavLat ((uint16_t)8)
#define offsetBestnavLon ((uint16_t)16)
#define offsetBestnavHgt ((uint16_t)24)
#define offsetBestnavLatDeviation ((uint16_t)40)
#define offsetBestnavLonDeviation ((uint16_t)44)
#define offsetBestnavHgtDeviation ((uint16_t)48)
#define offsetBestnavSatsTracked ((uint16_t)64)
#define offsetBestnavSatsUsed ((uint16_t)65)
#define offsetBestnavExtSolStat ((uint16_t)69)
#define offsetBestnavVelType ((uint16_t)76)
#define offsetBestnavHorSpd ((uint16_t)88)
#define offsetBestnavTrkGnd ((uint16_t)96)
#define offsetBestnavVertSpd ((uint16_t)104)
#define offsetBestnavVerspdStd ((uint16_t)112)
#define offsetBestnavHorspdStd ((uint16_t)116)

// BESTNAVXYZB
#define messageIdBestnavXyz ((uint16_t)240)
#define offsetBestnavXyzPsolStatus ((uint16_t)0)
#define offsetBestnavXyzPosType ((uint16_t)4)
#define offsetBestnavXyzPX ((uint16_t)8)
#define offsetBestnavXyzPY ((uint16_t)16)
#define offsetBestnavXyzPZ ((uint16_t)24)
#define offsetBestnavXyzPXDeviation ((uint16_t)32)
#define offsetBestnavXyzPYDeviation ((uint16_t)36)
#define offsetBestnavXyzPZDeviation ((uint16_t)40)
#define offsetBestnavXyzSatsTracked ((uint16_t)104)
#define offsetBestnavXyzSatsUsed ((uint16_t)105)
#define offsetBestnavXyzExtSolStat ((uint16_t)109)

// RECTIMEB for time/date
#define messageIdRectime ((uint16_t)102)
#define offsetRectimeClockStatus ((uint16_t)0)
#define offsetRectimeOffset ((uint16_t)4)
#define offsetRectimeOffsetStd ((uint16_t)12)
#define offsetRectimeUtcYear ((uint16_t)28)
#define offsetRectimeUtcMonth ((uint16_t)32)
#define offsetRectimeUtcDay ((uint16_t)33)
#define offsetRectimeUtcHour ((uint16_t)34)
#define offsetRectimeUtcMinute ((uint16_t)35)
#define offsetRectimeUtcMillisecond ((uint16_t)36)
#define offsetRectimeUtcStatus ((uint16_t)40)

// HWSTATUS has temperature info, and voltage info
#endif

class LG290P
{
  private:
    unsigned long lastUpdateGeodetic = 0;
    unsigned long lastUpdateEcef = 0;
    unsigned long lastUpdateDateTime = 0;
    unsigned long lastUpdateVersion = 0;

    std::map<std::string, int> nmeaCounters;
    std::map<int, int> rtcmCounters;
    typedef void (*nmeaCallback)(NmeaPacket &nmea);
    typedef void (*rtcmCallback)(RtcmPacket &rtcm);
    std::map<std::string, nmeaCallback> nmeaSubscriptions;
    std::map<int, rtcmCallback> rtcmSubscriptions;
    
    void clearCounters() { nmeaCounters.clear(); rtcmCounters.clear(); }

    void stopAutoReports(); // Delete all pointers to force reinit next time a helper function is called

#if false
    bool isNmeaFixed(); // Returns true when GNGGA NMEA reports position status >= 1
    LG290PResult getGeodetic(uint16_t maxWaitMs = 1500);
    LG290PResult updateEcef(uint16_t maxWaitMs = 1500);
    LG290PResult updateDateTime(uint16_t maxWaitMs = 1500);
#endif

    Print *_debugPort = nullptr; // The stream to send debug messages to if enabled. Usually Serial.

    SEMP_PARSE_STATE *_sempParse; // State of the SparkFun Extensible Message Parser

    bool lg290PLibrarySemaphoreBlock = false; // Gets set to true when the Unicore library needs to interact directly
                                               // with the serial hardware
#if false
    char configStringToFind[100] = {'\0'};
    bool configStringFound = false; // configHandler() sets true if we find the intended string
#endif

    bool _printBadChecksum = false;       // Display bad checksum message from the parser
    bool _printParserTransitions = false; // Display the parser transitions
    bool _printRxMessages = false;        // Display the received message summary
    bool _dumpRxMessages = false;         // Display the received message hex dump

    std::string commandName;  // The specific command response the parser is hunting for
    uint8_t commandResponse = LG290P_RESULT_OK; // Gets EOM result from parser
    NmeaPacket pqtmResponse;  // If found, parse puts resulting $PQTM sentence here
    static void LG290PProcessMessage(SEMP_PARSE_STATE *parse, uint16_t type);
    void nmeaHandler(SEMP_PARSE_STATE *parse);
    void rtcmHandler(SEMP_PARSE_STATE *parse);
    HardwareSerial *_hwSerialPort = nullptr;
    NmeaSnapshot *snapshot = nullptr;

  public:
    // Client interface
    bool begin(HardwareSerial &serialPort, Print *parserDebug = nullptr, Print *parserError = &Serial);
    bool isConnected();
    bool isBlocking();
    bool update();
    bool updateOnce();

    // Debugging
    void debugPrintf(const char *format, ...);
    void enableDebugging(Print &debugPort = Serial);
    void disableDebugging();
    void enableParserDebug(Print *print = &Serial);
    void disableParserDebug();
    void enableParserErrors(Print *print = &Serial);
    void disableParserErrors();
    void enablePrintBadChecksums();
    void disablePrintBadChecksums();
    void enablePrintParserTransitions();
    void disablePrintParserTransitions();
    void enablePrintRxMessages();
    void disablePrintRxMessages();
    void enableRxMessageDump();
    void disableRxMessageDump();
    void printParserConfiguration(Print *print = &Serial);
    void dumpBuffer(const uint8_t *buffer, uint16_t length);

    // Statistics
    std::map<std::string, int> &getNmeaCounters() { return nmeaCounters; }
    std::map<int, int> &getRtcmCounters() { return rtcmCounters; }

    // Mode
#if true
    bool setModeBase();
    bool setModeRover();
    bool setPortBaudrate(int port, uint32_t newBaud);
    bool setBaudrate(uint32_t newBaud);
    bool getPortInfo(int port, uint32_t &newBaud, uint8_t &dataBits, uint8_t &parity, uint8_t &stop, uint8_t &flowControl);
    bool enablePPS(uint16_t duration, bool alwaysOutput, bool positivePolarity = true);
    bool disablePPS();
    bool getPPSInfo(bool &enabled, uint16_t &duration, bool &alwaysOutput, bool &positivePolarity);
    bool getConstellationInfo(bool &enableGPS, bool &enableGLONASS, bool &enableGalileo, bool &enableBDS,
      bool &enableQZSS, bool &enableNavIC);
    bool configureConstellation(bool enableGPS, bool enableGLONASS, bool enableGalileo, bool enableBDS,
      bool enableQZSS, bool enableNavIC);
    bool getSerialNumber(std::string &serial);
    bool getVersionInfo(std::string &version, std::string &buildDate, std::string &buildTime);
    bool getFixInterval(uint16_t &fixInterval);
    bool setFixInterval(uint16_t fixInterval);
    bool factoryReset();
    bool coldReset();
    bool warmReset();
    bool hotReset();
    bool setMessageRate(const char *msgName, int rate, int msgver = -1);
    bool nmeaSubscribe(const char *msgName, nmeaCallback callback);
    bool nmeaUnsubscribe(const char *msgName);
    bool rtcmSubscribe(uint16_t type, rtcmCallback callback);
    bool rtcmUnsubscribe(uint16_t type);
    bool disableEngine();
    bool enableEngine();
    bool saveParameters();
    bool restoreParameters();

#else
    bool setMode(const char *modeType);
    bool setModeBase(const char *baseType);
    bool setModeBaseGeodetic(double latitude, double longitude, double altitude);
    bool setModeBaseECEF(double coordinateX, double coordinateY, double coordinateZ);
    bool setModeBaseAverage();
    bool setModeBaseAverage(uint16_t averageTime);
    bool setModeRover(const char *roverType);
    bool setModeRoverSurvey();
    bool setModeRoverUAV();
    bool setModeRoverAutomotive();
    bool setModeRoverMow();

    // Config
    bool setPortBaudrate(const char *comName, unsigned long newBaud);
    bool setBaudrate(unsigned long newBaud);
    bool enablePPS(uint32_t widthMicroseconds, uint16_t periodMilliseconds, bool positivePolarity = true,
                   int16_t rfDelay = 0, int16_t userDelay = 0);
    bool disablePPS();
    bool configurePPS(const char *configString);

    // Mask
    bool enableConstellation(const char *constellationName);
    bool disableConstellation(const char *constellationName);
    bool setElevationAngle(int16_t elevationDegrees, const char *constellationName);
    bool setElevationAngle(int16_t elevationDegrees);
    bool setMinCNO(uint8_t dBHz);
    bool enableFrequency(const char *frequencyName);
    bool disableFrequency(const char *frequencyName);
    bool enableSystem(const char *systemName);
    bool disableSystem(const char *systemName);
#endif

    // Data output
    bool setNMEAPortMessage(const char *sentenceType, const char *comName, float outputRate);
    bool setNMEAMessage(const char *sentenceType, float outputRate);
    bool setRTCMPortMessage(const char *sentenceType, const char *comName, float outputRate);
    bool setRTCMMessage(const char *sentenceType, float outputRate);

    // Other
    bool disableOutput();
    bool disableOutputPort(const char *comName);
    bool saveConfiguration(uint16_t maxWaitMs = 1500);

    uint16_t serialAvailable();
    uint8_t serialRead();
    void serialPrintln(const char *command);
    void clearBuffer();

    bool transmit(const char *command, const char *parms);
    bool sendCommand(const char *command, const char *parms = "", uint16_t maxWaitMs = 1500);
    bool sendCommandLine(const char *command, uint16_t maxWaitMs = 1500);
    NmeaPacket &getCommandResponse() { return pqtmResponse; }
    bool sendOkCommand(const char *command, const char *parms = "", uint16_t maxWaitMs = 1500);

    // Main snapshot helper functions
    bool isNewSnapshotAvailable(); 
    double getLatitude();
    double getLongitude();
    double getAltitude();
    double getHorizontalSpeed();
    double getVerticalSpeed();
    double getTrackGround();
    double getCourse();

    float getLatitudeDeviation();
    float getLongitudeDeviation();
    float getAltitudeDeviation();
    float getHorizontalSpeedDeviation();
    float getVerticalSpeedDeviation();

    double getEcefX();
    double getEcefY();
    double getEcefZ();
    float getEcefXDeviation();
    float getEcefYDeviation();
    float getEcefZDeviation();

    uint8_t getSIV();
    uint8_t getSatellitesTracked();
    uint8_t getSatellitesUsed();
    uint8_t getSolutionStatus();
    uint8_t getPositionType();
    uint8_t getVelocityType();

    uint8_t getRTKSolution();
    uint8_t getPseudorangeCorrection();

    uint16_t getYear();
    uint8_t getMonth();
    uint8_t getDay();
    uint8_t getHour();
    uint8_t getMinute();
    uint8_t getSecond();
    uint16_t getMillisecond();
    uint8_t getTimeStatus();
    uint8_t getDateStatus();
    double getTimeOffset();
    double getTimeOffsetDeviation();

    uint32_t getFixAgeMilliseconds(); // Based on Geodetic report

    uint8_t getModelType();
    char *getVersion();
    char *getID();
    char *getCompileTime();

    char *getVersionFull(uint16_t maxWaitMs = 1500);

#if false
    // Limit maxWaitMs for CONFIG interactions. 800ms good. 500ms too short.
    // because we rely on response timeout - there is no known end to the CONFIG response
    bool isConfigurationPresent(const char *configStringToFind, uint16_t maxWaitMs = 800);

    void unicoreHandler(uint8_t *data, uint16_t length);
    void configHandler(uint8_t *response, uint16_t length);
#endif
    bool initSnapshot();

#if false
    bool initBestnav(uint8_t rate = 1);
    UNICORE_BESTNAV_t *packetBESTNAV = nullptr;

    bool initBestnavXyz(uint8_t rate = 1);
    UNICORE_BESTNAVXYZ_t *packetBESTNAVXYZ = nullptr;

    bool initRectime(uint8_t rate = 1);
    UNICORE_RECTIME_t *packetRECTIME = nullptr;

    bool initVersion();
    UNICORE_VERSION_t *packetVERSION = nullptr;
#endif
};

#endif //_SPARKFUN_LG290P_GNSS_ARDUINO_LIBRARY_H
