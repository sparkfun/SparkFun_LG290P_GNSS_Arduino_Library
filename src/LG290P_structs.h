#ifndef _SPARKFUN_UNICORE_STRUCTS_H
#define _SPARKFUN_UNICORE_STRUCTS_H
#include <string>
#include <vector>

typedef struct
{
    bool newDataAvailable = false;
    double latitude = 0;
    double longitude = 0;
    double altitude = 0;
    double horizontalSpeed = 0;
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint32_t nanosecond = 0;
    uint8_t satellitesUsed = 0;
    char quality = '0';
    double hdop = 0;
    char fixStatus = 'N';
    double course = 0;
//***
    uint8_t timeStatus = 3; // 0 = valid, 3 = invalid
    uint8_t dateStatus = 0; // 0 = Invalid, 1 = valid, 2 = leap second warning
    double verticalSpeed = 0;
    double trackGround = 0;

    float latitudeDeviation = 0;
    float longitudeDeviation = 0;
    float heightDeviation = 0;
    float horizontalSpeedDeviation = 0;
    float verticalSpeedDeviation = 0;

    uint8_t positionType = 0; // 0 = None, 1 = FixedPos, 8 = DopplerVelocity, 16 = Single, ...
    uint8_t velocityType = 0; // 0 = None, 1 = FixedPos, 8 = DopplerVelocity, 16 = Single, ...

    uint8_t
        solutionStatus = 0; // 0 = Solution computed, 1 = Insufficient observation, 3 = No convergence, 4 = Covariance trace

    uint8_t satellitesTracked = 0;

    uint8_t rtkSolution = 0;
    uint8_t pseudorangeCorrection = 0;

} NmeaSnapshot;

class NmeaPacket
{
    std::vector<std::string> fields;
    bool hasChecksum = false;
    bool checksumValid = false;
    size_t charCount = 0;
    static void parseTime(const std::string &term, uint8_t &hour, uint8_t &minute, uint8_t &second, uint32_t &nanos);
    static void parseDate(const std::string &term, uint16_t &year, uint8_t &month, uint8_t &day);
    static void parseLocation(const std::string &termLat, const std::string &termNS, const std::string &termLong, const std::string &termEW, double &latitude, double &longitude);
    static void parseQuality(const std::string &term, char &quality);
    static void parseSatelliteCount(const std::string &term, uint8_t &used_satellites);
    static int32_t parseDecimal(const std::string &term);
    static void parseDegrees(const char *degTerm, const char *nsewTerm, double &angle);
    static void parseFixStatus(const std::string &term, char &fixStatus);
    static void parseSpeed(const std::string &term, double &speed);
    static void parseCourse(const std::string &term, double &course);
    static void parseHdop(const std::string &term, double &hdop);
    static void parseAltitude(const std::string &term, double &altitude);
   
public:
    /// @brief Indicates whether the sentence has reasonable NMEA construction
    /// @details Note: This method returns true if the sentence *has* a checksum, even if it's wrong
    /// @return true if sentence is valid NMEA
    bool IsValid() const            { return fields.size() > 0 && fields[0].length() > 1 && fields[0][0] == '$' && hasChecksum; }
    /// @brief Tests whether the NMEA checksum is correct
    /// @return true if Checksum is present and valid
    bool ChecksumValid() const      { return checksumValid; }
    bool HasChecksum() const        { return hasChecksum; }
    std::string ToString() const;
    size_t Size() const             { return charCount; }

    std::string TalkerId() const;
    std::string SentenceId() const;
    void Clear()                    { fields.clear(); }
    std::string operator[](int field) const      
                                    { return field >= fields.size() ? "" : fields[field]; }
    size_t FieldCount() const       { return fields.size(); }

    void processGGA(NmeaSnapshot *snapshot);
    void processRMC(NmeaSnapshot *snapshot);

    static NmeaPacket FromString(const std::string &str);
};

struct RtcmPacket
{
    uint8_t *buffer;
    uint16_t bufferlen;
    uint16_t payloadLen;
    uint16_t type;
};

#if false
typedef struct
{
    double latitude;
    double longitude;
    double altitude;
    double horizontalSpeed;
    double verticalSpeed;
    double trackGround;

    float latitudeDeviation;
    float longitudeDeviation;
    float heightDeviation;
    float horizontalSpeedDeviation;
    float verticalSpeedDeviation;

    uint8_t positionType; // 0 = None, 1 = FixedPos, 8 = DopplerVelocity, 16 = Single, ...
    uint8_t velocityType; // 0 = None, 1 = FixedPos, 8 = DopplerVelocity, 16 = Single, ...

    uint8_t
        solutionStatus; // 0 = Solution computed, 1 = Insufficient observation, 3 = No convergence, 4 = Covariance trace

    uint8_t satellitesTracked;
    uint8_t satellitesUsed;

    uint8_t rtkSolution = 0;
    uint8_t pseudorangeCorrection = 0;
} UNICORE_BESTNAV_data_t;

typedef struct
{
    // ubxAutomaticFlags automaticFlags;
    UNICORE_BESTNAV_data_t data;
    void (*callbackPointerPtr)(UNICORE_BESTNAV_data_t *);
    UNICORE_BESTNAV_data_t *callbackData;
} UNICORE_BESTNAV_t;

typedef struct
{
    double ecefX = 0;
    double ecefY = 0;
    double ecefZ = 0;
    float ecefXDeviation = 0;
    float ecefYDeviation = 0;
    float ecefZDeviation = 0;
} UNICORE_BESTNAVXYZ_data_t;

typedef struct
{
    // ubxAutomaticFlags automaticFlags;
    UNICORE_BESTNAVXYZ_data_t data;
    void (*callbackPointerPtr)(UNICORE_BESTNAVXYZ_data_t *);
    UNICORE_BESTNAVXYZ_data_t *callbackData;
} UNICORE_BESTNAVXYZ_t;

typedef struct
{
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint16_t millisecond = 0;
    uint8_t timeStatus = 3; // 0 = valid, 3 = invalid
    uint8_t dateStatus = 0; // 0 = Invalid, 1 = valid, 2 = leap second warning
    double timeOffset = 0;
    double timeDeviation = 0;
} UNICORE_RECTIME_data_t;

typedef struct
{
    // ubxAutomaticFlags automaticFlags;
    UNICORE_RECTIME_data_t data;
    void (*callbackPointerPtr)(UNICORE_RECTIME_data_t *);
    UNICORE_RECTIME_data_t *callbackData;
} UNICORE_RECTIME_t;

// #VERSION,98,GPS,UNKNOWN,1,711000,0,0,18,144;UM980,R4.10Build7923,HRPT00-S10C-P,2310415000001-MD22B1224961040,ff3bd496fd7ca68b,2022/09/28*55f61e51
typedef struct
{
    uint8_t modelType = 0;
    char swVersion[33 + 1] = {0}; // Add terminator
    char efuseID[33 + 1] = {0};   // Add terminator
    char compileTime[43 + 1] = {0};   // Add terminator
} UNICORE_VERSION_data_t;

typedef struct
{
    // ubxAutomaticFlags automaticFlags;
    UNICORE_VERSION_data_t data;
    void (*callbackPointerPtr)(UNICORE_VERSION_data_t *);
    UNICORE_VERSION_data_t *callbackData;
} UNICORE_VERSION_t;
#endif

#endif // _SPARKFUN_UNICORE_STRUCTS_H
