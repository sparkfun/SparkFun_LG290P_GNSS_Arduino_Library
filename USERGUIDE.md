Types

    // Types
    LG290P
    NmeaPacket
    RtmcPacket

Functions

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

    // Mode
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
        bool enableSystem(const char *systemName);
    bool disableSystem(const char *systemName);

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

    // Limit maxWaitMs for CONFIG interactions. 800ms good. 500ms too short.
    // because we rely on response timeout - there is no known end to the CONFIG response
    bool isConfigurationPresent(const char *configStringToFind, uint16_t maxWaitMs = 800);

    bool initSnapshot(uint8_t rate = 1);
