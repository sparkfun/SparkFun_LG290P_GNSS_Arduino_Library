# LG290P GNSS Module Interface

Warning: AI generated MD.
This library provides an interface for controlling and configuring an LG290P GNSS (Global Navigation Satellite System) module, including functionalities such as communication, debugging, satellite data retrieval, and device configuration.

## Table of Contents

  - [Handshaking and Client Interface](#handshaking-and-client-interface)
  - [Debugging](#debugging)
  - [Statistics](#statistics)
  - [Device Configuration](#device-configuration)
  - [Resets and Engine Control](#resets-and-engine-control)
  - [Message Subscriptions](#message-subscriptions)
  - [Command Handling](#command-handling)
  - [Satellite Information](#satellite-information)
  - [Survey Mode](#survey-mode)
  - [PVT Domain Reporting](#pvt-domain-reporting)
  - [Date and Time Information (PVT)](#date-and-time-information-pvt)
  - [EPE Error Domain](#epe-error-domain)
  - [PL Protection Level Domain](#pl-protection-level-domain)

## Handshaking and Client Interface

- **`bool begin(HardwareSerial &serialPort, Print *parserDebug = nullptr, Print *parserError = &Serial);`**  
  Initialize communication with the GNSS module using a serial port. Optional debugging and error printing can be specified.

- **`bool beginAutoBaudDetect(HardwareSerial &serialPort, int rxPin, int txPin, Print *parserDebug = nullptr, Print *parserError = &Serial);`**  
  Auto-detect which baud rate the port is connected at. Repeatedly calls 'begin()' above, trying all the various baud rates that the LG290P supports, starting with the default, 480600. Optional debugging and error printing can be specified.

- **`bool isConnected();`**  
  Check if the GNSS module is connected.

- **`bool isBlocking();`**  
  Check if the communication with the GNSS module is blocking other operations.

- **`bool update();`**  
  Update the GNSS module's state by processing any received data.

## Debugging

- **`void debugPrintf(const char *format, ...);`**  
  Print formatted debug information.

- **`void enableDebugging(Print &debugPort = Serial);`**  
  Enable debugging output to a specified port (default: Serial).

- **`void disableDebugging();`**  
  Disable debugging output.

- **`void enableParserDebug(Print *print = &Serial);`**  
  Enable debugging for message parsing.

- **`void disableParserDebug();`**  
  Disable debugging for message parsing.

- **`void enableParserErrors(Print *print = &Serial);`**  
  Enable parser error messages.

- **`void disableParserErrors();`**  
  Disable parser error messages.

- **`void enablePrintBadChecksums();`**  
  Enable printing of bad checksums for received messages.

- **`void disablePrintBadChecksums();`**  
  Disable printing of bad checksums.

- **`void enablePrintParserTransitions();`**  
  Enable printing of parser state transitions.

- **`void disablePrintParserTransitions();`**  
  Disable printing of parser state transitions.

- **`void enablePrintRxMessages();`**  
  Enable printing of received messages.

- **`void disablePrintRxMessages();`**  
  Disable printing of received messages.

- **`void enableRxMessageDump();`**  
  Enable full message dump of received messages.

- **`void disableRxMessageDump();`**  
  Disable full message dump of received messages.

- **`void printParserConfiguration(Print *print = &Serial);`**  
  Print current parser configuration.

- **`void dumpBuffer(const uint8_t *buffer, uint16_t length);`**  
  Dump the contents of a buffer to the debug port.

## Statistics

- **`int getNmeaCount(const char *sentenceId = nullptr);`**  
  Get the count of NMEA messages received. Optionally, filter by sentence ID.

- **`int getRtcmCount(int packetType = -1);`**  
  Get the count of RTCM messages received. Optionally, filter by packet type.

- **`void clearNmeaCount();`**  
  Clear NMEA message counters.

- **`void clearRtcmCount();`**  
  Clear RTCM message counters.

## Device Configuration

- **`bool setModeBase(bool resetAfter = true);`**  
  Set the device to BASE mode, optionally doing the necessary reset afterwards.

- **`bool setModeRover(bool resetAfter = true);`**  
  Set the device to ROVER mode, optionally doing the necessary reset afterwards.

- **`bool ensureModeBase();`**  
  Set the device to BASE mode, if it's not already in that mode.

- **`bool ensureModeRover(bool resetAfter = true);`**  
  Set the device to ROVER mode, if it's not already in that mode.

- **`bool getMode(int &mode);`**  
  Gets the device mode (ROVER=1 or BASE=2).

- **`bool setPortBaudrate(int port, uint32_t newBaud);`**  
  Set the baud rate of the specified port.

- **`bool setBaudrate(uint32_t newBaud);`**  
  Set the baud rate of the main communication port.

- **`bool getPortInfo(int port, uint32_t &newBaud, uint8_t &dataBits, uint8_t &parity, uint8_t &stop, uint8_t &flowControl);`**  
  Get information about the specified port.

- **`bool setPortInputProtocols(int port, uint8_t newFlags);`**  
  Enable or disable which protocols are available for input on the specified port.

- **`bool setPortOutputProtocols(int port, uint8_t newFlags);`**  
  Enable or disable which protocols are available for output on the specified port.

- **`bool getPortProtocols(int port, uint8_t &inputFlags, uint8_t &outputFlags);`**  
  Get which protocols are enabled or disabled on the specified port.

- **`bool setPPS(uint16_t duration, bool alwaysOutput, bool positivePolarity = true);`**  
  Enable Pulse-Per-Second (PPS) output with specified parameters.

- **`bool disablePPS();`**  
  Disable Pulse-Per-Second (PPS) output.

- **`bool getPPS(bool &enabled, uint16_t &duration, bool &alwaysOutput, bool &positivePolarity);`**  
  Get information about the PPS configuration.

- **`bool getConstellations(bool &enableGPS, bool &enableGlonass, bool &enableGalileo, bool &enableBds, bool &enableQzss, bool &enableNavIV);`**  
  Get enabled GNSS constellations.

- **`bool setConstellations(bool enableGPS, bool enableGlonass, bool enableGalileo, bool enableBds, bool enableQzss, bool enableNavIC);`**  
  Configure which GNSS constellations to enable.

- **`bool getSerialNumber(std::string &serial);`**  
  Retrieve the serial number of the GNSS module.

- **`bool getVersionInfo(std::string &version, std::string &buildDate, std::string &buildTime);`**  
  Retrieve version information from the GNSS module.

- **`bool getFixInterval(uint16_t &fixInterval);`**  
  Get the current fix interval of the GNSS module.

- **`bool setFixInterval(uint16_t fixInterval);`**  
  Set the fix interval for the GNSS module.

- **`bool setMessageRate(const char *msgName, int rate, int msgver = -1);`**  
  Set the message rate for a specific message type.

- **`bool save();`**  
  Save the current configuration parameters to non-volatile memory.

- **`bool factoryRestore();`**  
  Reset all settings to factory defaults. Need subsequent reset to take effect.

## Resets and Engine Control

- **`bool reset();`**  
  Perform a software reset on the GNSS module.

- **`bool coldStart();`**  
  Perform a cold reset on the GNSS module.

- **`bool warmStart();`**  
  Perform a warm reset on the GNSS module.

- **`bool hotStart();`**  
  Perform a hot reset on the GNSS module.

- **`bool disableEngine();`**  
  Disable the GNSS engine.

- **`bool enableEngine();`**  
  Enable the GNSS engine.

## Message Subscriptions

- **`bool nmeaSubscribe(const char *msgName, nmeaCallback callback);`**  
  Subscribe to a specific NMEA message.

- **`bool nmeaUnsubscribe(const char *msgName);`**  
  Unsubscribe from a specific NMEA message.

- **`bool nmeaSubscribeAll(nmeaCallback callback);`**  
  Subscribe to all NMEA messages.

- **`bool nmeaUnsubscribeAll();`**  
  Unsubscribe from all NMEA messages.

- **`bool rtcmSubscribe(uint16_t type, rtcmCallback callback);`**  
  Subscribe to a specific RTCM message type.

- **`bool rtcmUnsubscribe(uint16_t type);`**  
  Unsubscribe from a specific RTCM message type.

- **`bool rtcmSubscribeAll(rtcmCallback callback);`**  
  Subscribe to all RTCM messages.

- **`bool rtcmUnsubscribeAll();`**  
  Unsubscribe from all RTCM messages.

## Command Handling

- **`bool sendCommand(const char *command, const char *parms = "", uint16_t maxWaitMs = 1500);`**  
  Sends a custom command to the device. Automatically prepends `$` and calculates the checksum if needed. The response can be retrieved with `getCommandResponse()`.

- **`bool sendCommandLine(const char *command, uint16_t maxWaitMs = 1500);`**  
  Sends a full command line to the device, prepending `$` if needed and calculating the checksum.

- **`NmeaPacket &getCommandResponse();`**  
  Retrieves the response to the last successful command issued.

- **`bool sendOkCommand(const char *command, const char *parms = "", uint16_t maxWaitMs = 1500);`**  
  Sends a command expecting an "OK" response. Returns true if the response contains "OK" after the first comma.

## Satellite Information

- **`std::list<satinfo> getVisibleSats(const char *talker = nullptr);`**  
  Retrieves a list of visible satellites. Optionally specify a talker identifier (e.g., "GA") or get all satellites.

- **`bool isNewSatelliteInfoAvailable();`**  
  Checks if new satellite information has been received since the last query.

- **`uint16_t getSatellitesInViewCount();`**  
  Gets the number of satellites currently being tracked.

- **`uint16_t getSatellitesUsedCount();`**  
  Gets the number of satellites used for positioning.

## Survey Mode

- **`bool getSurveyDetails(int &mode, int &positionTimes, double &accuracyLimit, double &ecefX, double &ecefY, double &ecefZ);`**  
  Retrieves the current survey mode settings, including position times and accuracy limit.

- **`uint8_t getSurveyMode();`**  
  Retrieves the current Survey mode.

- **`bool setSurveyInMode(int positionTimes, double accuracyLimit = 0);`**  
  Sets the device to Survey-In mode with a specified number of position times and accuracy limit.

- **`bool setSurveyFixedMode(double ecefX, double ecefY, double ecefZ);`**  
  Sets the device to Fixed survey mode using ECEF coordinates.

- **`bool disableSurveyInMode(bool resetAfter = true);`**  
  Disables Survey In or Survey Fixed mode

## PVT Domain Reporting

- **`bool isNewSnapshotAvailable();`**  
  Checks if new geodetic snapshot data (location, time, date, fix, etc.) is available.

- **`double getLatitude();`**  
  Retrieves the current latitude in degrees.

- **`double getLongitude();`**  
  Retrieves the current longitude in degrees.

- **`double getAltitude();`**  
  Retrieves the current altitude in meters.

- **`double getHorizontalSpeed();`**  
  Retrieves the current horizontal speed in meters per second.

- **`uint8_t getfixQuality();`**  
  Retrieves the fix quality (a number 0+)

- **`char getfixStatus();`**
  Retrieves the fix status from RMC ('V' = void, 'A' = available)

- **`char getHdop();`**
  Retrieves HDOP

- **`char getPdop();`**
  Retrieves PDOP

- **`uint32_t getTimeOfWeek();`**
  Retrieves the number of milliseconds since Sunday midnight UTC

- **`double getGeoidalSeparation();`**
  Retrieves the Geoidal Separation in meters

- **`double getNorthVelocity();`**
  Retrieves the North velocity in meters per second

- **`double getEastVelocity();`**
  Retrieves the East velocity in meters per second

- **`double getDownVelocity();`**
  Retrieves the Down velocity in meters per second

## Date and Time Information (PVT)

- **`uint16_t getYear();`**  
  Retrieves the current year.

- **`uint8_t getMonth();`**  
  Retrieves the current month.

- **`uint8_t getDay();`**  
  Retrieves the current day.

- **`uint8_t getHour();`**  
  Retrieves the current hour.

- **`uint8_t getMinute();`**  
  Retrieves the current minute.

- **`uint8_t getSecond();`**  
  Retrieves the current second.

- **`uint16_t getMillisecond();`**  
  Retrieves the current millisecond.

- **`uint32_t getGeodeticAgeMs();`**  
  Retrieves the age of the last NMEA GGA/RMC report in milliseconds.

- **`uint32_t getEcefAgeMs();`**  
  Retrieves the age of the last RTCM 1005 report in milliseconds.

- **`void geodeticToEcef(double lat, double lon, double alt, double &xOut, double &yOut, double &zOut)`**
  Converts the geodetic position coordinate given by (lat/long/altitude) into an ECEF format coordinate

- **`void ecefToGeodetic(double x, double y, double z, double &latOut, double &lonOut, double &altOut);`**  
  Converts the ECEF position coordinate given by (x, y, z) into a geodetic (lat/long/altitude) format

## EPE Error Domain

- **`double getNorthError();`**  
  Returns the Estimated North Positioning Error in meters.

- **`double getEastError();`**  
  Returns the Estimated East Positioning Error in meters.

- **`double getDownError();`**  
  Returns the Estimated Down Positioning Error in meters.

- **`double get2DError();`**  
  Returns the Estimated 2D Positioning Error in meters.

- **`double get3DError();`**  
  Returns the Estimated 3D Positioning Error in meters.

## PL Protection Level Domain

- **`double getProbUncertainty();`**  
  Returns the Probability of Uncertainty Level per Epoch from PQTMPL in %.

- **`double getProtLevelNorth();`**  
  Returns the Protection Level of North Position from PQTMPL in mm.

- **`double getProtLevelEast();`**  
  Returns the Protection Level of East Position from PQTMPL in mm.

- **`double getProtLevelDown();`**  
  Returns the Protection Level of Down Position from PQTMPL in mm.

- **`double getProtLevelNorthVelocity();`**  
  Returns the Protection Level of North Velocity from PQTMPL in mm/s.

- **`double getProtLevelEastVelocity();`**  
  Returns the Protection Level of East Velocity from PQTMPL in mm/s.

- **`double getProtLevelDownVelocity();`**  
  Returns the Protection Level of Down Velocity from PQTMPL in mm/s.

- **`double getProtLevelTime();`**  
  Returns the Protection Level of Time from PQTMPL in ns.
