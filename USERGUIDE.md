# LG290P GNSS Module Interface

Warning: AI generated MD.
This library provides an interface for controlling and configuring an LG290P GNSS (Global Navigation Satellite System) module, including functionalities such as communication, debugging, satellite data retrieval, and device configuration.

## Table of Contents
- [Initialization](#initialization)
- [Debugging](#debugging)
- [Device Configuration](#device-configuration)
- [Resets and Engine Control](#resets-and-engine-control)
- [Message Subscriptions](#message-subscriptions)
- [Satellite Information](#satellite-information)
- [Survey Mode](#survey-mode)
- [Geodetic Information](#geodetic-information)
- [Statistics](#statistics)

## Initialization
- **`begin(HardwareSerial &serialPort, Print *parserDebug = nullptr, Print *parserError = &Serial)`**: Initializes the GNSS module with the specified serial port.
- **`isConnected()`**: Checks if the GNSS module is connected.
- **`isBlocking()`**: Returns whether the module is in blocking mode.
- **`update()`**: Updates the GNSS data.
- **`updateOnce()`**: Updates the GNSS data only once.

## Debugging
- **`debugPrintf(const char *format, ...)`**: Prints formatted debug information.
- **`enableDebugging(Print &debugPort = Serial)`**: Enables debugging on the specified port.
- **`disableDebugging()`**: Disables debugging output.
- **`enableParserDebug(Print *print = &Serial)`**: Enables parser debugging.
- **`disableParserDebug()`**: Disables parser debugging.
- **`enableParserErrors(Print *print = &Serial)`**: Enables error messages from the parser.
- **`disableParserErrors()`**: Disables error messages from the parser.
- **`enablePrintBadChecksums()`**: Enables the printing of messages with bad checksums.
- **`disablePrintBadChecksums()`**: Disables the printing of messages with bad checksums.

## Device Configuration
- **`setModeBase()`**: Configures the device to operate as a base station.
- **`setModeRover()`**: Configures the device to operate as a rover.
- **`setPortBaudrate(int port, uint32_t newBaud)`**: Sets the baud rate of the specified port.
- **`setBaudrate(uint32_t newBaud)`**: Sets the baud rate for the device.
- **`getPortInfo(int port, uint32_t &newBaud, uint8_t &dataBits, uint8_t &parity, uint8_t &stop, uint8_t &flowControl)`**: Retrieves the configuration of a specified port.

## Resets and Engine Control
- **`softwareReset()`**: Performs a software reset.
- **`coldReset()`**: Performs a cold reset, clearing all stored data.
- **`warmReset()`**: Performs a warm reset, retaining some data.
- **`hotReset()`**: Performs a hot reset, retaining most data.

## Message Subscriptions
- **`nmeaSubscribe(const char *msgName, nmeaCallback callback)`**: Subscribes to specific NMEA messages.
- **`nmeaUnsubscribe(const char *msgName)`**: Unsubscribes from specific NMEA messages.
- **`rtcmSubscribe(uint16_t type, rtcmCallback callback)`**: Subscribes to specific RTCM messages.
- **`rtcmUnsubscribe(uint16_t type)`**: Unsubscribes from specific RTCM messages.

## Satellite Information
- **`getVisibleSats(const char *talker = nullptr)`**: Retrieves a list of visible satellites.
- **`isNewSatelliteInfoAvailable()`**: Checks if new satellite information is available.

## Survey Mode
- **`getSurveyMode(int &mode, int &positionTimes, double &accuracyLimit, double &ecefX, double &ecefY, double &ecefZ)`**: Retrieves the current survey mode configuration.
- **`setSurveyInMode(int positionTimes, double accuracyLimit=0)`**: Configures the module to survey-in mode.

## Geodetic Information
- **`getLatitude()`**: Retrieves the current latitude.
- **`getLongitude()`**: Retrieves the current longitude.
- **`getAltitude()`**: Retrieves the current altitude.
- **`getHorizontalSpeed()`**: Retrieves the current horizontal speed.
- **`getVerticalSpeed()`**: Retrieves the current vertical speed.
- **`getTrackGround()`**: Retrieves the current track over ground.

## Statistics
- **`getNmeaCounters()`**: Retrieves NMEA message statistics.
- **`getRtcmCounters()`**: Retrieves RTCM message statistics.
