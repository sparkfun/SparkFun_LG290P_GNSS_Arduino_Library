/*
  Enable HAS corrections
  By: Nathan Seidle and Mikal Hart
  Date: January 3, 2026
  License: Public domain - do whatever you'd like.

  This example shows how to enable and monitor the High Accuracy Service (HAS) convergence.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  LG290P Breakout: https://www.sparkfun.com/sparkfun-quadband-gnss-rtk-breakout-lg290p-qwiic.html
  LG290P PiHat: https://www.sparkfun.com/sparkfun-gnss-flex-phat-lg290p.html

  Hardware Connections:
  Connect RX3 (green wire) of the LG290P to pin 14 on the ESP32
  Connect TX3 (orange wire) of the LG290P to pin 13 on the ESP32
  To make this easier, a 4-pin locking JST cable can be purchased here: https://www.sparkfun.com/products/17240
  Note: Almost any ESP32 pins can be used for serial.
  Connect a multi-band GNSS antenna: https://www.sparkfun.com/products/21801
*/

#include <SparkFun_LG290P_GNSS.h> // Click here to get the library: http://librarymanager/All#SparkFun_LG290P

// Adjust these values according to your configuration
int pin_UART1_TX = 14;
int pin_UART1_RX = 13;
int gnss_baud = 460800;

LG290P myGNSS;
HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun LG290P HAS E6 example");

  Serial.println("Initializing device...");

  // Increase buffer size to handle high baud rate streams
  // SerialGNSS.setRxBufferSize(1024 * 4);
 
  SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  //myGNSS.enableDebugging(Serial);        // Print all debug to Serial
  if (myGNSS.begin(SerialGNSS) == false) // Give the serial port over to the library
  {
    Serial.println("LG290P failed to respond. Check ports and baud rates. Freezing...");
    while (true)
      ;
  }
  Serial.println("LG290P detected!");

  Serial.println("Ensuring ROVER mode");
  myGNSS.ensureModeRover();

  if (myGNSS.setHighAccuracyService(2, 1) == true) // Enable HAS E6 corrections, use WGS84 datum
    Serial.println("HAS E6 corrections enabled.");
  else
    Serial.println("Failed to enable HAS E6 corrections.");

  delay(2000); // Wait before we read the settings 

  int mode = 0;
  int datum = 0;
  int timeout = 0;
  float horstd = 0;
  float verstd = 0;

  if (myGNSS.getHighAccuracyService(mode, datum, timeout, horstd, verstd) == true)
  {
    Serial.print("HAS Service: mode=");
    Serial.print(mode);
    Serial.print(", datum=");
    Serial.print(datum);
    Serial.print(", timeout=");
    Serial.print(timeout);
    Serial.print(", horstd=");
    Serial.print(horstd);
    Serial.print(", verstd=");
    Serial.println(verstd);
  }
  else
  {
    Serial.println("Failed to get HAS service info.");
  }
}

void loop()
{
  myGNSS.update(); // Must be called often to process incoming data

  if (myGNSS.getPppSolutionType() == 6 || myGNSS.getPppSolutionType() == 7)
  {
    if (myGNSS.getPppSolutionType() == 6)
      Serial.println("HAS Detected! PPP Solution: PPP Converging");
    else if (myGNSS.getPppSolutionType() == 7)
      Serial.println("HAS Converged! PPP Solution: PPP Converged");

    Serial.printf("DatumID: %d ", myGNSS.getDatumId());
    if(myGNSS.getDatumId() == 1)
      Serial.println("WGS84");
    else if(myGNSS.getDatumId() == 2)
      Serial.println("PPP Original");
    else if(myGNSS.getDatumId() == 3)
      Serial.println("CGCS2000");
    
    Serial.printf("Diff ID: %d ", myGNSS.getPppDifferentialId());
    if(myGNSS.getPppDifferentialId() == 9001)
      Serial.println("B2b PPP");
    else if(myGNSS.getPppDifferentialId() == 9002)
      Serial.println("HAS/E6");

    Serial.printf("Diff Age: %d seconds\r\n", myGNSS.getPppDifferentialAge());
  }
  else
  {
    Serial.printf("PPP Solution Type: %d ", myGNSS.getPppSolutionType());
    if(myGNSS.getPppSolutionType() == 1)
      Serial.println("Single");
    else if(myGNSS.getPppSolutionType() == 2)
      Serial.println("Differential (SBAS / DGPS)");
  }

  int horizontalPositionalAccuracy = myGNSS.get2DError() * 1000; // Convert meters to mm
  int threeDPositionalAccuracy = myGNSS.get3DError() * 1000;
  
  Serial.printf("2D Error: %dmm 3D Error: %dmm Sats Used: %d Sats in View: %d\r\n",
                horizontalPositionalAccuracy, threeDPositionalAccuracy, myGNSS.getSatellitesUsedCount(), myGNSS.getSatellitesInViewCount());

  Serial.println();
  
  // Generally speaking, delaying between checks is a bad idea because the LG290P is outputting so much serial. 
  // Instead, call myGNSS.update() far more often in order to parse it all.
  // If you must do other things and delay a lot, then increase setRxBufferSize to 4096 before beginning the serial port (see above).
  delay(1000); // A bad delay to demonstrate the parsing issue
}