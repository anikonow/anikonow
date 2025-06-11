/*
  SD card datalogger

  The circuit:
 ** MOSI - pin 51
 ** MISO - pin 50
 ** CLK - pin 52
 ** CS - pin 53 
 5v
 Ground

#include <SPI.h>
#include <SD.h>

  u-blox SAM-M10q GNSS receiver

  The circuit: (Serial3)
3.3v
Ground
** Tx 14
** Rx 15

  Blue Robotics Ping 1-d

The circuit: (Serial1)
5v
Ground
** Rx 18
** Tx 19
*/

// sd card 
#include <SPI.h>
#include <SD.h>

const int chipSelect = 53;

// gnss
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

SFE_UBLOX_GNSS_SERIAL myGNSS; // SFE_UBLOX_GNSS_SERIAL uses Serial (UART). For I2C or SPI, see Example1 and Example3

#define mySerial Serial3 // Use Serial1 to connect to the GNSS module. Change this if required


// ping 1d

#include "ping1d.h"

static const uint8_t arduinoRxPin = 19; //Serial1 rx
static const uint8_t arduinoTxPin = 18; //Serial1 tx

static Ping1D ping { Serial1 };

static const uint8_t ledPin = 13;


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(1000);
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  String dataString = "";
  dataString += "lat,lon,depth,confidence";

    if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }

//GPS Setup
  Serial.begin(115200);
  delay(1000); 
  Serial.println("SparkFun u-blox Example");

  mySerial.begin(9600); // u-blox F9 and M10 modules default to 38400 baud. Change this if required

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myGNSS.begin(mySerial) == false) //Connect to the u-blox module using mySerial (defined above)
  {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay (1000);
  }

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART1 port to output UBX only (turn off NMEA noise)

//Ping Setup

Serial1.begin(115200);
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  Serial.println("Blue Robotics ping1d-simple.ino");
  while (!ping.initialize()) {
    Serial.println("\nPing device failed to initialize!");
    Serial.println("Are the Ping rx/tx wired correctly?");
    Serial.print("Ping rx is the green wire, and should be connected to Arduino pin 18");
    Serial.print(arduinoTxPin);
    Serial.println(" (Arduino tx)");
    Serial.print("Ping tx is the white wire, and should be connected to Arduino pin 19");
    Serial.print(arduinoRxPin);
    Serial.println(" (Arduino rx)");
    delay(2000);
  }

}

void loop() {
 
 //Ping 1d loop
 {
  if (ping.update()) {
  /*  Serial.print("Distance: ");
    Serial.print(ping.distance()*.003281);
    Serial.print("\tConfidence: ");
    Serial.println(ping.confidence()); 
  */
  } else {
    Serial.println("No update received!");
  }

  // Toggle the LED to show that the program is running
  digitalWrite(ledPin, !digitalRead(ledPin));

 
 // Request (poll) the position, velocity and time (PVT) information.
  // The module only responds when a new position is available. Default is once per second.
  // getPVT() returns true when new data is received.
  if (myGNSS.getPVT() == true)
  {
   /* int32_t latitude = myGNSS.getLatitude();

    int32_t longitude = myGNSS.getLongitude();

    int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
    */
  }
 
 
  // make a string for assembling the data to log:
  String dataString = "";
  /*
  // read three sensors and append to the string:
  for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ",";
    }
  }
  */

  dataString += String(myGNSS.getLatitude());
  dataString += ",";
  
  dataString += String(myGNSS.getLongitude());
  dataString += ",";

  dataString += String(ping.distance()*.003281);
  dataString += ",";
  
  dataString += String(ping.confidence());
  dataString += ",";

  dataString += String(myGNSS.getHour());//,myGNSS.getMinute(),myGNSS.getSecond());
  dataString += ":";
  dataString += String(myGNSS.getMinute());
  dataString += ":";
  dataString += String(myGNSS.getSecond());

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
 }
}










