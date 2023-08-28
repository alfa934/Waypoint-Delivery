// https://electropeak.com/learn/interfacing-neo-7m-gps-module-with-arduino/
#include <Wire.h> 
#include <TinyGPS++.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
#define GPSSerial Serial1
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;


void setup()
{
    Serial.begin(115200);
    GPSSerial.begin(GPSBaud);
    Serial.println(F("Initialising GPS..."));
    delay(2000);
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (GPSSerial.available() > 0)
    if (gps.encode(GPSSerial.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo()
{
    if(gps.location.isUpdated()){
        
        Serial.print("SAT    LAT    LNG:        ");
        Serial.print(gps.satellites.value());
        Serial.print("    ");
        Serial.print(gps.location.lat(), 6);
        Serial.print("\t");
        Serial.println(gps.location.lng(), 6);

        delay(1000);
    }
    
}
