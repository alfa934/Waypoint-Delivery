#include <Wire.h>
#include <TinyGPS++.h>
#include <HMC5883L.h>

// Create GPS instance
TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;

// Create Compass instance
HMC5883L cmps;

/************ NAVIGATION PARAMETERS ************/
int headingTarget, headingCurrent, headingError;
int distToTarget, originalDistToTarget;
float currentLat, currentLong;

float waypointList[][2] = {
/** {LAT, LONG } **/
    { 0.0, 0.0 },
    { 1.0, 1.0 },
    { 2.0, 2.0 }
};

float targetLat = waypointList[0][0];
float targetLong = waypointList[0][1];

#define HEADING_TOLERANCE 8    // +/- in degrees
#define WAYPOINT_TOLERANCE 3   // +/- in metres
#define STOP_WAYPOINT 5        // in seconds

/*
 * Serial1 -> pins 19 (RX) and 18 (TX), 
 * Serial2 -> pins 17 (RX) and 16 (TX), 
 * Serial3 -> pins 15 (RX) and 14 (TX).
 */

/* FUNCTIONS */
float GetHeadingDegrees();

void setup() {
    
    Serial.begin(9600); // Display Information
    
    /************************* Start Compass Connection *************************/
    while(!cmps.begin()) {
        Serial.println(F("Searching for HMC5883L sensor! Check wiring"));
        delay(500);
    }
    Serial.println(F("HMC5883L Connected!...");
    // Set measurement range
    cmps.setRange(HMC5883L_RANGE_1_3GA);
    // Set measurement mode
    cmps.setMeasurementMode(HMC5883L_CONTINOUS);
    // Set data rate
    cmps.setDataRate(HMC5883L_DATARATE_30HZ);
    // Set number of samples averaged
    cmps.setSamples(HMC5883L_SAMPLES_8);
    // Set calibration offset. See HMC5883L_calibration.ino
    cmps.setOffset(0, 0, 0); // (x, y, x) FIND OFFSET
    
    /************************* Start GPS communication *************************/
    Serial1.begin(GPSBaud);
    
}

void loop() {
    // put your main code here, to run repeatedly:

}

float GetHeadingDegrees()
{
    Vector norm = cmps.readNormalize();
    
    float heading = atan2(norm.YAxis, norm.XAxis);
    
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
    heading += declinationAngle;
    
    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0)
        heading += 2 * PI;
    
    if (heading > 2 * PI)
        heading -= 2 * PI;
    
    return heading * (180/M_PI);
}
