#include <Wire.h>
#include <TinyGPS++.h>
#include <Adafruit_GPS.h>
#include <HMC5883L.h>

// Create GPS instance
TinyGPSPlus gps;
#define GPSSerial Serial1
Adafruit_GPS SettingGPS(&GPSSerial);

// Create Compass instance
HMC5883L cmps;

/************ NAVIGATION PARAMETERS ************/
float waypointList[][2] = {
/** {LAT, LONG } **/
    { 1.0, 1.0 },
    { 2.0, 2.0 },
    { 3.0, 3.0 },
    { 0.0, 0.0 }
};

#define WAYPOINT_LIST_SIZE sizeof(waypointList)/sizeof(waypointList[0])

int headingTarget, headingCurrent;
int headingError, outputError;
int distToTarget, originalDistToTarget;
float currentLat, currentLong;
float targetLat = waypointList[0][0];
float targetLong = waypointList[0][1];
int waypointIndex = 0;

#define HEADING_TOLERANCE 8             // +/- in degrees
#define WAYPOINT_TOLERANCE 3            // +/- in metres
#define STOP_WAYPOINT (3 * 1000)        // in milliseconds

/*
 * Serial1 -> pins 19 (RX) and 18 (TX), 
 * Serial2 -> pins 17 (RX) and 16 (TX), 
 * Serial3 -> pins 15 (RX) and 14 (TX).
 */

/* FUNCTIONS */
float GetHeadingDegrees();
void GetCurrentLocation();
void CalculateHeading();
void MoveRobot();

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void setup() {
    
    Serial.begin(9600); // Display Information

    /**************************** Start Motor Setup ****************************/
    MotorSetup();
    
    /************************ Start Compass Connection *************************/
    while(!cmps.begin()) {
        Serial.println(F("Searching for HMC5883L sensor! Check wiring"));
        delay(500);
    }
    Serial.println(F("HMC5883L Connected!..."));
    // Set measurement range
    cmps.setRange(HMC5883L_RANGE_1_3GA);
    // Set measurement mode
    cmps.setMeasurementMode(HMC5883L_CONTINOUS);
    // Set data rate
    cmps.setDataRate(HMC5883L_DATARATE_30HZ);
    // Set number of samples averaged
    cmps.setSamples(HMC5883L_SAMPLES_8);
    // Set calibration offset. See How to Calibrate HMC5883L FOR DUMMIES.txt
    cmps.setOffset(-36.77, -1.635, 0); // (x, y, x) FIND OFFSET
    
    /************************* Start GPS communication *************************/
    GPSSerial.begin(9600);
    // Turns on RMC and GGA (fixed data)
    SettingGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // 1 Hz update rate
    SettingGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    // Turn off antenna status info
    SettingGPS.sendCommand(PGCMD_NOANTENNA);

    delay(1000);
}


void loop() {
START:
    /* Get Latitude and Longitude */
    GetCurrentLocation();

    /* Update DISTANCE to next waypoint*/
    distToTarget = TinyGPSPlus::distanceBetween( currentLat, currentLong, 
                                                 targetLat, targetLong );

    /* If Waypoint has been REACHED */
    if(distToTarget <= WAYPOINT_TOLERANCE)
    {
        RobotStop();
        delay(STOP_WAYPOINT);

        /* Change to next Waypoint*/
        waypointIndex++;
        targetLat  = waypointList[waypointIndex][0];
        targetLong = waypointList[waypointIndex][1];

        // if finished
        if( waypointIndex == WAYPOINT_LIST_SIZE )
        {
            RobotStop();
            while(1); // TURN OFF/RESET ROBOT
        } 
        else {
            goto START;
        }
    }

    /* Update COURSE to next waypoint*/
    headingTarget = TinyGPSPlus::courseTo( currentLat, currentLong, 
                                           targetLat, targetLong );

    /* Check if GPS data is valid (Long. is usually +'ve in Indonesia */
    if( currentLong > 1 && targetLong > 1 ) 
    {
        headingCurrent = GetHeadingDegrees();
        
        CalculateHeading();

        MoveRobot();

        // not sure what's next
    }
    
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println(F("No GPS detected: check wiring."));
        while(true);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

float GetHeadingDegrees()
{
    Vector norm = cmps.readNormalize();
    
    float heading = atan2(norm.YAxis, norm.XAxis);
    
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
    float declinationAngle = (0.0 + (34.0 / 60.0)) / (180 / M_PI);
    heading += declinationAngle;
    
    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0)
        heading += 2 * PI;
    
    if (heading > 2 * PI)
        heading -= 2 * PI;
    
    return heading * (180/M_PI); // heading in degrees
}

void GetCurrentLocation()
{
    while(GPSSerial.available() > 0) 
        if(gps.encode(GPSSerial.read()))
        {
            /******** Update Long and Lat ********/
            if(gps.location.isValid())
            {
                currentLat = gps.location.lat();
                currentLong = gps.location.lng();
                break; // received new data, so move on
            } 
            /*************************************/
        }
    
}

void CalculateHeading() {
    headingError = headingTarget - headingCurrent;

    // adjust for compass wrap
    if(headingError < -180) headingError += 360;
    if(headingError >  180) headingError -= 360;
    
    // if within tolerance, dont turn
    if( abs(headingError) <= HEADING_TOLERANCE ) 
        outputError = 0;
    else if( (headingError > -60) && (headingError < 60) )
        outputError = headingError;
    else if( headingError >=  60 )
        outputError =  100;
    else if( headingError <= -60 )
        outputError = -100;
}

void MoveRobot()
{
    if(outputError == 0)
    {
        RobotMove(60, 60);             
    }
    else if(outputError < 60)
    {
        if(outputError < 0)             
            RobotMove(60 + outputError, 60);
        else                            
            RobotMove(60, 60 - outputError);
    } 
    else if (outputError ==  100 || outputError == -100)
    {
         RobotMove(50, -50);      
    }
}
