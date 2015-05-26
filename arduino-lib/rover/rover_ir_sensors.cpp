#include "Arduino.h"
#include "rover_ir_sensors.h"

//------------------------------------------------------------------------------
RoverIRSensors::RoverIRSensors( 
    int frontLeftLedPin, int frontRightLedPin,
    int rearLeftLedPin, int rearRightLedPin,
    int frontLeftSensorPin, int frontRightSensorPin,
    int rearLeftSensorPin, int rearRightSensorPin )

    : mFrontLeftLedPin( frontLeftLedPin ),
    mFrontLeftSensorPin( frontLeftSensorPin ),
    mLastFrontLeftReading( 0 ),
    mFrontRightLedPin( frontRightLedPin ),
    mFrontRightSensorPin( frontRightSensorPin ),
    mLastFrontRightReading( 0 ),
    mRearLeftLedPin( rearLeftLedPin ),
    mRearLeftSensorPin( rearLeftSensorPin ),
    mLastRearLeftReading( 0 ),
    mRearRightLedPin( rearRightLedPin ),
    mRearRightSensorPin( rearRightSensorPin ),
    mLastRearRightReading( 0 )
{
    pinMode( mFrontLeftLedPin, OUTPUT );
    pinMode( mFrontRightLedPin, OUTPUT );
    pinMode( mRearLeftLedPin, OUTPUT );
    pinMode( mRearRightLedPin, OUTPUT );
}

//------------------------------------------------------------------------------
void RoverIRSensors::takeReadings()
{
    mLastFrontLeftReading = readIRSensor( mFrontLeftLedPin, mFrontLeftSensorPin );
    mLastFrontRightReading = readIRSensor( mFrontRightLedPin, mFrontRightSensorPin );
    mLastRearLeftReading = readIRSensor( mRearLeftLedPin, mRearLeftSensorPin );
    mLastRearRightReading = readIRSensor( mRearRightLedPin, mRearRightSensorPin );
}

//------------------------------------------------------------------------------
int RoverIRSensors::readIRSensor( int ledPin, int sensorPin ) const
{
    const int READ_DELAY_MS = 1;
    int result = 0;

    digitalWrite( ledPin, HIGH );
    delay( READ_DELAY_MS );
    result = analogRead( sensorPin );    // Get total IR
    digitalWrite( ledPin, LOW );
    delay( READ_DELAY_MS );
    result -= analogRead( sensorPin );

    return result;
}

