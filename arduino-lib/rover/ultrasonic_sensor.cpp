#include "Arduino.h"
#include "ultrasonic_sensor.h"

//------------------------------------------------------------------------------
const float UltrasonicSensor::MAX_RANGE = 4.0;

//------------------------------------------------------------------------------
UltrasonicSensor::UltrasonicSensor( int sensorPin )
    : mSensorPin( sensorPin ),
    mLastMeasuredRange( MAX_RANGE )
{
}

//------------------------------------------------------------------------------
float UltrasonicSensor::measureRange() const
{
    // Send out request pulse
    pinMode( mSensorPin, OUTPUT );
    digitalWrite( mSensorPin, LOW );
    delayMicroseconds( 2 );
    digitalWrite( mSensorPin, HIGH );
    delayMicroseconds( 5 );
    digitalWrite( mSensorPin, LOW );

    // Read in the response pulse
    pinMode( mSensorPin, INPUT );
    long durationUS = pulseIn( mSensorPin, HIGH );

    // Convert from US to metres
    long distanceCM = durationUS/29/2;
    mLastMeasuredRange = (float)distanceCM / 100.0;

    return mLastMeasuredRange;
}



