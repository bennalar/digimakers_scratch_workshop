// Copyright (c) 2013 Dawn Robotics Ltd - Alan Broun <abroun@dawnrobotics.co.uk>
// Adapted by Phil Bennett 2015 for the Rover kit

#include <stdint.h>
#include "rover_motor.h"


/*
//Original values
const int LEFT_DIR_PIN = 7;
const int LEFT_PWM_PIN = 9;
const int LEFT_ENCODER_FIRST_PIN = 3;
const int LEFT_ENCODER_SECOND_PIN = 5;

const int RIGHT_DIR_PIN = 8;
const int RIGHT_PWM_PIN = 10;
const int RIGHT_ENCODER_FIRST_PIN = 2;
const int RIGHT_ENCODER_SECOND_PIN = 4;

//const int ULTRASONIC_PIN = 12;
*/

//Values from the manual
const int ONBOARD_LED_PIN = 13;

const int LEFT_DIR_PIN = 12;
const int LEFT_PWM_PIN = 11;
const int LEFT_ENCODER_FIRST_PIN = 3;
const int LEFT_ENCODER_SECOND_PIN = 5;

const int RIGHT_DIR_PIN = 7;
const int RIGHT_PWM_PIN = 6;
const int RIGHT_ENCODER_FIRST_PIN = 2;
const int RIGHT_ENCODER_SECOND_PIN = 4;

//const int ULTRASONIC_PIN = 12;

const float ABS_MOVE_RPM = 40.0f;
const float ABS_TURN_RPM = 40.0f;

//FIXME: ?? Other motor driver has additional field for current pin
RoverMotor gLeftMotor( LEFT_DIR_PIN, LEFT_PWM_PIN,
    LEFT_ENCODER_FIRST_PIN, LEFT_ENCODER_SECOND_PIN );
RoverMotor gRightMotor( RIGHT_DIR_PIN, RIGHT_PWM_PIN,
    RIGHT_ENCODER_FIRST_PIN, RIGHT_ENCODER_SECOND_PIN );
//------------------------------------------------------------------------------
/* Function orward declarations */
void blinkOnboardLed();

//------------------------------------------------------------------------------
void setup()
{    
    Serial.begin( 9600 );
}

//------------------------------------------------------------------------------
void loop()
{
    gLeftMotor.update();
    gRightMotor.update();
    
    while ( Serial.available() )
    {
        //Blink twice to indicate a serial read
        blinkOnboardLed(100);
        blinkOnboardLed(100);
        char command = Serial.read();
        switch ( command )
        {
        case 'f':
            {
                gLeftMotor.setTargetRPM( ABS_MOVE_RPM );
                gRightMotor.setTargetRPM( ABS_MOVE_RPM );
                break;
            }
        case 'b':
            {
                gLeftMotor.setTargetRPM( -ABS_MOVE_RPM );
                gRightMotor.setTargetRPM( -ABS_MOVE_RPM );
                break;
            }
        case 'l':
            {
                gLeftMotor.setTargetRPM( -ABS_TURN_RPM );
                gRightMotor.setTargetRPM( ABS_TURN_RPM );
                break;
            }
        case 'r':
            {
                gLeftMotor.setTargetRPM( ABS_TURN_RPM );
                gRightMotor.setTargetRPM( -ABS_TURN_RPM );
                break;
            }
        case '\n':
            {
                // Ignore
                break;
            }
        case 's':
        default:
            {
                gLeftMotor.setTargetRPM( 0.0 );
                gRightMotor.setTargetRPM( 0.0 );
                break;
            }
        }
    }
    
    // Use the ultrasonic sensor to look for obstacles
    /*pinMode( ULTRASONIC_PIN, OUTPUT );
    digitalWrite( ULTRASONIC_PIN, LOW );
    delayMicroseconds( 2 );
    digitalWrite( ULTRASONIC_PIN, HIGH );
    delayMicroseconds( 5 );
    digitalWrite( ULTRASONIC_PIN, LOW );
    pinMode( ULTRASONIC_PIN, INPUT );
    long duration = pulseIn( ULTRASONIC_PIN, HIGH );
    long distanceCM = duration/29/2;*/	
    
    // Output robot status here
    Serial.print( gLeftMotor.getLastMeasuredEncoderTicks() );
    Serial.print( " " );
    Serial.println( gRightMotor.getLastMeasuredEncoderTicks() );
    //Serial.print( " " );
    //Serial.println( distanceCM );
}

void blinkOnboardLed(int numMilliSecs)
{
    digitalWrite(ONBOARD_LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(numMilliSecs);               // wait for numMilliSecs
    digitalWrite(ONBOARD_LED_PIN, LOW);    // turn the LED off by making the voltage LOW
}

