// Copyright (c) 2013 Dawn Robotics Ltd - Alan Broun <abroun@dawnrobotics.co.uk>
// Adapted by Phil Bennett 2015 for the Rover kit

#include <stdint.h>
#include <Servo.h>
#include <utility_direct_pin_read.h>
#include <ultrasonic_sensor.h>
#include <Encoder.h>
#include <utility_interrupt_pins.h>
#include <rover_ir_sensors.h>
#include <rover_motor.h>


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
const int LEFT_CURRENT_PIN = A6;

const int RIGHT_DIR_PIN = 7;
const int RIGHT_PWM_PIN = 6;
const int RIGHT_ENCODER_FIRST_PIN = 2;
const int RIGHT_ENCODER_SECOND_PIN = 4;
const int RIGHT_CURRENT_PIN = A7;

const int PAN_SERVO_PIN = 9;
const int PAN_TILT_PIN = 10;
const int ULTRASONIC_PIN = 8;

const int NUM_IR_SENSORS = 4;
const int IR_LED_PINS[ NUM_IR_SENSORS ] = { A0, A0, A1, A1 };   
const int IR_SENSOR_PINS[ NUM_IR_SENSORS ] = { A3, A2, A4, A5 };

const int CLOSE_RANGE_IR_VALUE = 150;

const float ABS_MOVE_RPM = 40.0f;
const float ABS_TURN_RPM = 40.0f;

const float NO_RANGE_RECORDED = -999.0f; /* indicates that we haven't taken a range */

const int LOOK_FORWARD_PAN_ANGLE = 77; //0 = 90 degrees right
const int LOOK_FORWARD_TILT_ANGLE = 120;

Servo gPanServo;
Servo gTiltServo;
int gPanAngle = LOOK_FORWARD_PAN_ANGLE;
int gTiltAngle = LOOK_FORWARD_TILT_ANGLE;

//------------------------------------------------------------------------------
/* Initialise C++ classes */

RoverMotor gLeftMotor( LEFT_DIR_PIN, LEFT_PWM_PIN,
    LEFT_ENCODER_FIRST_PIN, LEFT_ENCODER_SECOND_PIN, LEFT_CURRENT_PIN);
RoverMotor gRightMotor( RIGHT_DIR_PIN, RIGHT_PWM_PIN,
    RIGHT_ENCODER_FIRST_PIN, RIGHT_ENCODER_SECOND_PIN, RIGHT_CURRENT_PIN );
    
UltrasonicSensor gUltrasonicSensor( ULTRASONIC_PIN ) ;
RoverIRSensors gRoverIRSensors( 
    IR_LED_PINS[ 0 ], IR_LED_PINS[ 1 ],
    IR_LED_PINS[ 2 ], IR_LED_PINS[ 3 ],
    IR_SENSOR_PINS[ 0 ], IR_SENSOR_PINS[ 1 ],
    IR_SENSOR_PINS[ 2 ], IR_SENSOR_PINS[ 3 ] );

//------------------------------------------------------------------------------
/* Function orward declarations */
void blinkOnboardLed();
void stop();

//------------------------------------------------------------------------------
void setup()
{    
    gPanServo.attach( PAN_SERVO_PIN );
    gTiltServo.attach( PAN_TILT_PIN );  
    
    // Point the ultrasonic sensor forward
    gPanAngle = LOOK_FORWARD_PAN_ANGLE;
    gTiltAngle = LOOK_FORWARD_TILT_ANGLE;

    gPanServo.write( gPanAngle );
    gTiltServo.write( gTiltAngle );
    delay( 1000 );
    
    Serial.begin( 9600 );
}

//------------------------------------------------------------------------------
void loop()
{
    // Read from the robot's sensors
    float ultrasonicRange = gUltrasonicSensor.measureRange();
    gRoverIRSensors.takeReadings();
    int frontLeftIR = gRoverIRSensors.lastFrontLeftReading();
    int frontRightIR = gRoverIRSensors.lastFrontRightReading();
    int rearLeftIR = gRoverIRSensors.lastRearLeftReading();
    int rearRightIR = gRoverIRSensors.lastRearRightReading();
    
    gLeftMotor.update();
    gRightMotor.update();
    
    
    // Check to see if we've hit an obstacle we didn't see
      if ( gLeftMotor.isStalled()
          || gRightMotor.isStalled() )
      {
        //TODO: Send an error back
          stop();
          
          gLeftMotor.clearStall();
          gRightMotor.clearStall();
      }
    
    else if ( Serial.available() )
    {
        //Blink twice to indicate a serial read
        blinkOnboardLed(100);
        blinkOnboardLed(100);
        char command = Serial.read();

        switch ( command )
        {
          /* 
          A safety mechanism to prevent the robot from driving into walls/obstacles
          - Unable to go forward if any front IR sensors detect an obstacle 
          - Unable to go backwards if any rear IR sensors detect an obstacle
          */
        case 'f':
            {
              if ( CLOSE_RANGE_IR_VALUE > frontLeftIR && 
                   CLOSE_RANGE_IR_VALUE > frontRightIR )
                 {  
                  gLeftMotor.setTargetRPM( ABS_MOVE_RPM );
                  gRightMotor.setTargetRPM( ABS_MOVE_RPM );
                 }
                 else
                 {
                   //TODO: Send an error back
                   stop();
                 }
                 break;
            }
        case 'b':
            {
              if ( CLOSE_RANGE_IR_VALUE > rearLeftIR && 
                   CLOSE_RANGE_IR_VALUE > rearRightIR )
                 {  
                  gLeftMotor.setTargetRPM( -ABS_MOVE_RPM );
                  gRightMotor.setTargetRPM( -ABS_MOVE_RPM );
                 }
                 else
                 {
                   //TODO: Send an error back
                   stop();
                 }
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
                stop(); 
                break;
            }
        }
    }
    
    // Output robot status here
    Serial.print( gLeftMotor.getLastMeasuredEncoderTicks() );
    Serial.print( " " );
    Serial.print( gRightMotor.getLastMeasuredEncoderTicks() );
    Serial.print( " " );
    /* Return range in CM */
    Serial.print( ultrasonicRange * 100 );
    Serial.print( " " );
    Serial.print( frontLeftIR );
    Serial.print( " " );
    Serial.print( frontRightIR );
    Serial.print( " " );
    Serial.print( rearLeftIR );
    Serial.print( " " );
    Serial.print( rearRightIR );
    Serial.println( "" );
    
}

void blinkOnboardLed(int numMilliSecs)
{
    digitalWrite(ONBOARD_LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(numMilliSecs);               // wait for numMilliSecs
    digitalWrite(ONBOARD_LED_PIN, LOW);    // turn the LED off by making the voltage LOW
}

void stop()
{
  gLeftMotor.setTargetRPM( 0.0 );
  gRightMotor.setTargetRPM( 0.0 ); 
}
