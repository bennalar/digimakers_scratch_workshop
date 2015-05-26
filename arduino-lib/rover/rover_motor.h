
#include "Encoder.h"

class RoverMotor
{
    public: RoverMotor( int dirPin, int pwmPin,
        int firstEncoderPin, int secondEncoderPin, int motorCurrentPin );

    // Checks to see if the motor has stalled
    public: bool isStalled() const { return mbIsStalled; }

    // Clears the stall flag, and lets the motor freewheel. 
    // The target RPM is set to zero
    public: void clearStall();

    // Allows the motor to freewheel (doesn't attempt to reach
    // a target RPM) and also sets the target RPM to zero.
    public: void allowMotorToFreewheel();

    // The update loop for the motor controller
    public: void update();

    // Gets the current speed we're trying to run the motors at
    public: float getTargetRPM() const { return mTargetRPM; }
    
    // Sets the speed in revolutionRPM we should try to drive the motor at. If the target RPM is non-zero
    // then it will be constrained so that its absolute value falls in the range [MIN_ABS_RPM, MAX_ABS_RPM]
    public: void setTargetRPM( float targetRPM );
    
    // Returns the encoder tick count
    public: long getLastMeasuredEncoderTicks() const { return mLastMeasuredEncoderTicks; }
    
    // Returns the speed in revolutions per minute
    public: float getLastMeasuredRPM() const { return mLastMeasuredRPM; }
    
    // Returns the motor current
    public: float getLastMeasuredCurrent() const { return mLastMeasuredCurrent; }
    
    // Gets the duty cycle we're running at
    public: float getCurDutyCycle() const { return mCurDutyCycle; }

    public: static const unsigned long MIN_TIME_DIFF_MS;
    public: static const float MIN_ABS_RPM;
    public: static const float MAX_ABS_RPM;
    public: static const float NUM_TICKS_PER_ROTATION;
    public: static const float P_GAIN;
    public: static const float I_GAIN;
    public: static const float D_GAIN;
    public: static const float STALL_CURRENT;

    // Member variables
    Encoder mEncoder;
    int mDirPin;
    int mPwmPin;
    int mMotorCurrentPin;
    bool mbIsStalled;
    bool mbIsFreewheeling;
    float mTargetRPM;
    long mLastMeasuredEncoderTicks;
    float mLastMeasuredRPM;
    float mLastMeasuredCurrent;
    float mCurDutyCycle;
    float mIntegralRPM;
    float mLastErrorRPM;
    unsigned long mLastUpdateTimeMS;
};

