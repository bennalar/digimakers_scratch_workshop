
class RoverIRSensors
{
    public: RoverIRSensors( 
        int frontLeftLedPin, int frontRightLedPin,
        int rearLeftLedPin, int rearRightLedPin,
        int frontLeftSensorPin, int frontRightSensorPin,
        int rearLeftSensorPin, int rearRightSensorPin );

    public: void takeReadings();
    public: int lastFrontLeftReading() const { return mLastFrontLeftReading; };
    public: int lastFrontRightReading() const { return mLastFrontRightReading; };
    public: int lastRearLeftReading() const { return mLastRearLeftReading; };
    public: int lastRearRightReading() const { return mLastRearRightReading; };

    private: int readIRSensor( int ledPin, int sensorPin ) const;

    // Member variables
    private: int mFrontLeftLedPin;
    private: int mFrontLeftSensorPin;
    private: int mLastFrontLeftReading;
    private: int mFrontRightLedPin;
    private: int mFrontRightSensorPin;
    private: int mLastFrontRightReading;
    private: int mRearLeftLedPin;
    private: int mRearLeftSensorPin;
    private: int mLastRearLeftReading;
    private: int mRearRightLedPin;
    private: int mRearRightSensorPin;
    private: int mLastRearRightReading;
};


