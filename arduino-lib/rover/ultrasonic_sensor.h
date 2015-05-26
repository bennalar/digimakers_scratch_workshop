
class UltrasonicSensor
{
    public: UltrasonicSensor( int sensorPin );

    public: float measureRange() const;
    public: float getLastMeasuredRange() const;

    public: const static float MAX_RANGE;

    // Member variables
    private: int mSensorPin;
    private: mutable float mLastMeasuredRange;
};

