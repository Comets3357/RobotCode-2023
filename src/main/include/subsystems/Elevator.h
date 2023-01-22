#pragma once


#include <rev/CANSparkMax.h>

struct ElevatorData
{
    bool elevatorRunning = false;
    bool elevatorAbsoluteEncoderInitialized = false;
};

enum ElevatorRunMode
{
    ABSOLUTE_RUN,
    RELATIVE_RUN,
    NONE
};

class Elevator
{
public:

    void RobotInit(const RobotData &robotData, ElevatorData &elevatorData);
    void RobotPeriodic(const RobotData &robotData, ElevatorData &elevatorData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, ElevatorData elevatorData);
    void UpdateData(const RobotData &robotData, ElevatorData elevatorData);

private:

    ElevatorRunMode runMode = ABSOLUTE_RUN;

    void SetElevatorPosition(double elevatorAbsolutePosition);
    void SemiAuto(const RobotData &robotData, ElevatorData &ElevatorData);
    void Manual(const RobotData &robotData, ElevatorData &ElevatorData);
    //double AbsoluteToRelative(double currentAbsolutePosition);
    void DisableSoftLimits();
    void EnableSoftLimits();
    void ZeroRelativePosition(ElevatorData &elevatorData);
    void ForceZeroElevator();

    bool IsAbsoluteEncoderInitialized(ElevatorData &bullbarData);

    bool softLimitsEnabled = false;
    bool forceZeroed = false;

    bool absoluteEncoderFeedBackDevice = true;

    rev::CANSparkMax elevatorMotor{100, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder elevatorRelativeEncoder = elevatorMotor.GetEncoder();
    rev::SparkMaxPIDController elevatorPIDController = elevatorMotor.GetPIDController();
    rev::SparkMaxAbsoluteEncoder elevatorAbsoluteEncoder = elevatorMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    // Encoder Min and Max Values 
    double elevatorMaxPosition = elevatorMinPosition+31.375; // TODO: fix this value when we get subsystem
    double elevatorMinPosition = 0; // TODO: fix this value when we get subsystem

    double elevatorUpwardSpeed = 0.4;
    double elevatorDownwardSpeed = -0.4;






    //elevator heights
    double intakeElevatorAbsolutePos = 0.0;
    double midElevatorAbsolutePos = 0.0;
    double highElevatorAbsolutePos = 0.0;
    double humanPlayerElevatorAbsolutePos = 0.0;

    
};





