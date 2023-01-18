#pragma once


#include <rev/CANSparkMax.h>

struct ElevatorData
{
    bool elevatorRunning = false;
    bool elevatorAbsoluteEncoderInitialized = false;
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

    void SetElevatorAbsolutePosition(double elevatorAbsolutePosition, int PIDSlot);
    void SetElevatorRelativePosition(double elevatorAbsolutePosition, int PIDSlot);
    void SemiAuto(const RobotData &robotData, ElevatorData &ElevatorData);
    void Manual(const RobotData &robotData, ElevatorData &ElevatorData);
    double AbsoluteToRelative(double currentAbsolutePosition);
    void ToggleSoftLimits();
    void ZeroRelativePosition(ElevatorData &elevatorData);
    void ForceZeroElevator();

    bool IsAbsoluteEncoderInitialized(ElevatorData &bullbarData);

    bool softLimitsToggled = false;

    rev::CANSparkMax elevatorMotor{100, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder elevatorRelativeEncoder = elevatorMotor.GetEncoder();
    rev::SparkMaxPIDController elevatorPIDController = elevatorMotor.GetPIDController();
    rev::SparkMaxAbsoluteEncoder elevatorAbsoluteEncoder = elevatorMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    // Encoder Min and Max Values 
    double elevatorRelativeMaxPosition = 13; // TODO: fix this value when we get subsystem
    double elevatorRelativeMinPosition = 0; // TODO: fix this value when we get subsystem
    double elevatorAbsoluteMaxPosition = 0.93418697534; // TODO: fix this value when we get subsystem
    double elevatorAbosluteMinPosition = 0.14207; // TODO: fix this value when we get subsystem

    double elevatorUpwardSpeed = 0.4;
    double elevatorDownwardSpeed = -0.4;






    //elevator heights
    double intakeElevatorAbsolutePos = 0.0;
    double midElevatorAbsolutePos = 0.0;
    double highElevatorAbsolutePos = 0.0;
    double humanPlayerElevatorAbsolutePos = 0.0;

    
};





