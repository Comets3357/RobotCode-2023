#pragma once

#include "RobotData.h"
#include <rev/CANSparkMax.h>

class Elevator
{
public:

    struct ElevatorData
    {
        bool elevatorRunning = false;
    };

    void RobotInit(const RobotData &robotData, ElevatorData &elevatorData);
    void RobotPeriodic(const RobotData &robotData, ElevatorData &elevatorData);

private:

    void ElevatorPosition(double elevatorPosition, int PIDSlot, bool &elevatorRunning);
    void SemiAuto(const RobotData &robotData, ElevatorData &ElevatorData);
    void Manual(const RobotData &robotData, ElevatorData &ElevatorData);
    void AbsoluteToRelative(double currentAbsolutePosition);
    void ToggleSoftLimits();

    double positionError = 1;

    rev::CANSparkMax elevatorMotor{100, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder elevatorEncoder = elevatorMotor.GetEncoder();
    rev::SparkMaxPIDController elevatorPIDController = elevatorMotor.GetPIDController();

        // Encoder Min and Max Values 
    double elevatorRelativeMaxPosition = 13; // TODO: fix this value when we get subsystem
    double elevatorRelativeMinPosition = 0; // TODO: fix this value when we get subsystem

    double elevatorAbsoluteMaxPosition = 0.93418697534; // TODO: fix this value when we get subsystem
    double elevatorAbosluteMinPosition = 0.14207; // TODO: fix this value when we get subsystem

    double elevatorUpwardSpeed = 0.4;
    double elevatorDownwardSpeed = -0.4;

    bool softLimitsToggled = false;
};