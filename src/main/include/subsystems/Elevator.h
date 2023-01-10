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

    bool RunElevatorToPos(double pos, int PIDSlot, bool &elevatorRunning);

    double positionError = 1;

    rev::CANSparkMax elevatorMotor{100, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder elevatorEncoder = elevatorMotor.GetEncoder();
    rev::SparkMaxPIDController elevatorPIDController = elevatorMotor.GetPIDController();
};