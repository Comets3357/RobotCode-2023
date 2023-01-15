#include <subsystems/Elevator.h>

void Elevator::RobotInit(const RobotData &robotData, ElevatorData &elevatorData)
{
    elevatorMotor.RestoreFactoryDefaults();
    elevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    elevatorMotor.SetInverted(false);
    elevatorMotor.EnableVoltageCompensation(10.5);
    elevatorPIDController.SetP(0.1, 0);
    elevatorMotor.BurnFlash();
    
}

void Elevator::RobotPeriodic(const RobotData &robotData, ElevatorData &elevatorData)
{
    switch (robotData.controlData.mode) 
    {
        case MODE_TELEOP_MANUAL:
            Manual(robotData, elevatorData);
            break;
        case MODE_TELEOP_SA:
            SemiAuto(robotData, elevatorData);
            break;
        default:
            SemiAuto(robotData, elevatorData);
            break;
    }
}

void Elevator::SemiAuto(const RobotData &robotData, ElevatorData &elevatorData)
{
    if (!softLimitsToggled) 
    {
        ToggleSoftLimits();
    }

    if (robotData.controlData.saElevatorUp) 
    {
        SetElevatorPosition(elevatorRelativeMaxPosition, 0);

    }
    else if (robotData.controlData.saElevatorDown) 
    {
        SetElevatorPosition(elevatorRelativeMinPosition, 0);
    }

    // switch (robotData.controlData.elevatorSetPosition)
    // {
    //     case SET_POSITION_1:
    //     ElevatorPosition(robotData.controlData.saSetPosition1, 0, elevatorData.elevatorRunning);
    //         break;

    //     case SET_POSITION_2:
    //     ElevatorPosition(robotData.controlData.saSetPosition2, 0, elevatorData.elevatorRunning);
    //         break;

    //     case SET_POSITION_3:
    //     ElevatorPosition(robotData.controlData.saSetPosition3, 0, elevatorData.elevatorRunning);
    //     break;

    //     default:
    //         break;

    // }

}

void Elevator::Manual(const RobotData &robotData, ElevatorData &elevatorData)
{
    if (softLimitsToggled) 
    {
        ToggleSoftLimits();
    }
}

/*
* @param elevatorAbsolutePosition desired absolute position of elevator
* @param PIDSlot PID slot to use for setting position
* @note Sets position of elevator using absoute position
*/
void Elevator::SetElevatorPosition(double elevatorAbsolutePosition, int PIDSlot)
{
    elevatorPIDController.SetReference(elevatorAbsolutePosition, rev::CANSparkMax::ControlType::kDutyCycle, PIDSlot);

}

/*
* @note Toggles the soft limits on and off
* @note for when code switches between manual
* @note and semi automatic
*/
void Elevator::ToggleSoftLimits() 
{
    if (softLimitsToggled)
    {
        elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
        elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
    }
    else if (!softLimitsToggled) 
    {
        elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

        elevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, elevatorRelativeMinPosition - 0.1);
        elevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, elevatorRelativeMaxPosition + 0.1);
    }
}

/*
* @param currentAbsolutePosition Pass in the current absolute position of Elevator
* @note Converts the absolute position to relative position
*/
double Elevator::AbsoluteToRelative(double currentAbsolutePosition) 
{
    double slope = (elevatorRelativeMaxPosition - elevatorRelativeMinPosition) / (elevatorAbsoluteMaxPosition - elevatorAbosluteMinPosition);
    double b = elevatorRelativeMinPosition - (slope * elevatorAbosluteMinPosition);
    return ((slope * currentAbsolutePosition) + b);
}

void Elevator::ZeroRelativePosition()
{
    bullbarSliderRelativeEncoder.SetPosition(AbsoluteToRelative(bullbarSliderAbsoluteEncoder.GetPosition()));
}