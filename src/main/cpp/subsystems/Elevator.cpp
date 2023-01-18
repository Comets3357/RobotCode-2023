#include "RobotData.h"
#include <subsystems/Elevator.h>


void Elevator::RobotInit(const RobotData &robotData, ElevatorData &elevatorData)
{
    elevatorMotor.RestoreFactoryDefaults();
    elevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    elevatorMotor.SetInverted(false);
    elevatorMotor.EnableVoltageCompensation(10.5);
    elevatorPIDController.SetP(0.1, 0); //need to be tuned
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

    if (elevatorRelativeEncoder.GetVelocity() <= 1)
    {
        ZeroRelativePosition(elevatorData);
    }

}

void Elevator::SemiAuto(const RobotData &robotData, ElevatorData &elevatorData)
{
    if (!softLimitsToggled) 
    {
        ToggleSoftLimits();
    }

    if (elevatorData.elevatorAbsoluteEncoderInitialized)
    {
        if (robotData.controlData.saElevatorSetHumanPlayerPosition)
        {
            SetElevatorAbsolutePosition(humanPlayerElevatorAbsolutePos, 0);
        }

        else if (robotData.controlData.saElevatorSetMidPosition)
        {
            SetElevatorAbsolutePosition(midElevatorAbsolutePos, 0);
        }

        else if (robotData.controlData.saElevatorSetHighPosition)
        {
            SetElevatorAbsolutePosition(highElevatorAbsolutePos, 0);
        }

        else if (robotData.controlData.saElevatorSetIntakePosition)
        {
            SetElevatorAbsolutePosition(intakeElevatorAbsolutePos, 0);
        }
    }
    else
    {
        if (robotData.controlData.saElevatorSetHumanPlayerPosition)
        {
            SetElevatorRelativePosition(AbsoluteToRelative(humanPlayerElevatorAbsolutePos), 0);
        }

        else if (robotData.controlData.saElevatorSetMidPosition)
        {
            SetElevatorRelativePosition(AbsoluteToRelative(midElevatorAbsolutePos), 0);
        }

        else if (robotData.controlData.saElevatorSetHighPosition)
        {
            SetElevatorRelativePosition(AbsoluteToRelative(highElevatorAbsolutePos), 0);
        }

        else if (robotData.controlData.saElevatorSetIntakePosition)
        {
            SetElevatorRelativePosition(AbsoluteToRelative(intakeElevatorAbsolutePos), 0);
        }
    }

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
void Elevator::SetElevatorAbsolutePosition(double elevatorAbsolutePosition, int PIDSlot)
{
    elevatorPIDController.SetReference(elevatorAbsolutePosition, rev::CANSparkMax::ControlType::kDutyCycle, PIDSlot);
}

void Elevator::SetElevatorRelativePosition(double elevatorAbsolutePosition, int PIDSlot)
{
    elevatorPIDController.SetReference(elevatorAbsolutePosition, rev::CANSparkMax::ControlType::kPosition, PIDSlot);
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

void Elevator::ZeroRelativePosition(ElevatorData &elevatorData)
{
    if (IsAbsoluteEncoderInitialized(elevatorData))
    {
        elevatorRelativeEncoder.SetPosition(AbsoluteToRelative(elevatorAbsoluteEncoder.GetPosition()));
    }
    
}

bool Elevator::IsAbsoluteEncoderInitialized(ElevatorData &elevatorData)
{
    if (elevatorAbsoluteEncoder.GetPosition() >= 0.01)
    {
        elevatorData.elevatorAbsoluteEncoderInitialized = true;
    }
    else 
    {
        elevatorData.elevatorAbsoluteEncoderInitialized = false;
    }

    return elevatorData.elevatorAbsoluteEncoderInitialized;
}