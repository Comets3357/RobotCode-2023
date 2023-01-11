#include <subsystems/Elevator.h>




void Elevator::RobotInit(const RobotData &robotData, ElevatorData &elevatorData)
{
    elevatorMotor.RestoreFactoryDefaults();
    elevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    elevatorMotor.SetInverted(false);
    elevatorPIDController.SetP(0.1, 0);
    elevatorMotor.BurnFlash();
    
}

void Elevator::RobotPeriodic(const RobotData &robotData, ElevatorData &elevatorData)
{
    
}

void Elevator::SemiAuto(const RobotData &robotData, ElevatorData &elevatorData)
{
    if (!softLimitsToggled) 
    {
        ToggleSoftLimits();
    }

    if (robotData.controlData.saElevatorUp) 
    {
        ElevatorPosition(elevatorUpwardSpeed, 0, elevatorData.elevatorRunning);
        ElevatorPosition(elevatorRelativeMaxPosition, 0, elevatorData.elevatorRunning);

    }
    else if (robotData.controlData.saElevatorDown) 
    {
        ElevatorPosition(elevatorDownwardSpeed, 0, elevatorData.elevatorRunning);
        ElevatorPosition(elevatorRelativeMinPosition, 0, elevatorData.elevatorRunning);
    }
    switch (robotData.controlData.elevatorSetPosition)
    {
        case SET_POSITION_1:
        ElevatorPosition(robotData.controlData.saSetPosition1, 0, elevatorData.elevatorRunning);
        break;

        case SET_POSITION_2:
        ElevatorPosition(robotData.controlData.saSetPosition2, 0, elevatorData.elevatorRunning);
        break;

        case SET_POSITION_3:
        ElevatorPosition(robotData.controlData.saSetPosition3, 0, elevatorData.elevatorRunning);
        break;

        default:
        break;

    }
}

void Elevator::Manual(const RobotData &robotData, ElevatorData &elevatorData)
{
    if (softLimitsToggled) 
    {
        ToggleSoftLimits();
    }

    if (robotData.controlData.mElevatorDown)
    {
        elevatorMotor.Set(-0.3);
    }
    else if (robotData.controlData.mElevatorUp)
    {
        elevatorMotor.Set(0.3);
    }
}

void Elevator::ElevatorPosition(double elevatorPosition, int PIDSlot, bool &elevatorRunning)
{
    elevatorPIDController.SetReference(elevatorPosition, rev::CANSparkMax::ControlType::kPosition, PIDSlot);
    if (elevatorEncoder.GetVelocity() > 0.1)
    {
        elevatorRunning = true;
    }
    else
    {
        elevatorRunning = false;
    }
}

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