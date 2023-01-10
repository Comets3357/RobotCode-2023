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

bool Elevator::RunElevatorToPos(double pos, int PIDSlot, bool &elevatorRunning)
{
    elevatorPIDController.SetReference(pos, rev::ControlType::kPosition, PIDSlot);
    if (elevatorEncoder.GetPosition() > pos - positionError && elevatorEncoder.GetPosition() < pos + positionError)
    {
        elevatorRunning = true;
        return true;
    }
    else
    {
        elevatorRunning = false;
        return false;
    }
}