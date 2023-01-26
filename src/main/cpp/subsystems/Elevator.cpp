#include "RobotData.h"
#include <subsystems/Elevator.h>


void Elevator::RobotInit(const RobotData &robotData, ElevatorData &elevatorData)
{
    elevatorMotor.RestoreFactoryDefaults();
    elevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    elevatorMotor.SetInverted(false);
    elevatorMotor.EnableVoltageCompensation(10.5);
    elevatorPIDController.SetP(0.1, 0); //need to be tuned
    elevatorPIDController.SetFeedbackDevice(elevatorAbsoluteEncoder);
    elevatorMotor.BurnFlash();

    //FIND THESE VALUES THEN GOOD
    elevatorAbsoluteEncoder.SetPositionConversionFactor(0.1);
    elevatorAbsoluteEncoder.SetZeroOffset(0.1);
    elevatorRelativeEncoder.SetPositionConversionFactor(0.1);
    
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

    if (elevatorRelativeEncoder.GetVelocity() <= 1 && runMode != RELATIVE_RUN)
    {
        ZeroRelativePosition(elevatorData);
    }

}

void Elevator::SemiAuto(const RobotData &robotData, ElevatorData &elevatorData)
{
    if (!softLimitsEnabled) 
    {
        EnableSoftLimits();
    }

    if (elevatorData.elevatorAbsoluteEncoderInitialized && runMode != ABSOLUTE_RUN)
    {
        runMode = ABSOLUTE_RUN;
        elevatorPIDController.SetFeedbackDevice(elevatorAbsoluteEncoder);
    }
    else if (elevatorForceZeroed && runMode != RELATIVE_RUN)
    {
        runMode = RELATIVE_RUN;
        elevatorPIDController.SetFeedbackDevice(elevatorRelativeEncoder);
    }

    if (runMode != NONE)
    {
        if (robotData.controlData.saElevatorSetHumanPlayerPosition)
        {
            SetElevatorPosition(humanPlayerElevatorAbsolutePos);
        }

        else if (robotData.controlData.saElevatorSetMidPosition)
        {
            SetElevatorPosition(midElevatorAbsolutePos);
        }

        else if (robotData.controlData.saElevatorSetHighPosition)
        {
            SetElevatorPosition(highElevatorAbsolutePos);
        }

        else if (robotData.controlData.saElevatorSetIntakePosition)
        {
            SetElevatorPosition(intakeElevatorAbsolutePos);
        }
    }
    else
    {
        elevatorMotor.Set(0);
    }



    if (elevatorProfileActive)
    {
        units::time::second_t elapsedTime{robotData.timerData.secSinceEnabled - elevatorProfileStartTime};
        auto setpoint = elevatorProfile.Calculate(elapsedTime);

        elevatorPIDController.SetReference(setpoint.position.value(), rev::CANSparkMax::ControlType::kPosition);

        if (elevatorProfile.IsFinished(elapsedTime))
        {
            elevatorProfileActive = false;
        }
    }


}

void Elevator::Manual(const RobotData &robotData, ElevatorData &elevatorData)
{
    if (softLimitsEnabled) 
    {
        DisableSoftLimits();
    }

    if (robotData.controlData.forceZeroElevator)
    {
        ForceZeroElevator();
    }

    if (robotData.controllerData.sLYStick > 0.08 || robotData.controllerData.sLYStick < -0.08)
    {
        elevatorMotor.Set(robotData.controllerData.sLYStick * 0.25);
    }
    else
    {
        elevatorMotor.Set(0);
    }
}

void Elevator::MoveElevator(double targetPos, const RobotData& robotData)
{
    elevatorProfileActive = true;
    elevatorProfileStartTime = robotData.timerData.secSinceEnabled;

    if (runMode == ABSOLUTE_RUN)
    {
        elevatorProfileStartPos = elevatorAbsoluteEncoder.GetPosition();
        elevatorProfileEndPos = targetPos;
    }
    else
    {
        elevatorProfileStartPos = elevatorRelativeEncoder.GetPosition();
        elevatorProfileEndPos = targetPos;
    }

    elevatorProfile = frc::TrapezoidProfile<units::degrees>
    {
        frc::TrapezoidProfile<units::degrees>::Constraints{70_deg_per_s, 15_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{elevatorProfileEndPos}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{elevatorProfileStartPos}, units::angular_velocity::degrees_per_second_t{elevatorRelativeEncoder.GetVelocity()}}
    };

}

/*
* @param elevatorAbsolutePosition desired absolute position of elevator
* @param PIDSlot PID slot to use for setting position
* @note Sets position of elevator using absoute position
*/
void Elevator::SetElevatorPosition(double elevatorAbsolutePosition)
{
    elevatorPIDController.SetReference(elevatorAbsolutePosition, rev::CANSparkMax::ControlType::kPosition, 0);
}

/*
* @note Toggles the soft limits on and off
* @note for when code switches between manual
* @note and semi automatic
*/
void Elevator::EnableSoftLimits() 
{
    elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    elevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, elevatorMinPosition - 0.1);
    elevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, elevatorMaxPosition + 0.1);
}

void Elevator::DisableSoftLimits() 
{
    elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
}

void Elevator::ForceZeroElevator()
{
    elevatorRelativeEncoder.SetPosition(0);
    elevatorForceZeroed = true;
}



void Elevator::ZeroRelativePosition(ElevatorData &elevatorData)
{
    if (IsAbsoluteEncoderInitialized(elevatorData))
    {
        elevatorRelativeEncoder.SetPosition(elevatorAbsoluteEncoder.GetPosition());
        runMode = RELATIVE_RUN;
    }
    
}

bool Elevator::IsAbsoluteEncoderInitialized(ElevatorData &elevatorData)
{
    if (elevatorAbsoluteEncoder.GetPosition() >= 0.01)
    {
        elevatorData.elevatorAbsoluteEncoderInitialized = true;
        runMode = ABSOLUTE_RUN;
    }
    else 
    {
        elevatorData.elevatorAbsoluteEncoderInitialized = false;
        runMode = NONE;
    }

    return elevatorData.elevatorAbsoluteEncoderInitialized;
}