#include "RobotData.h"



void Elevator::RobotInit(const RobotData &robotData, ElevatorData &elevatorData)
{
    elevatorMotor.RestoreFactoryDefaults();
    elevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    elevatorMotor.SetInverted(true);
    elevatorMotor.EnableVoltageCompensation(10.5);
    elevatorPIDController.SetP(0.3, 0); 
    //elevatorPIDController.SetFeedbackDevice(elevatorAbsoluteEncoder);
    elevatorMotor.BurnFlash();
    elevatorPIDController.SetFeedbackDevice(elevatorRelativeEncoder);
    elevatorPIDController.SetOutputRange(-0.5,0.5);

    //FIND THESE VALUES THEN GOOD
    // elevatorAbsoluteEncoder.SetPositionConversionFactor(0.1);
    // elevatorAbsoluteEncoder.SetZeroOffset(0.1);
    // elevatorRelativeEncoder.SetPositionConversionFactor(0.1);
    
}

void Elevator::RobotPeriodic(const RobotData &robotData, ElevatorData &elevatorData)
{
    switch (robotData.controlData.mode) 
    {
        case MODE_TELEOP_MANUAL:
            Manual(robotData, elevatorData);
            break;
        case MODE_TELEOP_SA:
            Manual(robotData, elevatorData);
            break;
        default:
            SemiAuto(robotData, elevatorData);
            break;
    }

    if (elevatorRelativeEncoder.GetVelocity() <= 1 && runMode != RELATIVE_RUN)
    {
        ZeroRelativePosition(elevatorData);
    }

    frc::SmartDashboard::PutNumber("elevator relative pos", elevatorRelativeEncoder.GetPosition());
    if (robotData.controllerData.sXBtn)
    {
        MoveElevator(20, robotData);
    }

    if (elevatorProfileActive)
    {
        units::time::second_t elapsedTime{robotData.timerData.secSinceEnabled - elevatorProfileStartTime};
        auto setpoint = elevatorProfile.Calculate(elapsedTime);

        elevatorPIDController.SetReference(setpoint.position.value(), rev::CANSparkMax::ControlType::kPosition);
        frc::SmartDashboard::PutNumber("elevatorPos TRAP", setpoint.position.value());
        if (elevatorProfile.IsFinished(elapsedTime))
        {
            elevatorProfileActive = false;
        }
    }

}

void Elevator::SemiAuto(const RobotData &robotData, ElevatorData &elevatorData)
{
    if (!softLimitsEnabled) 
    {
        EnableSoftLimits();
    }

    // if (elevatorData.elevatorAbsoluteEncoderInitialized && runMode != ABSOLUTE_RUN)
    // {
    //     runMode = RELATIVE_RUN;
    //     elevatorPIDController.SetFeedbackDevice(elevatorAbsoluteEncoder);
    // }
    else if (elevatorForceZeroed && runMode != RELATIVE_RUN)
    {
        runMode = RELATIVE_RUN;
        elevatorPIDController.SetFeedbackDevice(elevatorRelativeEncoder);
    }
    runMode = RELATIVE_RUN;
    elevatorPIDController.SetFeedbackDevice(elevatorRelativeEncoder);

    if (runMode != NONE)
    {
        if (robotData.controlData.saElevatorSetHumanPlayerPosition)
        {
            MoveElevator(humanPlayerElevatorAbsolutePos, robotData);
        }

        else if (robotData.controlData.saElevatorSetMidPosition)
        {
            MoveElevator(midElevatorAbsolutePos, robotData);
        }

        else if (robotData.controlData.saElevatorSetHighPosition)
        {
            MoveElevator(highElevatorAbsolutePos, robotData);
        }

        else if (robotData.controlData.saElevatorSetIntakePosition)
        {
            MoveElevator(intakeElevatorAbsolutePos, robotData);
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
        frc::SmartDashboard::PutNumber("elevatorPos TRAP", setpoint.position.value());
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
    EnableSoftLimits();
    if (robotData.controlData.forceZeroElevator)
    {
        ForceZeroElevator();
    }

    // if (robotData.controllerData.sLYStick > 0.08 || robotData.controllerData.sLYStick < -0.08)
    // {
    //     elevatorMotor.Set(robotData.controllerData.sLYStick * 0.2);
    // }
    // else
    // {
    //     elevatorMotor.Set(0);
    // }
}

void Elevator::MoveElevator(double targetPos, const RobotData& robotData)
{
    elevatorProfileActive = true;
    elevatorProfileStartTime = robotData.timerData.secSinceEnabled;

    if (runMode == ABSOLUTE_RUN)
    {
        elevatorProfileStartPos = elevatorRelativeEncoder.GetPosition();
        elevatorProfileEndPos = targetPos;
    }
    else
    {
        elevatorProfileStartPos = elevatorRelativeEncoder.GetPosition();
        elevatorProfileEndPos = targetPos;
    }

    elevatorProfile = frc::TrapezoidProfile<units::degrees>
    {
        frc::TrapezoidProfile<units::degrees>::Constraints{40_deg_per_s, 30_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{elevatorProfileEndPos}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{elevatorProfileStartPos}, units::angular_velocity::degrees_per_second_t{elevatorRelativeEncoder.GetVelocity()}}
    };

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

    elevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, elevatorMinPosition + 2);
    elevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, elevatorMaxPosition - 2);
}

void Elevator::DisableSoftLimits() 
{
    elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
}

void Elevator::ForceZeroElevator()
{
    elevatorRelativeEncoder.SetPosition(10);
    elevatorForceZeroed = true;
}



void Elevator::ZeroRelativePosition(ElevatorData &elevatorData)
{
    if (IsAbsoluteEncoderInitialized(elevatorData))
    {
        elevatorRelativeEncoder.SetPosition(elevatorAbsoluteEncoder.GetPosition());
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
        runMode = NONE;
    }

    return elevatorData.elevatorAbsoluteEncoderInitialized;
}