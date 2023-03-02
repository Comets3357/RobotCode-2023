#include "RobotData.h"

void Elevator::RobotInit(const RobotData &robotData, ElevatorData &elevatorData)
{
    elevatorMotor.RestoreFactoryDefaults();
    elevatorAbsoluteEncoder.SetInverted(robotData.configData.elevatorConfigData.invertAbosolute);
    elevatorAbsoluteEncoder.SetPositionConversionFactor(robotData.configData.elevatorConfigData.absoluteConversionFactor);
    elevatorAbsoluteEncoder.SetZeroOffset(robotData.configData.elevatorConfigData.absoluteZeroOffset);
    
    elevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    elevatorMotor.SetInverted(robotData.configData.elevatorConfigData.invertRelative);
    elevatorMotor.EnableVoltageCompensation(robotData.configData.elevatorConfigData.voltageComp);
    elevatorMotor.SetSmartCurrentLimit(robotData.configData.elevatorConfigData.currentLimit);

    extendedLimitSwitch.EnableLimitSwitch(false);
    retractedLimitSwitch.EnableLimitSwitch(false);

    elevatorPIDController.SetP(robotData.configData.elevatorConfigData.pValue, 0); 
    elevatorRelativeEncoder.SetPositionConversionFactor(robotData.configData.elevatorConfigData.relativeConversionFactor);
    elevatorRelativeEncoder.SetPosition(10);

    elevatorMotor.BurnFlash();

    // elevatorPIDController.SetFeedbackDevice(elevatorAbsoluteEncoder);

    elevatorPIDController.SetOutputRange(-1,1);

    // elevatorRelativeEncoder.SetPositionConversionFactor(0.51);
    // elevatorRelativeEncoder.SetPosition(10);



    //FIND THESE VALUES THEN GOOD
    
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
            SemiAuto(robotData, elevatorData);
            break;
        default:
            SemiAuto(robotData, elevatorData);
            break;
    }

    frc::SmartDashboard::PutNumber("elevator output current", elevatorMotor.GetOutputCurrent());

    // if (elevatorRelativeEncoder.GetVelocity() <= 1 && runMode != ELEVATOR_RELATIVE_RUN)
    // {
    //     ZeroRelativePosition(elevatorData);
    // }

    frc::SmartDashboard::PutNumber("elevator relative pos", elevatorRelativeEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("elevator absolute position", elevatorAbsoluteEncoder.GetPosition());
}

void Elevator::SemiAuto(const RobotData &robotData, ElevatorData &elevatorData)
{
    if (!softLimitsEnabled) 
    {
        EnableSoftLimits();
    }

    if (elevatorRelativeEncoder.GetPosition() > 40)
    {
        elevatorData.drivebaseSlowMode = true;
    }
    else
    {
        elevatorData.drivebaseSlowMode = false;
    }

    if (elevatorData.elevatorAbsoluteEncoderInitialized && runMode != ELEVATOR_ABSOLUTE_RUN)
    {
        runMode = ELEVATOR_RELATIVE_RUN;
        elevatorPIDController.SetFeedbackDevice(elevatorRelativeEncoder);
    }
    
    if (elevatorForceZeroed && runMode != ELEVATOR_RELATIVE_RUN)
    {
        runMode = ELEVATOR_RELATIVE_RUN;
        elevatorPIDController.SetFeedbackDevice(elevatorRelativeEncoder);
    }
    // runMode = ELEVATOR_RELATIVE_RUN;
    // elevatorPIDController.SetFeedbackDevice(elevatorRelativeEncoder);

    if (runMode != ELEVATOR_NONE)
    {
        if (robotData.controlData.saHomePosition)
        {
            MoveElevator(10, robotData, 0);
        }
        else if (robotData.controlData.saPositionMid)
        {
            MoveElevator(24, robotData, 0);
        }
        else if (robotData.controlData.saPositionHigh)
        {
            MoveElevator(50, robotData, 0);
        }
        else if (robotData.controlData.saSetUpPosition)
        {
            MoveElevator(0, robotData, 0);
        }
    }
    else
    {
        elevatorMotor.Set(0);
    }

    // if ((robotData.endEffectorData.pastReadOfGamePiece != NONE) && (robotData.endEffectorData.gamePieceType == NONE))
    // {
    //     MoveElevator(10, robotData, 0.25);
    // }

    if (elevatorProfileActive)
    {
        if (robotData.timerData.secSinceEnabled > elevatorProfileStartTime)
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


}

void Elevator::Manual(const RobotData &robotData, ElevatorData &elevatorData)
{
    elevatorData.drivebaseSlowMode = false;
    if (softLimitsEnabled) 
    {
        DisableSoftLimits();
    }
    
    if (robotData.controlData.forceZeroElevator || robotData.controlData.mForceZeroElevator)
    {
        ForceZeroElevator();
    }


    if ((robotData.controllerData.sLYStick > 0.08 || robotData.controllerData.sLYStick < -0.08) && !robotData.controlData.shift)
    {
        elevatorMotor.Set(robotData.controllerData.sLYStick * 1);
    }
    else
    {
        elevatorMotor.Set(0);
    }
}

void Elevator::MoveElevator(double targetPos, const RobotData& robotData, double timeOffset)
{
    elevatorTimeOffset = timeOffset;
    elevatorProfileActive = true;
    elevatorProfileStartTime = robotData.timerData.secSinceEnabled+timeOffset;

    if (runMode == ELEVATOR_ABSOLUTE_RUN)
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
        frc::TrapezoidProfile<units::degrees>::Constraints{240_deg_per_s, 140_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{elevatorProfileEndPos}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{elevatorProfileStartPos}, units::angular_velocity::degrees_per_second_t{0}}
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

    elevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, elevatorMinPosition + 0.05);
    elevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, elevatorMaxPosition - 0.1);
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
        runMode = ELEVATOR_NONE;
    }

    return elevatorData.elevatorAbsoluteEncoderInitialized;
}