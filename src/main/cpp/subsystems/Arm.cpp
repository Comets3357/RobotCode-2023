#include "subsystems/Arm.h"
#include "RobotData.h"
#include <cmath>

void Arm::RobotInit(ArmData &armData)
{
    // Wrist Initialization
    armWristPIDController.SetP(0.1, 0);
    armWristPIDController.SetI(0, 0);
    armWristPIDController.SetD(0, 0);
    armWristPIDController.SetIZone(0, 0);
    armWristPIDController.SetFF(0, 0);

    armWristPIDController.SetOutputRange(-1, 1, 0);
    armWrist.EnableVoltageCompensation(10.5);
    armWrist.SetSmartCurrentLimit(45);

    armWrist.BurnFlash();

    armPivotPIDController.SetP(0.1, 0);
    armPivotPIDController.SetI(0, 0);
    armPivotPIDController.SetD(0, 0);
    armPivotPIDController.SetIZone(0, 0);
    armPivotPIDController.SetFF(0, 0);

    armPivotPIDController.SetOutputRange(-1, 1, 0);
    armPivot.EnableVoltageCompensation(10.5);
    armPivot.SetSmartCurrentLimit(45);

    armPivot.BurnFlash();

    ZeroRelativePositionWrist(armData);
    ZeroRelativePositionPivot(armData);

    // ToggleSoftLimits();
}

void Arm::RobotPeriodic(const RobotData &robotData, ArmData &armData)
{
   // changing controls based off the mode robot is in
    switch (robotData.controlData.mode) 
    {
        case MODE_TELEOP_MANUAL:
            Manual(robotData, armData);
            break;
        case MODE_TELEOP_SA:
            SemiAuto(robotData, armData);
            break;
        default:
            SemiAuto(robotData, armData);
            break;
    }
}

void Arm::SemiAuto(const RobotData &robotData, ArmData &armData)
{

    // if (!softLimitsToggled)
    // {
    //     ToggleSoftLimits();
    // }

    if (robotData.controlData.saArmIntakePosition)
    {
        // SetAngleOfWrist(armData, 0);
        // SetAngleOfPivot(armData, 0);
    }
    if (robotData.controlData.saMoveArm)
    {
        
    }
}

void Arm::Manual(const RobotData &robotData, ArmData &armData)
{
    // if (softLimitsToggled)
    // {
    //     ToggleSoftLimits();
    // }
    if (robotData.controlData.mMovePivot)
    {
        armPivot.Set(robotData.controllerData.sLYStick);
    }

    if (robotData.controlData.mMoveWrist)
    {
        armWrist.Set(robotData.controllerData.sRYStick);
    }
}

void Arm::DisabledInit()
{

}

void Arm::DisabledPeriodic(const RobotData &robotData, ArmData &armData)
{

}
void Arm::UpdateData(const RobotData &robotData, ArmData &armData)
{
    
}

void Arm::SetAngleOfWrist(ArmData &armData, double desiredAngle)
{
    if (armData.wristInitialized)
    {
        armWristPIDController.SetReference(desiredAngle, rev::CANSparkMax::ControlType::kDutyCycle);
    }
    else 
    {
        armWristPIDController.SetReference(desiredAngle, rev::CANSparkMax::ControlType::kPosition);

    }
}

void Arm::SetAngleOfPivot(ArmData &armData, double desiredAngle)
{
    if (armData.pivotInitialized)
    {
        armPivotPIDController.SetReference(desiredAngle, rev::CANSparkMax::ControlType::kDutyCycle);
    }
    else
    {
        armPivotPIDController.SetReference(desiredAngle, rev::CANSparkMax::ControlType::kPosition);

    }
}

void Arm::DisableSoftLimits()
{
    armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
        
    armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);

    softLimitsToggled = false;
}

void Arm::EnableSoftLimits(ArmData &armData)
{
    if (armData.pivotInitialized && armData.wristInitialized)
    {
        armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armWristAbosluteMinPosition - 0.1);
        armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armWristAbsoluteMaxPosition + 0.1);

        armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armPivotAbsoluteMinPosition - 0.1);
        armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armPivotAbsoluteMaxPosition + 0.1);
    }
    else 
    {
        armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armWristRelativeMinPosition - 0.1);
        armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armWristRelativeMaxPosition + 0.1);

        armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armPivotRelativeMinPosition - 0.1);
        armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armPivotRelativeMaxPosition + 0.1);        
    }


    armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    softLimitsToggled = true;
}