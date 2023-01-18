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
        armWristPIDController.SetReference(0.5, rev::CANSparkMax::ControlType::kDutyCycle);
        armPivotPIDController.SetReference(0.5, rev::CANSparkMax::ControlType::kDutyCycle);
    }
    if (robotData.controlData.saMoveArm)
    {
        armWristPIDController.SetReference(0.5, rev::CANSparkMax::ControlType::kDutyCycle);
        armPivotPIDController.SetReference(0.5, rev::CANSparkMax::ControlType::kDutyCycle);
    }
}

void Arm::Manual(const RobotData &robotData, ArmData &armData)
{
    // if (softLimitsToggled)
    // {
    //     ToggleSoftLimits();
    // }
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

/*
* @param pivotPosition Desired relative encoder pivot position
*/
void Arm::ArmWrist(double wristPosition)
{
    armWristPIDController.SetReference(wristPosition, rev::CANSparkMax::ControlType::kPosition);
}

void Arm::ToggleSoftLimits() 
{
    if (softLimitsToggled)
    {
        armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
        armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
        
        armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
        armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);

        softLimitsToggled = false;
    }
    else if (!softLimitsToggled) 
    {
        armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

        armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armWristRelativeMinPosition - 0.1);
        armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armWristRelativeMaxPosition + 0.1);

        armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

        armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armPivotRelativeMinPosition - 0.1);
        armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armPivotRelativeMaxPosition + 0.1);

        softLimitsToggled = true;
    }
}

void Arm::ZeroRelativePositionWrist(ArmData &armData)
{
    if (IsWristAbolsoluteEncoderInitialized(armData))
    {
        armWristRelativeEncoder.SetPosition(AbsoluteToRelativeWrist(armWristAbsoluteEncoder.GetPosition()));
    }
}


void Arm::ZeroRelativePositionPivot(ArmData &armData)
{
    if (IsPivotAbolsoluteEncoderInitialized(armData))
    {
        armPivotRelativeEncoder.SetPosition(AbsoluteToRelativeWrist(armPivotAbsoluteEncoder.GetPosition()));
    }

}

bool Arm::IsWristAbolsoluteEncoderInitialized(ArmData &armData)
{
    if (armWristAbsoluteEncoder.GetPosition() >= 0.01)
    {
        armData.wristInitialized = true;
    }
    else 
    {
        armData.wristInitialized = false;
    }
    
    return armData.wristInitialized;
}

bool Arm::IsPivotAbolsoluteEncoderInitialized(ArmData &armData)
{
    if (armPivotAbsoluteEncoder.GetPosition() >= 0.01)
    {
        armData.pivotInitialized = true;
    }
    else 
    {
        armData.pivotInitialized = false;
    }
    
    return armData.pivotInitialized;
}

double Arm::AbsoluteToRelativeWrist(double currentAbsolutePosition)
{
    double slope = (armWristRelativeMaxPosition - armWristRelativeMinPosition) / (armWristAbsoluteMaxPosition - armWristAbosluteMinPosition);
    double b = armWristRelativeMinPosition - (slope * armWristAbosluteMinPosition);
    return ((slope * currentAbsolutePosition) + b);
}

double Arm::ArmToRelativePivot(double currentAbsolutePosition)
{
    double slope = (armPivotRelativeMaxPosition - armPivotRelativeMinPosition) / (armPivotAbsoluteMaxPosition - armPivotAbsoluteMinPosition);
    double b = armPivotRelativeMinPosition - (slope * armPivotAbsoluteMinPosition);
    return ((slope * currentAbsolutePosition) + b);
}
