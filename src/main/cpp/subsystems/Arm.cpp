#include "subsystems/Arm.h"
#include "RobotData.h"
#include <cmath>

void Arm::RobotInit()
{
    // Intake Pivot
    armWristPIDController.SetP(0.1, 0);
    armWristPIDController.SetI(0, 0);
    armWristPIDController.SetD(0, 0);
    armWristPIDController.SetIZone(0, 0);
    armWristPIDController.SetFF(0, 0);
    armWristPIDController.SetOutputRange(-1, 1, 0);
    armWrist.EnableVoltageCompensation(10.5);
    armWrist.SetSmartCurrentLimit(20);
    armWrist.BurnFlash();
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
void Arm::DisabledInit()
{

}
void Arm::DisabledPeriodic(const RobotData &robotData, ArmData &armData)
{

}
void Arm::updateData(const RobotData &robotData, ArmData &armData)
{
    
}

/*
* @param pivotPosition Desired relative encoder pivot position
*/
void Arm::ArmWrist(double wristPosition)
{
    armWristPIDController.SetReference(wristPosition, rev::CANSparkMax::ControlType::kPosition);
}

void Arm::SemiAuto(const RobotData &robotData, ArmData &armData)
{
    if (robotData.controlData.saArmIntakePosition)
    {

    }
    if (robotData.controlData.saMoveArm)
    {
        
    }
}
void Arm::Manual(const RobotData &robotData, ArmData &armData)
{

}

void Arm::ToggleSoftLimits() 
{
    if (softLimitsToggled)
    {
        armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
        armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
    }
    else if (!softLimitsToggled) 
    {
        armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

        armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armWristRelativeMinPosition - 0.1);
        armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armWristRelativeMaxPosition + 0.1);
    }
}