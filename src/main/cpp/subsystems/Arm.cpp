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

void Arm::RobotPeriodic()
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
    }
    else if (!softLimitsToggled) 
    {
        armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

        armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armWristRelativeMinPosition - 0.1);
        armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armWristRelativeMaxPosition + 0.1);
    }
}