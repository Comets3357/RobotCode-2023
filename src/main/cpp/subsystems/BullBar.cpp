#include "subsystems/BullBar.h"
#include "RobotData.h"
#include <cmath>

void BullBar::RobotInit()
{
        // BullBar Rollers
    bullbarRollers.RestoreFactoryDefaults();
    bullbarRollers.SetInverted(true);
    bullbarRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    bullbarRollers.SetSmartCurrentLimit(45);
    bullbarRollers.EnableVoltageCompensation(10.5);

    // BullBar Pivot
    bullbarSliderPIDController.SetP(0.1, 0);
    bullbarSliderPIDController.SetI(0, 0);
    bullbarSliderPIDController.SetD(0, 0);
    bullbarSliderPIDController.SetIZone(0, 0);
    bullbarSliderPIDController.SetFF(0, 0);
    bullbarSliderPIDController.SetOutputRange(-1, 1, 0);
    bullbarSlider.EnableVoltageCompensation(10.5);
    bullbarSlider.SetSmartCurrentLimit(20);
    bullbarSlider.BurnFlash();
}

void BullBar::RobotPeriodic(const RobotData &robotData, BullBarData &bullbarData)
{
    // changing controls based off the mode robot is in
    switch (robotData.controlData.mode) 
    {
        case MODE_TELEOP_MANUAL:
            Manual(robotData, bullbarData);
            break;
        case MODE_TELEOP_SA:
            SemiAuto(robotData, bullbarData);
            break;
        default:
            SemiAuto(robotData, bullbarData);
            break;
    }

}


void BullBar::SemiAuto(const RobotData &robotData, BullBarData &bullbarData)
{
    if (robotData.controlData.saBullBarExtension)
    {
        bullbarSliderPIDController.SetReference(1, rev::CANSparkMax::ControlType::kDutyCycle);
        bullbarRollers.Set(0.5);
    }
    else
    {
        bullbarSliderPIDController.SetReference(0, rev::CANSparkMax::ControlType::kDutyCycle);
        bullbarRollers.Set(0);
    }
}

void BullBar::Manual(const RobotData &robotData, BullBarData &bullbarData)
{

}

void BullBar::DisabledInit()
{

}

void BullBar::DisabledPeriodic(const RobotData &robotData, BullBarData &bullbarData)
{
    
}