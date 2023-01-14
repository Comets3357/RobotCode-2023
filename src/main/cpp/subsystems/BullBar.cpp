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

    // Callibrating Relative Based On Absolute Position
    ZeroBullBar();
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

    // reseeding the position of relative if the motor isn't running
    if (bullbarSliderRelativeEncoder.GetVelocity() < 1)
    {
        ZeroBullBar();
    }

}