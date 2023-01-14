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

    // Intake Pivot
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