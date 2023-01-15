#include "subsystems/BullBar.h"
#include "RobotData.h"
#include <cmath>

void BullBar::RobotInit(BullBarData &bullbarData)
{
    // BullBar Rollers
    bullbarRollers.RestoreFactoryDefaults();
    bullbarRollers.SetInverted(true);
    bullbarRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    bullbarRollers.SetSmartCurrentLimit(45);
    bullbarRollers.EnableVoltageCompensation(10.5);
    bullbarRollers.BurnFlash();

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

    ZeroRelativePosition(bullbarData);
    ToggleSoftLimits();
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

    if (bullbarSliderRelativeEncoder.GetVelocity() <= 1)
    {
        ZeroRelativePosition(bullbarData);
    }

}


void BullBar::SemiAuto(const RobotData &robotData, BullBarData &bullbarData)
{
    if (!softLimitsToggled)
    {
        ToggleSoftLimits();
    }

    if (bullbarData.bullBarAbsoluteEncoderInitialized)
    {
        if (robotData.controlData.saBullBarExtension)
        {
            bullbarSliderPIDController.SetReference(bullBarAbsoluteMaxPosition, rev::CANSparkMax::ControlType::kDutyCycle);
            bullbarRollers.Set(bullBarRollerExtendedSpeed);
        }
        else
        {
            bullbarSliderPIDController.SetReference(bullBarAbsoluteMinPosition, rev::CANSparkMax::ControlType::kDutyCycle);
            bullbarRollers.Set(bullBarRollerRetractedSpeed);
        }
    }
    else 
    {
        if (robotData.controlData.saBullBarExtension)
        {
            bullbarSliderPIDController.SetReference(bullBarRelativeMaxPosition, rev::CANSparkMax::ControlType::kPosition);
            bullbarRollers.Set(bullBarRollerExtendedSpeed);
        }
        else
        {
            bullbarSliderPIDController.SetReference(bullBarRelativeMinPosition, rev::CANSparkMax::ControlType::kPosition);
            bullbarRollers.Set(bullBarRollerRetractedSpeed);
        }
    }
    
}

void BullBar::Manual(const RobotData &robotData, BullBarData &bullbarData)
{
    if (softLimitsToggled)
    {
        ToggleSoftLimits();
    }
}


void BullBar::UpdateData(const RobotData &robotData, BullBarData &bullbarData)
{
    frc::SmartDashboard::PutNumber("BULL BAR ABS POSITION", bullbarSliderAbsoluteEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("BULL BAR ABS ENCODER INIT", bullbarData.bullBarAbsoluteEncoderInitialized);
}

/*
* @note Zeros the relative position on the Slider motor to its realitive position
*/
void BullBar::ZeroRelativePosition(BullBarData &bullbarData)
{
    if (IsAbsoluteEncoderInitialized(bullbarData))
    {
        bullbarSliderRelativeEncoder.SetPosition(AbsoluteToRelative(bullbarSliderAbsoluteEncoder.GetPosition()));
    }
}

/*
* @param currentAbsolutePosition takes in current absolute position and converts to relative
*/
double BullBar::AbsoluteToRelative(double currentAbsolutePosition)
{
    double slope = (bullBarRelativeMaxPosition - bullBarRelativeMinPosition) / (bullBarAbsoluteMaxPosition - bullBarAbsoluteMinPosition);
    double b = bullBarRelativeMinPosition - (slope * bullBarAbsoluteMinPosition);
    return ((slope * currentAbsolutePosition) + b);
}

/*
* @note Toggles the soft limits on and off
* @note for when code switches between manual
* @note and semi automatic
*/
void BullBar::ToggleSoftLimits() 
{
    if (softLimitsToggled)
    {
        bullbarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
        bullbarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
    }
    else if (!softLimitsToggled) 
    {
        bullbarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        bullbarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

        bullbarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, bullBarRelativeMinPosition - 0.1);
        bullbarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, bullBarRelativeMaxPosition + 0.1);
    }
}

/*
* @note Checks to see if the absolute encoder has initialized
*/
bool BullBar::IsAbsoluteEncoderInitialized(BullBarData &bullbarData)
{
    if (bullbarSliderAbsoluteEncoder.GetPosition() >= 0.01)
    {
        bullbarData.bullBarAbsoluteEncoderInitialized = true;
    }
    else 
    {
        bullbarData.bullBarAbsoluteEncoderInitialized = false;
    }

    return bullbarData.bullBarAbsoluteEncoderInitialized;
}

/*
* @note If all else fails, this is what 
* @note driver uses to force zero the bull
* @note bar to ensure functionality
*/
void BullBar::ForceZeroBullBar()
{
    bullbarSliderRelativeEncoder.SetPosition(0);
}


void BullBar::DisabledInit()
{

}

void BullBar::DisabledPeriodic(const RobotData &robotData, BullBarData &bullbarData)
{

}