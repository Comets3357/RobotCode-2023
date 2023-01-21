#include "subsystems/BullBar.h"
#include "RobotData.h"
#include <cmath>

void BullBar::RobotInit(BullBarData &bullBarData)
{
    // BullBar Rollers
    bullBarRollers.RestoreFactoryDefaults();
    bullBarRollers.SetInverted(true);
    bullBarRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    bullBarRollers.SetSmartCurrentLimit(45);
    bullBarRollers.EnableVoltageCompensation(10.5);

    bullBarRollers.BurnFlash();

    // BullBar Pivot
    bullBarSliderPIDController.SetP(3, 0);
    bullBarSliderPIDController.SetI(0, 0);
    bullBarSliderPIDController.SetD(0, 0);
    bullBarSliderPIDController.SetIZone(0, 0);
    bullBarSliderPIDController.SetFF(0, 0);
    bullBarSliderPIDController.SetOutputRange(-1, 1, 0);
    bullBarSlider.EnableVoltageCompensation(10.5);
    bullBarSlider.SetSmartCurrentLimit(20);
    bullBarSlider.SetInverted(true);
    bullBarSlider.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    bullBarSlider.BurnFlash();

    ZeroRelativePosition(bullBarData);
    ToggleSoftLimits();

    frc::SmartDashboard::PutBoolean("FORCE ZERO BULL BAR", 0);
    frc::SmartDashboard::PutNumber("bull bar abs position", bullBarSliderAbsoluteEncoder.GetPosition());
}

void BullBar::RobotPeriodic(const RobotData &robotData, BullBarData &bullBarData)
{
    // changing controls based off the mode robot is in
    switch (robotData.controlData.mode) 
    {
        case MODE_TELEOP_MANUAL:
            Manual(robotData, bullBarData);
            break;
        case MODE_TELEOP_SA:
            SemiAuto(robotData, bullBarData);
            break;
        default:
            SemiAuto(robotData, bullBarData);
            break;
    }

    if (bullBarSliderRelativeEncoder.GetVelocity() <= 1)
    {
        ZeroRelativePosition(bullBarData);
    }

    UpdateData(robotData, bullBarData);
    frc::SmartDashboard::PutNumber("bull bar abs position", bullBarSliderAbsoluteEncoder.GetPosition());

    if (forceZero)
    {
        ForceZeroBullBar();
    }

}


void BullBar::SemiAuto(const RobotData &robotData, BullBarData &bullBarData)
{
    if (!softLimitsToggled)
    {
        ToggleSoftLimits();
    }

    // frc::SmartDashboard::PutBoolean("")

    // Absolute encoder is initialized and the code the abs position is used
    if (bullBarData.bullBarAbsoluteEncoderInitialized)
    {
        if (robotData.controlData.saConeIntake)
        {
            bullBarSliderPIDController.SetReference(bullBarConeIntakeAbsolutePosition, rev::CANSparkMax::ControlType::kDutyCycle);
            bullBarRollers.Set(bullBarRollerExtendedSpeed);
        }
        else if (robotData.controlData.saCubeIntake)
        {
            bullBarSliderPIDController.SetReference(bullBarCubeIntakeAbsolutePosition, rev::CANSparkMax::ControlType::kDutyCycle);
            bullBarRollers.Set(bullBarRollerRetractedSpeed);
        }
        else
        {
            // bullBarSliderPIDController.SetReference(bullBarAbsoluteMinPosition, rev::CANSparkMax::ControlType::kDutyCycle);
            bullBarSlider.Set(0);
            bullBarRollers.Set(0);
        }
    }
    else // abs encoder is not up, so we default off of relative encoder
    {
        if (robotData.controlData.saConeIntake)
        {
            bullBarSliderPIDController.SetReference(bullBarRelativeMaxPosition, rev::CANSparkMax::ControlType::kPosition);
            bullBarRollers.Set(bullBarRollerExtendedSpeed);
        }
        else if (robotData.controlData.saCubeIntake)
        {
            bullBarSliderPIDController.SetReference(bullBarRelativeMaxPosition - 10, rev::CANSparkMax::ControlType::kPosition);
            bullBarRollers.Set(bullBarRollerRetractedSpeed);
        }
        else
        {
            bullBarSliderPIDController.SetReference(bullBarRelativeMinPosition, rev::CANSparkMax::ControlType::kPosition);
            bullBarRollers.Set(0);
        }
    }
    
}

void BullBar::Manual(const RobotData &robotData, BullBarData &bullBarData)
{

    frc::SmartDashboard::PutBoolean("manual working", true);
    if (softLimitsToggled)
    {
        ToggleSoftLimits();
    }

    if (robotData.controlData.mBullBarExtension)
    { 
        bullBarSlider.Set(robotData.controllerData.sRYStick * 0.25);
    }
    else 
    {
        bullBarSlider.Set(0);
    }

    if (robotData.controlData.mBullBarRollerForward)
    { 
        bullBarRollers.Set(0.6);
    }
    else if (robotData.controlData.mBullBarRollerBackward)
    {
        bullBarRollers.Set(-0.6);
    }
    else 
    {
        bullBarRollers.Set(0);
    }

    if (robotData.controlData.mForceZeroBullBar)
    {
        ForceZeroBullBar();
    }
}


void BullBar::UpdateData(const RobotData &robotData, BullBarData &bullBarData)
{
    // frc::SmartDashboard::PutNumber("bull bar abs position", bullBarSliderAbsoluteEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("bull bar abs init successful", bullBarData.bullBarAbsoluteEncoderInitialized);
    
    forceZero = frc::SmartDashboard::GetBoolean("FORCE ZERO BULL BAR", 0);
}

/*
* @note Zeros the relative position on the Slider motor to its realitive position
*/
void BullBar::ZeroRelativePosition(BullBarData &bullBarData)
{
    if (IsAbsoluteEncoderInitialized(bullBarData))
    {
        bullBarSliderRelativeEncoder.SetPosition(AbsoluteToRelative(bullBarSliderAbsoluteEncoder.GetPosition()));
        frc::SmartDashboard::PutNumber("relative zeroed position", AbsoluteToRelative(bullBarSliderAbsoluteEncoder.GetPosition()));
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
        bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
        bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);

        softLimitsToggled = false;
    }
    else if (!softLimitsToggled) 
    {
        bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

        bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, bullBarRelativeMinPosition - 0.1);
        bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, bullBarRelativeMaxPosition + 0.1);

        softLimitsToggled = true;
    }
}

/*
* @note Checks to see if the absolute encoder has initialized
*/
bool BullBar::IsAbsoluteEncoderInitialized(BullBarData &bullBarData)
{
    if (bullBarSliderAbsoluteEncoder.GetPosition() >= 0.01)
    {
        bullBarData.bullBarAbsoluteEncoderInitialized = true;
    }
    else 
    {
        bullBarData.bullBarAbsoluteEncoderInitialized = false;
    }

    return bullBarData.bullBarAbsoluteEncoderInitialized;
}

/*
* @note If all else fails, this is what 
* @note driver uses to force zero the bull
* @note bar to ensure functionality
*/
void BullBar::ForceZeroBullBar()
{
    bullBarSliderRelativeEncoder.SetPosition(0);
}


void BullBar::DisabledInit()
{
    bullBarSlider.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void BullBar::DisabledPeriodic(const RobotData &robotData, BullBarData &bullBarData)
{

}