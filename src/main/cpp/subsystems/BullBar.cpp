#include "subsystems/BullBar.h"
#include "RobotData.h"
#include <cmath>

void BullBar::RobotInit(BullBarData &bullBarData)
{ // check current vals and then burn flash if they are different
    // BullBar Rollers
    bullBarRollers.RestoreFactoryDefaults();
    bullBarRollers.SetInverted(true);
    bullBarRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    bullBarRollers.SetSmartCurrentLimit(45);
    bullBarRollers.EnableVoltageCompensation(10.5);
    bullBarRollers.BurnFlash();


    bullBarSliderAbsoluteEncoder.SetInverted(true);
    // bullBarSliderAbsoluteEncoder.SetPositionConversionFactor(108.43);
    // bullBarSliderAbsoluteEncoder.SetZeroOffset(-111.49);

    // bullBarSliderRelativeEncoder.SetPositionConversionFactor(5.25);

    bullBarSliderAbsoluteEncoder.SetPositionConversionFactor(108.43);
    bullBarSliderAbsoluteEncoder.SetZeroOffset(47.5);
    

    bullBarSliderRelativeEncoder.SetPositionConversionFactor(0.19048);

    bullBarSliderPIDController.SetFeedbackDevice(bullBarSliderAbsoluteEncoder);

    // BullBar Pivot

    // abs
    bullBarSliderPIDController.SetP(0.225, 0);
    bullBarSliderPIDController.SetI(0, 0);  
    bullBarSliderPIDController.SetD(0, 0);
    bullBarSliderPIDController.SetIZone(0, 0);
    bullBarSliderPIDController.SetFF(0, 0);
    bullBarSliderPIDController.SetOutputRange(-1, 1, 0);

    // relative
    bullBarSliderPIDController.SetP(0.225, 1);
    bullBarSliderPIDController.SetI(0, 1);
    bullBarSliderPIDController.SetD(0, 1);
    bullBarSliderPIDController.SetIZone(0, 1);
    bullBarSliderPIDController.SetFF(0, 1);
    bullBarSliderPIDController.SetOutputRange(-1, 1, 1);

    bullBarSlider.EnableVoltageCompensation(10.5);
    bullBarSlider.SetSmartCurrentLimit(20);
    bullBarSlider.SetInverted(true);
    bullBarSlider.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    bullBarSlider.BurnFlash();
    

    ZeroRelativePosition(bullBarData);
    EnableSoftLimits(bullBarData);

    frc::SmartDashboard::PutBoolean("FORCE ZERO BULL BAR", 0);
    frc::SmartDashboard::PutNumber("bull bar abs position", bullBarSliderAbsoluteEncoder.GetPosition());
}

void BullBar::RobotPeriodic(const RobotData &robotData, BullBarData &bullBarData)
{

    
    // if (absoluteWasInitialized && !IsAbsoluteEncoderInitialized(bullBarData));
    // {
    //     EnableSoftLimits(bullBarData);
    // }
    
    absoluteWasInitialized = IsAbsoluteEncoderInitialized(bullBarData);

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

    frc::SmartDashboard::PutNumber("encoder", bullBarSliderRelativeEncoder.GetPosition());

    if (bullBarSliderRelativeEncoder.GetVelocity() <= 1) // && inRelativeMode
    {
        ZeroRelativePosition(bullBarData);
    }

    UpdateData(robotData, bullBarData);
    frc::SmartDashboard::PutNumber("bull bar abs position", bullBarSliderAbsoluteEncoder.GetPosition());

    // if (forceZero)
    // {
    //     ForceZeroBullBar();
    // }

    frc::SmartDashboard::PutBoolean("soft limits toggled", softLimitsToggled);

    UpdateData(robotData, bullBarData);


}


void BullBar::SemiAuto(const RobotData &robotData, BullBarData &bullBarData)
{
    // if (!softLimitsToggled)
    // {
    //     EnableSoftLimits(bullBarData);
    // }

    EnableSoftLimits(bullBarData);

    frc::SmartDashboard::PutNumber("run mode bull bar", runMode);

    if (bullBarData.bullBarAbsoluteEncoderInitialized && runMode != ABSOLUTE_RUN)
    {
        runMode = ABSOLUTE_RUN;
        bullBarSliderPIDController.SetFeedbackDevice(bullBarSliderAbsoluteEncoder);
    }
    else if ((bullBarForcedZeroed && runMode != RELATIVE_RUN))
    {
        runMode = RELATIVE_RUN;
        bullBarSliderPIDController.SetFeedbackDevice(bullBarSliderRelativeEncoder);
    }

    if (runMode != NONE)
    {
        if (robotData.controlData.saConeIntake)
        {
            bullBarSliderPIDController.SetReference(bullBarConeIntakePosition, rev::CANSparkMax::ControlType::kPosition, 0);
            bullBarRollers.Set(bullBarRollerRetractedSpeed);
        }
        else if (robotData.controlData.saCubeIntake)
        {
            bullBarSliderPIDController.SetReference(bullBarCubeIntakePosition, rev::CANSparkMax::ControlType::kPosition, 0);
            bullBarRollers.Set(bullBarRollerExtendedSpeed);
        }
        else
        {
            bullBarSliderPIDController.SetReference(bullBarMinPosition, rev::CANSparkMax::ControlType::kPosition, 0);
            bullBarRollers.Set(0);
        }
    }
    else
    {
        bullBarSlider.Set(0);
        bullBarRollers.Set(0);
    }
    
}

void BullBar::Manual(const RobotData &robotData, BullBarData &bullBarData)
{

    frc::SmartDashboard::PutBoolean("manual working", true);
    if (softLimitsToggled)
    {
        DisableSoftLimits();
    }
    // EnableSoftLimits(bullBarData);

    

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
        bullBarSliderRelativeEncoder.SetPosition(bullBarSliderAbsoluteEncoder.GetPosition());
        frc::SmartDashboard::PutNumber("relative zeroed position", bullBarSliderRelativeEncoder.GetPosition());
    }
}

/*
* @param currentAbsolutePosition takes in current absolute position and converts to relative
*/
// double BullBar::AbsoluteToRelative(double currentAbsolutePosition)
// {
//     double slope = (bullBarRelativeMaxPosition - bullBarRelativeMinPosition) / (bullBarAbsoluteMaxPosition - bullBarAbsoluteMinPosition);
//     double b = bullBarRelativeMinPosition - (slope * bullBarAbsoluteMinPosition);
//     return ((slope * currentAbsolutePosition) + b);
// }

/*
* @note Disables soft limits
*/
void BullBar::DisableSoftLimits() 
{
    bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);

    softLimitsToggled = false;
}

/*
* @note Enables soft limits
*/
void BullBar::EnableSoftLimits(BullBarData &bullBarData)
{
    // if (bullBarData.bullBarAbsoluteEncoderInitialized)
    // {
    //     bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, bullBarMinPosition + 0.3);
    //     bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, bullBarMaxPosition - 0.3); 
    // }
    // else
    // {
    //     bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, bullBarMinPosition + 0.3);
    //     bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, bullBarMaxPosition - 0.3); 
    // }

    
    bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, bullBarMinPosition + 0.7);
    bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, bullBarMaxPosition - 0.3); 

    bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    softLimitsToggled = true;
}

/*
* @note Checks to see if the absolute encoder has initialized
*/
bool BullBar::IsAbsoluteEncoderInitialized(BullBarData &bullBarData)
{
    if (bullBarSliderAbsoluteEncoder.GetPosition() >= 0.01)
    {
        bullBarData.bullBarAbsoluteEncoderInitialized = true;
        runMode = ABSOLUTE_RUN;
    }
    else 
    {
        bullBarData.bullBarAbsoluteEncoderInitialized = false;
        runMode = NONE;
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
    bullBarSliderRelativeEncoder.SetPosition(10);
    bullBarForcedZeroed = true;
}


void BullBar::DisabledInit()
{
    bullBarSlider.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void BullBar::DisabledPeriodic(const RobotData &robotData, BullBarData &bullBarData)
{

}