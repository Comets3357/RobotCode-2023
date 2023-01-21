#include "subsystems/BullBar.h"
#include "RobotData.h"
#include <cmath>

void BullBar::RobotInit(BullBarData &bullBarData)
{ // check current vals and then burn flash if they are different
    // BullBar Rollers
    BullBarRollerBurnFlash();

    // BullBar Slider
    BullBarSliderBurnFlash();
    
    ZeroRelativePosition(bullBarData);
    EnableSoftLimits(bullBarData);

    frc::SmartDashboard::PutBoolean("FORCE ZERO BULL BAR", 0);
    frc::SmartDashboard::PutNumber("bull bar abs position", bullBarSliderAbsoluteEncoder.GetPosition());
}

void BullBar::RobotPeriodic(const RobotData &robotData, BullBarData &bullBarData)
{
    if (absoluteWasInitialized && !IsAbsoluteEncoderInitialized(bullBarData));
    {
        EnableSoftLimits(bullBarData);
    }
    
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

    if (bullBarSliderRelativeEncoder.GetVelocity() <= 1) // && inRelativeMode
    {
        ZeroRelativePosition(bullBarData);
    }

    UpdateData(robotData, bullBarData);
    frc::SmartDashboard::PutNumber("bull bar abs position", bullBarSliderAbsoluteEncoder.GetPosition());

    if (forceZero)
    {
        ForceZeroBullBar();
    }

    frc::SmartDashboard::PutBoolean("soft limits toggled", softLimitsToggled);


}


void BullBar::SemiAuto(const RobotData &robotData, BullBarData &bullBarData)
{
    if (!softLimitsToggled)
    {
        EnableSoftLimits(bullBarData);
    }

    // Absolute encoder is initialized and the code the abs position is used
    if (bullBarData.bullBarAbsoluteEncoderInitialized)
    {
        if (robotData.controlData.saConeIntake)
        {
            bullBarSliderPIDController.SetReference(bullBarConeIntakeAbsolutePosition, rev::CANSparkMax::ControlType::kPosition);
            bullBarRollers.Set(bullBarRollerExtendedSpeed);
        }
        else if (robotData.controlData.saCubeIntake)
        {
            bullBarSliderPIDController.SetReference(bullBarCubeIntakeAbsolutePosition, rev::CANSparkMax::ControlType::kPosition);
            bullBarRollers.Set(bullBarRollerRetractedSpeed);
        }
        else
        {
            bullBarSliderPIDController.SetReference(bullBarAbsoluteMinPosition, rev::CANSparkMax::ControlType::kPosition);
            bullBarRollers.Set(0);
        }
    }
    else // abs encoder is not up, so we default off of relative encoder
    {
        if (robotData.controlData.saConeIntake)
        {
            bullBarSliderPIDController.SetReference(bullBarConeIntakeRelativePosition, rev::CANSparkMax::ControlType::kPosition);
            bullBarRollers.Set(bullBarRollerExtendedSpeed);
        }
        else if (robotData.controlData.saCubeIntake)
        {
            bullBarSliderPIDController.SetReference(bullBarCubeIntakeRelativePosition, rev::CANSparkMax::ControlType::kPosition);
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
        DisableSoftLimits();
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

void BullBar::DisableSoftLimits() 
{
    bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);

    softLimitsToggled = false;
}

void BullBar::EnableSoftLimits(BullBarData &bullBarData)
{
    bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    if (bullBarData.bullBarAbsoluteEncoderInitialized)
    {
        bullBarSliderPIDController.SetFeedbackDevice(bullBarSliderAbsoluteEncoder);
        bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, bullBarAbsoluteMinPosition + 0.005);
        bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, bullBarAbsoluteMaxPosition - 0.005);
    }
    else 
    {
        bullBarSliderPIDController.SetFeedbackDevice(bullBarSliderRelativeEncoder);
        bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, bullBarRelativeMinPosition + 0.1);
        bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, bullBarRelativeMaxPosition - 0.1);
    }   

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
    }
    else 
    {
        bullBarData.bullBarAbsoluteEncoderInitialized = false;
    }

    return bullBarData.bullBarAbsoluteEncoderInitialized;
}

void BullBar::BullBarRollerBurnFlash()
{
    bullBarRollerBurnFlash = false;
    if (bullBarRollers.GetInverted() == false)
    {
        bullBarRollers.SetInverted(true);
        bullBarRollerBurnFlash = true;
    }
    if (bullBarRollers.GetIdleMode() != rev::CANSparkMax::IdleMode::kCoast)
    {
        bullBarRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        bullBarRollerBurnFlash = true;
    }
    if (bullBarRollers.GetVoltageCompensationNominalVoltage() != 10.5)
    {
        bullBarRollers.EnableVoltageCompensation(10.5);
        bullBarRollerBurnFlash = true;
    }
    bullBarRollers.SetSmartCurrentLimit(45);
    if (bullBarRollerBurnFlash = true)
        bullBarRollers.BurnFlash();
}

void BullBar::BullBarSliderBurnFlash()
{
    bullBarSliderBurnFlash = false;
    if (bullBarSliderPIDController.GetP() != 3)
    {
        bullBarSliderPIDController.SetP(3, 0);
        bullBarSliderBurnFlash = true;  
    }
    if (bullBarSliderPIDController.GetI() != 0)
    {
        bullBarSliderPIDController.SetI(0, 0);
        bullBarSliderBurnFlash = true;
    }
    if (bullBarSliderPIDController.GetD() != 0)
    {
        bullBarSliderPIDController.SetD(0, 0);
        bullBarSliderBurnFlash = true;
    }
    if (bullBarSliderPIDController.GetIZone() != 0)
    {
        bullBarSliderPIDController.SetIZone(0, 0);
        bullBarSliderBurnFlash = true;
    }
    if (bullBarSliderPIDController.GetFF() != 0)
    {
        bullBarSliderPIDController.SetFF(0, 0);
        bullBarSliderBurnFlash = true;
    }
    if (bullBarSliderPIDController.GetOutputMin() != -1 || bullBarSliderPIDController.GetOutputMax() != 1)
    {
        bullBarSliderPIDController.SetOutputRange(-1, 1, 0);
        bullBarSliderBurnFlash = true;
    }
    if (bullBarSlider.GetVoltageCompensationNominalVoltage() != 10.5)
    {
        bullBarSlider.EnableVoltageCompensation(10.5);
        bullBarSliderBurnFlash = true;
    }
    if (bullBarSlider.GetInverted() != false)
    {
        bullBarSlider.SetInverted(false);
        bullBarSliderBurnFlash = true;
    }
        if (bullBarSlider.GetIdleMode() != rev::CANSparkMax::IdleMode::kBrake)
    {
        bullBarSlider.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        bullBarSliderBurnFlash = true;
    }
    bullBarSlider.SetSmartCurrentLimit(20);
    if (bullBarSliderBurnFlash = true)
        bullBarSlider.BurnFlash();
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