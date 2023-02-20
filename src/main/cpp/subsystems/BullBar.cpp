#include "subsystems/BullBar.h"
#include "RobotData.h"
#include <cmath>

void BullBar::RobotInit(const RobotData &robotData, BullBarData &bullBarData)
{ // check current vals and then burn flash if they are different
    // BullBar Rollers

    bullBarRollers.SetInverted(robotData.configData.bullBarConfigData.invertRollers);


    bullBarSliderAbsoluteEncoder.SetInverted(robotData.configData.bullBarConfigData.invertSliderAbsolute);
    bullBarSliderAbsoluteEncoder.SetPositionConversionFactor(robotData.configData.bullBarConfigData.absoluteConversion);
    bullBarSliderAbsoluteEncoder.SetZeroOffset(robotData.configData.bullBarConfigData.absoluteOffset);

    bullBarSliderRelativeEncoder.SetPositionConversionFactor(robotData.configData.bullBarConfigData.relativeConversion);
    bullBarSliderRelativeEncoder.SetPosition(10);

    bullBarSliderPIDController.SetFeedbackDevice(bullBarSliderAbsoluteEncoder);

    // BullBar 

    // abs
    bullBarSliderPIDController.SetP(robotData.configData.bullBarConfigData.pValue, 0);
    bullBarSliderPIDController.SetI(0, 0);  
    bullBarSliderPIDController.SetD(0, 0);
    bullBarSliderPIDController.SetIZone(0, 0);
    bullBarSliderPIDController.SetFF(0, 0);
    bullBarSliderPIDController.SetOutputRange(-1, 1, 0);

    bullBarSlider.EnableVoltageCompensation(robotData.configData.bullBarConfigData.voltageComp);
    bullBarSlider.SetSmartCurrentLimit(robotData.configData.bullBarConfigData.currentLimit);
    bullBarSlider.SetInverted(robotData.configData.bullBarConfigData.invertSliderRelative);
    bullBarSlider.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    bullBarSlider.BurnFlash();
    

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

    frc::SmartDashboard::PutNumber("encoder", bullBarSliderRelativeEncoder.GetPosition());

    // if (bullBarSliderRelativeEncoder.GetVelocity() <= 1) // && inRelativeMode
    // {
    //     ZeroRelativePosition(bullBarData);
    // }

    // UpdateData(robotData, bullBarData);
    frc::SmartDashboard::PutNumber("bull bar abs position", bullBarSliderAbsoluteEncoder.GetPosition());

    if (forceZero)
    {
        ForceZeroBullBar();
    }

    frc::SmartDashboard::PutBoolean("soft limits toggled", softLimitsToggled);

    UpdateData(robotData, bullBarData);

    frc::SmartDashboard::PutBoolean("RBUMPER", robotData.controllerData.sRBumper);
}


void BullBar::SemiAuto(const RobotData &robotData, BullBarData &bullBarData)
{
    if (!softLimitsToggled)
    {
        EnableSoftLimits(bullBarData);
    }

    // EnableSoftLimits(bullBarData);

    frc::SmartDashboard::PutNumber("run mode bull bar", runMode);

    if (bullBarData.bullBarAbsoluteEncoderInitialized && runMode != BULLBAR_ABSOLUTE_RUN)
    {
        runMode = BULLBAR_ABSOLUTE_RUN;
        bullBarSliderPIDController.SetFeedbackDevice(bullBarSliderAbsoluteEncoder);
    }
    else if ((bullBarForcedZeroed && runMode != BULLBAR_RELATIVE_RUN))
    {
        runMode = BULLBAR_RELATIVE_RUN;
        bullBarSliderPIDController.SetFeedbackDevice(bullBarSliderRelativeEncoder);
    }

    if ((bullBarSliderAbsoluteEncoder.GetPosition() > bullBarCubeIntakePosition - 0.5) 
    || (bullBarSliderRelativeEncoder.GetPosition() > bullBarCubeIntakePosition - 0.5))
    {
        bullBarData.bullBarSafePosition = true;
    }
    else
    {
        bullBarData.bullBarSafePosition = false;
    }

    if ((bullBarSliderAbsoluteEncoder.GetPosition() < 12 && bullBarData.bullBarAbsoluteEncoderInitialized) || bullBarSliderRelativeEncoder.GetPosition() < 12)
    {
        bullBarData.bullBarUprightConeSafePosition = true;
    }
    else
    {
        bullBarData.bullBarUprightConeSafePosition = false;
    }
    frc::SmartDashboard::PutBoolean("Bull Bar Safe", bullBarData.bullBarSafePosition);

    if (runMode != BULLBAR_NONE)
    {
        if (robotData.controlData.saConeIntake)
        {
            if (robotData.armData.wristSafePosition)
            {
                bullBarSliderPIDController.SetReference(bullBarConeIntakePosition, rev::CANSparkMax::ControlType::kPosition, 0);
            }
            if (robotData.controlData.shift)
            {
                bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -bullBarRollerExtendedSpeed);
            }
            else
            {
                bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, bullBarRollerRetractedSpeed);
            }

            
        }
        else if (robotData.controlData.saConeFlipPosition)
        {
            if (robotData.armData.wristSafePosition)
            {
                bullBarSliderPIDController.SetReference(16, rev::CANSparkMax::ControlType::kPosition, 0);
            }
            bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, bullBarRollerExtendedSpeed);  
        }
        else if (robotData.controlData.saCubeIntake)
        {
            if (robotData.armData.wristSafePosition)
            {
                bullBarSliderPIDController.SetReference(bullBarCubeIntakePosition, rev::CANSparkMax::ControlType::kPosition, 0);
            }
            if (robotData.controlData.shift)
            {
                bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -bullBarRollerExtendedSpeed); 
            }
            else
            {
                bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, bullBarRollerExtendedSpeed); 
            }  
        }
        else
        {
            if (robotData.armData.wristSafePosition)
            {
                bullBarSliderPIDController.SetReference(bullBarMinPosition, rev::CANSparkMax::ControlType::kPosition, 0);
                
            }
            if(!bullBarData.bullBarUprightConeSafePosition)
            {
                bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -bullBarRollerExtendedSpeed); 
            }
            else
            {
                bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0); 
            }
        }
    }
    else
    {
        bullBarSlider.Set(0);
        bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    }
    
}

void BullBar::Manual(const RobotData &robotData, BullBarData &bullBarData)
{

    frc::SmartDashboard::PutBoolean("l working", true);
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
        bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.8);
    }
    else if (robotData.controlData.mBullBarRollerBackward)
    {
        bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.8);
    }
    else 
    {
        bullBarRollers.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
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
    bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, bullBarMinPosition + 1);
    bullBarSlider.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, bullBarMaxPosition - 0.35); 

    bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    bullBarSlider.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    softLimitsToggled = true;
}

/*
* @note Checks to see if the absolute encoder xhas initialized
*/
bool BullBar::IsAbsoluteEncoderInitialized(BullBarData &bullBarData)
{
    if (bullBarSliderAbsoluteEncoder.GetPosition() >= 0.01)
    {
        bullBarData.bullBarAbsoluteEncoderInitialized = true;
        runMode = BULLBAR_ABSOLUTE_RUN;
    }
    else 
    {
        bullBarData.bullBarAbsoluteEncoderInitialized = false;
        runMode = BULLBAR_NONE;
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