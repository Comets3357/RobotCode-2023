#pragma once

#include "Constants.h"
#include "auton/Auton.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycle.h>
#include <frc/DigitalInput.h>
#include <ctre/Phoenix.h>

struct RobotData;

// Add forceZeroBullBar && ifForceZeroed && switch from abs to relative 
struct BullBarData
{
    bool bullBarAbsoluteEncoderInitialized = false;

    bool bullBarSafePosition = false;
};

enum BullBarRunMode
{
    BULLBAR_ABSOLUTE_RUN,
    BULLBAR_RELATIVE_RUN,
    BULLBAR_NONE
};

class BullBar
{
public:

    void RobotInit(BullBarData &bullBarData);
    void RobotPeriodic(const RobotData &robotData, BullBarData &bullBarData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, BullBarData &bullBarData);

private:

    void UpdateData(const RobotData &robotData, BullBarData &bullBarData);

    //double AbsoluteToRelative(double currentAbsolutePosition);

    void BullBarRollers(double rollerSpeed);
    void BullBarSlider(double sliderPosition);
    void SemiAuto(const RobotData &robotData, BullBarData &bullBarData);
    void Manual(const RobotData &robotData, BullBarData &bullBarData);
    // void ToggleSoftLimits(BullBarData &bullBarData);
    void EnableSoftLimits(BullBarData &bullBarData);
    void DisableSoftLimits();
    void ZeroRelativePosition(BullBarData &bullBarData);
    void ForceZeroBullBar();

    
    bool IsAbsoluteEncoderInitialized(BullBarData &bullBarData);

    BullBarRunMode runMode = BULLBAR_NONE;

    // force zeroing the bull bar
    bool forceZero = false;
    bool bullBarForcedZeroed = false;

    
    // Encoder Min and Max Values 
    double bullBarMaxPosition = 28;
    double bullBarMinPosition = 10;

    // Positions for intaking cone or cube
    double bullBarConeIntakePosition = bullBarMinPosition + 17.208;
    double bullBarCubeIntakePosition = bullBarMinPosition + 12.5;

    // intake speed
    double bullBarRollerExtendedSpeed = 0.9;
    double bullBarRollerRetractedSpeed = 0;
    
    // intake and outake speeds
    double bullBarRollerOutwardSpeed = 0.9;
    double bullBarRollerInwardSpeed = -0.9;
    
    // Bull Bar Roller Initialization
    ctre::phoenix::motorcontrol::can::VictorSPX bullBarRollers{bullBarRollerID};

    // Bull Bar Slider Initialization
    rev::CANSparkMax bullBarSlider = rev::CANSparkMax(bullBarSliderID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder bullBarSliderRelativeEncoder = bullBarSlider.GetEncoder(); // Relative Encoder
    rev::SparkMaxPIDController bullBarSliderPIDController = bullBarSlider.GetPIDController(); // PID Controller
    rev::SparkMaxAbsoluteEncoder bullBarSliderAbsoluteEncoder = bullBarSlider.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    bool softLimitsToggled = false;  
    bool absoluteWasInitialized = false;
};
