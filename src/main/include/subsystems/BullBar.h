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

struct RobotData;

// Add forceZeroBullBar && ifForceZeroed && switch from abs to relative 
struct BullBarData
{
    bool bullBarAbsoluteEncoderInitialized = false;
};

class BullBar
{
public:

    void RobotInit(BullBarData &bullBarData);
    void RobotPeriodic(const RobotData &robotData, BullBarData &bullBarData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, BullBarData &bullBarData);
    void UpdateData(const RobotData &robotData, BullBarData &bullBarData);

private:

    double AbsoluteToRelative(double currentAbsolutePosition);

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

    // force zeroing the bull bar
    bool forceZero = false;
    bool bullBarForcedZeroed = false;

    
    // Encoder Min and Max Values 
    double bullBarRelativeMaxPosition = 18;
    double bullBarRelativeMinPosition = 0;
    double bullBarAbsoluteMinPosition = 0;
    double bullBarAbsoluteMaxPosition = 18;

    // Positions for intaking cone or cube
    double bullBarConeIntakeAbsolutePosition = bullBarAbsoluteMinPosition + 17.208;
    double bullBarConeIntakeRelativePosition = bullBarRelativeMinPosition + 17.208;
    double bullBarCubeIntakeAbsolutePosition = bullBarAbsoluteMinPosition + 14.497;
    double bullBarCubeIntakeRelativePosition = bullBarRelativeMinPosition + 14.497;

    // intake speed
    double bullBarRollerExtendedSpeed = 0.5;
    double bullBarRollerRetractedSpeed = 0;
    
    // intake and outake speeds
    double bullBarRollerOutwardSpeed = 0.4;
    double bullBarRollerInwardSpeed = -0.4;
    
    // Bull Bar Roller Initialization
    rev::CANSparkMax bullBarRollers = rev::CANSparkMax(bullBarRollerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder bullBarRollersRelativeEncoder = bullBarRollers.GetEncoder(); // Relative Encoder

    // Bull Bar Slider Initialization
    rev::CANSparkMax bullBarSlider = rev::CANSparkMax(bullBarSliderID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder bullBarSliderRelativeEncoder = bullBarSlider.GetEncoder(); // Relative Encoder
    rev::SparkMaxPIDController bullBarSliderPIDController = bullBarSlider.GetPIDController(); // PID Controller
    rev::SparkMaxAbsoluteEncoder bullBarSliderAbsoluteEncoder = bullBarSlider.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    bool softLimitsToggled = false;  
    bool absoluteWasInitialized = false;
};
