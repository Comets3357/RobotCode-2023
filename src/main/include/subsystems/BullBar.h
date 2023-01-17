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
    void ToggleSoftLimits();
    void ZeroRelativePosition(BullBarData &bullBarData);
    void ForceZeroBullBar();
    
    bool IsAbsoluteEncoderInitialized(BullBarData &bullBarData);

    // Positions for intaking cone or cube
    double bullBarConeIntakeAbsolutePosition = 0;
    double bullBarConeIntakeRelativePosition = 0;
    double bullBarCubeIntakeAbsolutePosition = 0;
    double bullBarCubeIntakeRelativePosition = 0;

    // Encoder Min and Max Values 
    double bullBarRelativeMaxPosition = 0;
    double bullBarRelativeMinPosition = 0;
    double bullBarAbsoluteMaxPosition = 0.4203;
    double bullBarAbsoluteMinPosition = 0.295;

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
};
