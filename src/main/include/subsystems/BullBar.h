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

};

class BullBar
{
public:

    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, BullBarData &bullbarData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, BullBarData &bullbarData);
    void updateData(const RobotData &robotData, BullBarData &bullbarData);

private:

    double AbsoluteToRelative(double currentAbsolutePosition);
    void BullBarRollers(double rollerSpeed);
    void BullBarSlider(double sliderPosition);
    void SemiAuto(const RobotData &robotData, BullBarData &bullbarData);
    void Manual(const RobotData &robotData, BullBarData &bullbarData);
    void ToggleSoftLimits();
    void ZeroBullBar();
    


    // Bull Bar Roller Initialization
    rev::CANSparkMax bullbarRollers = rev::CANSparkMax(bullbarRollerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder bullbarRollersRelativeEncoder = bullbarRollers.GetEncoder(); // Relative Encoder

    // Bull Bar Slider Initialization
    rev::CANSparkMax bullbarSlider = rev::CANSparkMax(bullbarSliderID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder bullbarSliderRelativeEncoder = bullbarSlider.GetEncoder(); // Relative Encoder
    rev::SparkMaxPIDController bullbarSliderPIDController = bullbarSlider.GetPIDController(); // PID Controller
    rev::SparkMaxAbsoluteEncoder bullbarSliderAbsoluteEncoder = bullbarSlider.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    // Encoder Min and Max Values 
    double bullbarSliderRelativeMaxPosition = 13; // TODO: fix this value when we get subsystem
    double bullbarSliderRelativeMinPosition = 0; // TODO: fix this value when we get subsystem

    double bullbarSliderAbsoluteMaxPosition = 0.93418697534; // TODO: fix this value when we get subsystem
    double bullbarSliderAbosluteMinPosition = 0.14207; // TODO: fix this value when we get subsystem

    double bullbarRollerOutwardSpeed = 0.4;
    double bullbarRollerInwardSpeed = -0.4;

    bool softLimitsToggled = false;
};
