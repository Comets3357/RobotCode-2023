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

struct IntakeData
{

};

class Intake
{
public:

    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, IntakeData &intakeData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, IntakeData &intakeData);
    void updateData(const RobotData &robotData, IntakeData &intakeData);

private:

    double AbsoluteToRelative(double currentAbsolutePosition);

    // Intake Roller Initialization
    rev::CANSparkMax intakeRollers = rev::CANSparkMax(intakeRollerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakeRollersRelativeEncoder = intakeRollers.GetEncoder(); // Relative Encoder

    // Intake Pivot Initialization
    rev::CANSparkMax intakePivot = rev::CANSparkMax(intakePivotID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder intakePivotRelativeEncoder = intakePivot.GetEncoder(); // Relative Encoder
    rev::SparkMaxPIDController intakePivotPIDController = intakePivot.GetPIDController(); // PID Controller
    frc::DigitalInput m_input{intakeAbsoluteEncoderPort};
    frc::DutyCycle intakePivotAbsoluteEncoder = frc::DutyCycle{m_input}; // Absolute Encoder

    // Encoder Min and Max Values
    double intakePivotRelativeMaxPosition = 13;
    double intakePivotRelativeMinPosition = 0;

    double intakePivotAbsoluteMaxPosition = 0.93418697534;
    double intakePivotAbosluteMinPosition = 0.14207;
};
