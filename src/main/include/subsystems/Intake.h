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
    void IntakeRollers(double rollerSpeed);
    void SemiAuto(const RobotData &robotData, IntakeData &intakeData);
    void Manual(const RobotData &robotData, IntakeData &intakeData);
    


    // Intake Roller Initialization
    rev::CANSparkMax intakeRollers = rev::CANSparkMax(intakeRollerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax intakeRollers2 = rev::CANSparkMax(intakeRoller2ID, rev::CANSparkMax::MotorType::kBrushless);

    rev::SparkMaxRelativeEncoder intakeRollersRelativeEncoder = intakeRollers.GetEncoder(); // Relative Encoder

    double intakeRollerOutwardSpeed = 0.4;
    double intakeRollerInwardSpeed = -0.4;

    double intakeRollerCubeInwardSpeed = 0.4;

    bool softLimitsToggled = false;
};
