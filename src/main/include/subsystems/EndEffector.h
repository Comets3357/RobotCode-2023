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

struct EndEffectorData
{

};

class EndEffector
{
public:

    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, EndEffectorData &endEffectorData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, EndEffectorData &endEffectorData);
    void UpdateData(const RobotData &robotData, EndEffectorData &endEffectorData);

private:

    void SetIntakeRollerSpeed(double rollerSpeed);
    void SemiAuto(const RobotData &robotData, EndEffectorData &endEffectorData);
    void Manual(const RobotData &robotData, EndEffectorData &endEffectorData);
    
    // Intake Roller Initialization
    rev::CANSparkMax intakeRollers = rev::CANSparkMax(intakeRollerID, rev::CANSparkMax::MotorType::kBrushless);

    double intakeRollerOutwardSpeed = 0.4;
    double intakeRollerInwardSpeed = -0.4;

    double intakeRollerCubeInwardSpeed = 0.4;

    bool softLimitsToggled = false;
};
