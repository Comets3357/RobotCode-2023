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

    void SetEndEffectorRollerSpeed(double rollerSpeed);
    void SemiAuto(const RobotData &robotData, EndEffectorData &endEffectorData);
    void Manual(const RobotData &robotData, EndEffectorData &endEffectorData);
    
    // EndEffector Roller Initialization
    rev::CANSparkMax endEffectorRollers = rev::CANSparkMax(endEffectorRollerID, rev::CANSparkMax::MotorType::kBrushless);

    double EndEffectorRollerOutwardSpeed = 0.4;
    double EndEffectorRollerInwardSpeed = -0.4;

    double EndEffectorRollerCubeInwardSpeed = 0.4;

    bool softLimitsToggled = false;
};
