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
    bool isCone = true;
    bool isCube = false;
};

class EndEffector
{
public:

    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, EndEffectorData &endEffectorData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, EndEffectorData &endEffectorData);

private:

    void SetEndEffectorRollerSpeed(double rollerSpeed);
    void SemiAuto(const RobotData &robotData, EndEffectorData &endEffectorData);
    void Manual(const RobotData &robotData, EndEffectorData &endEffectorData);
    void UpdateData(const RobotData &robotData, EndEffectorData &endEffectorData);
    
    // EndEffector Roller Initialization
    rev::CANSparkMax endEffectorRollers = rev::CANSparkMax(endEffectorRollerID, rev::CANSparkMax::MotorType::kBrushless);

    rev::SparkMaxLimitSwitch coneLimitSwitch = endEffectorRollers.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
    rev::SparkMaxLimitSwitch cubeLimitSwitch = endEffectorRollers.GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);

    double EndEffectorRollerOutwardSpeed = 0.5;
    double EndEffectorRollerInwardSpeed = -0.5;

    double EndEffectorRollerCubeInwardSpeed = 0.2;

};
