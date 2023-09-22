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

enum GamePiece
{
    CUBE,
    CONE,
    NONE
};

struct EndEffectorData
{
    GamePiece gamePieceType = NONE;
    GamePiece lastPieceType = CONE;

    GamePiece pastReadOfGamePiece = NONE;

    bool armRetractRequest = false;

    double gamePieceDistance = 0;

    bool gamePieceShuffleboard = false;

    double distanceReading = 0;

};

class EndEffector
{
public:

    void RobotInit(const RobotData &robotData);
    void RobotPeriodic(const RobotData &robotData, EndEffectorData &endEffectorData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, EndEffectorData endEffectorData);

private:

    void SetEndEffectorRollerSpeed(double rollerSpeed);
    void AdvancedSemiAuto(const RobotData &robotData, EndEffectorData &endEffectorData);
    void SemiAuto(const RobotData &robotData, EndEffectorData &endEffectorData);
    void Manual(const RobotData &robotData, EndEffectorData &endEffectorData);
    void UpdateData(const RobotData &robotData, EndEffectorData &endEffectorData);
    
    // EndEffector Roller Initialization
    rev::CANSparkMax endEffectorRollers = rev::CANSparkMax(endEffectorRollerID, rev::CANSparkMax::MotorType::kBrushless);

    rev::SparkMaxLimitSwitch coneLimitSwitch = endEffectorRollers.GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
    rev::SparkMaxLimitSwitch cubeLimitSwitch = endEffectorRollers.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
    //rev::SparkMaxAbsoluteEncoder distanceSensor = endEffectorRollers.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    double EndEffectorRollerOutwardSpeed = 0.8;
    double EndEffectorRollerInwardSpeed = -0.8;

    double EndEffectorRollerCubeInwardSpeed = 0.4;

    bool eject = false;



    

};
