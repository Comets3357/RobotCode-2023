#pragma once

#include <frc/TimedRobot.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>
#include <wpi/SmallString.h>
#include <vector>
#include <string>
#include <frc/smartdashboard/SendableChooser.h>

#include "controller/Controller.h"


struct RobotData;

struct AutonData
{
    std::string autonRoutineName;
    frc::Trajectory trajectory;
    int autonStep = -1; // starts at -1 because getNextAutonStep() increments
    std::vector<std::string> pathGroup;

    bool autonToggle = false;
};

class Auton
{
public:
    void RobotInit(ControlData &controlData, AutonData &autonData);
    void AutonomousInit(AutonData &autonData);
    void AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControlData &controlData, ControllerData &controllerData);
    void DisabledInit();
private:
    void sendAutonSelectionChooser();

    frc::SendableChooser<std::string> autonChooser;

    int step = 0;


    // secondary controls:
    void potato(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);

    void testplace(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
    // void citrus(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);

    // void nearFieldOne(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);

    // // exit tarmac, collect 1, turn, shoot 2:
    // void taxiShoot(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
    // void taxiShootA(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);

    // // four ball autons (taxiShoot, neighboring ball, termincal shoot):
    // void fourBallB(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
    // void fourBallC(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
    
    void placeCone(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
    void Loading(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
        void OneMiddleClimb(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);

    void TwoMiddleClimb(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
    void Bump(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
    void LoadingNoClimb(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
    void BumpNoClimb(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
    void Traverse(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
    void ThreeLoading(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);

    // void sixBallC(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData);
};