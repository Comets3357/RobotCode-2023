#pragma once
#include "Constants.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <cmath>
#include <deque>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

struct RobotData;

struct LimelightData
{

};

class Limelight
{
public:
    void RobotPeriodic(const RobotData &robotData, LimelightData &limelightData);

private:

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); 
    // double botPose[6] = {0,0,0,0,0,0};

};