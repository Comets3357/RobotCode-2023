#pragma once
#include "Constants.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include "LimelightHelpers.h"
#include <cmath>
#include <deque>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>
#include <math.h>

struct RobotData;

struct LimelightData
{
    double limelightPastOdometryX = 0;
    double limelightPastOdometryY = 0;

    double limelightOdometryX = 0;
    double limelightOdometryY = 0;
};

class Limelight
{
public:
    void RobotPeriodic(const RobotData &robotData, LimelightData &limelightData);

private:

    double GetDistance();

    // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); 
    LimelightHelpers::LimelightResultsClass llresults; //= LimelightHelpers::getLatestResults();
    std::vector<double> limelightOdometry = {0,0,0,0,0,0};


    double tempX = 0;
    double tempY = 0;
    int numberOfTagsInView = 0;

    double distanceToClosestTag = 0;

    double aprilTagHeight = 13.0;
    double limelightHeight = 15.75;
    
    double limelightAngle = 11.6;

    double inchesToMeters = 0.0254;


};