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

    bool limelightAllowedToReset = false;
    double latency = 0;
    
    frc::Pose2d Odometry;

    bool allowExtend = false;
    bool pastExtendAllow = false;

    double angleOffFromCenter = 0;
    double distanceFromCenter = 0;

    double x;
    double y;
};

class Limelight
{
public:
    void RobotPeriodic(const RobotData &robotData, LimelightData &limelightData);

private:

    double GetDistance(double targetHeight, double vertAngleOffset);

    // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); 
    LimelightHelpers::LimelightResultsClass llresultsOne; //= LimelightHelpers::getLatestResults();
    LimelightHelpers::LimelightResultsClass llresultsTwo; 
    std::vector<double> limelightOdometry = {0,0,0,0,0,0};


    double tempX = 0;
    double tempY = 0;

    double pastX = 0;
    double pastY = 0;

    int numberOfTagsInView = 0;

    // 63.3 x 49.7

    units::radian_t gyroRadians{0};
    frc::Rotation2d gyroRotation{gyroRadians};

    int limelightOneID = 0;
    int limelightTwoID = 0;




};