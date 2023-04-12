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
    bool hasTarget = false;;
};

class Limelight
{
public:
    void RobotPeriodic(const RobotData &robotData, LimelightData &limelightData);

private:

    double GetDistance(double targetHeight, double vertAngleOffset);

    // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); 
    LimelightHelpers::LimelightResultsClass llresults; //= LimelightHelpers::getLatestResults();
    std::vector<double> limelightOdometry = {0,0,0,0,0,0};


    double tempX = 0;
    double tempY = 0;

    double pastX = 0;
    double pastY = 0;

    int numberOfTagsInView = 0;

    double distanceFromTarget = 0;
    double angleOff = 0;

    double aprilTagHeight = 13.0;
    double lowerPollHeight = 22.125; // need to tune
    double higherPollHeight = 41.875; // need to tune

    double limelightHeight = 11.03;

    double cameraDistanceFromCenter = 11.84;
    double midEndEffectorDistanceFromCenter = 13 + 12; // need to tune
    double highEndEffectorDistanceFromCenter = 13 + 38; // need to tune
    
    double limelightAngle = 25;

    // 63.3 x 49.7
    double upperAngleOffset = 12.66;
    double lowerAngleOffset = 0;

    double inchesToMeters = 0.0254;

    double distanceFromCenterOfRobot = 0;
    double angleFromCenterOfRobot = 0;

    double secondAngleFromCenter = 0;;
    double finalAngle;

    units::radian_t gyroRadians{0};
    frc::Rotation2d gyroRotation{gyroRadians};

    double distanceBetweenMidAndTopPoll = 17;




};