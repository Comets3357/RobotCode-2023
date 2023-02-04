#include "RobotData.h"

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData)
{
    // frc::SmartDashboard::PutNumber("odometry x place", table->GetNumber("tid", 0.0));
    
    llresults = LimelightHelpers::getLatestResults();
    frc::SmartDashboard::PutNumber("I AM HERE", 0000000000000000010000000000);
    distanceToClosestTag = GetDistance();
    frc::SmartDashboard::PutNumber("something useful", -distanceToClosestTag / inchesToMeters);
    if (robotData.controlData.saResetOdometry)
    {
        limelightOdometry.clear();
        limelightOdometry = llresults.targetingResults.botPose_wpiblue;
        numberOfTagsInView = llresults.targetingResults.FiducialResults.size();

        tempX = limelightOdometry.at(0);
        tempY = limelightOdometry.at(1);

        frc::SmartDashboard::PutNumber("ll x", tempX);
        frc::SmartDashboard::PutNumber("ll y", tempY);

        limelightData.limelightPastOdometryX = limelightData.limelightOdometryX;
        limelightData.limelightPastOdometryY = limelightData.limelightOdometryY;

        if ((fabs(tempX - robotData.drivebaseData.odometryX) < 1) && (fabs(tempY - robotData.drivebaseData.odometryY) < 1))
        {
            

            if (((distanceToClosestTag < 2) && (numberOfTagsInView >= 1)) || 
                ((distanceToClosestTag > 4 && distanceToClosestTag < 7) && (numberOfTagsInView >= 2)))
            {
                limelightData.limelightOdometryX = tempX;
                limelightData.limelightOdometryY = tempY;
            }
            else
            {
                limelightData.limelightOdometryX = 100;
                limelightData.limelightOdometryY = 100;
            }
        }
        else 
        {
            limelightData.limelightOdometryX = 100;
            limelightData.limelightOdometryY = 100;
        }
    }
}

double Limelight::GetDistance()
{
    double tempDist = 0;
    double verticalAngle = 0;

    verticalAngle = (LimelightHelpers::getTY("") + limelightAngle) * (pi / 180);
    tempDist = (aprilTagHeight - limelightHeight) / (std::tan(verticalAngle)) * inchesToMeters;

    frc::SmartDashboard::PutNumber("ty", LimelightHelpers::getTY(""));
    return tempDist;
}