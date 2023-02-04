#include "RobotData.h";

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData)
{
    // frc::SmartDashboard::PutNumber("odometry x place", table->GetNumber("tid", 0.0));

    if (robotData.controlData.saResetOdometry)
    {
        limelightOdometry.clear();
        llresults = LimelightHelpers::getLatestResults();
        limelightOdometry = llresults.targetingResults.botPose_wpiblue;
        numberOfTagsInView = llresults.targetingResults.FiducialResults.size();

        tempX = limelightOdometry.at(0);
        tempY = limelightOdometry.at(1);

        frc::SmartDashboard::PutNumber("ll x", tempX);
        frc::SmartDashboard::PutNumber("ll y", tempY);

        if ((fabs(tempX - robotData.drivebaseData.odometryX) < 1) && (fabs(tempY - robotData.drivebaseData.odometryY) < 1))
        {
            distanceToClosestTag = GetDistance();

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

    verticalAngle = (LimelightHelpers::getTY() + limelightAngle) * (pi / 180);
    tempDist = (aprilTagHeight - limelightHeight) / (std::tan(verticalAngle)) * inchesToMeters;

    return tempDist;
}