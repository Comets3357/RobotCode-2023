#include "RobotData.h";

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData)
{
    // frc::SmartDashboard::PutNumber("odometry x place", table->GetNumber("tid", 0.0));

    if (robotData.controlData.saResetOdometry)
    {
        limelightOdometry.clear();
        limelightOdometry = table->GetNumberArray("botpose_wpiblue", std::vector<double>(6));

        tempX = limelightOdometry.at(0);
        tempY = limelightOdometry.at(1);

        if ((fabs(tempX - robotData.drivebaseData.odometryX) < 1) && (fabs(tempY - robotData.drivebaseData.odometryY) < 1))
        {
            distanceToClosestTag = GetDistance();

            if ((distanceToClosestTag < 2) || (distanceToClosestTag > 4 && distanceToClosestTag < 7))
            {
                limelightData.limelightOdometryX = tempX;
                limelightData.limelightOdometryY = tempY;
            }
        }
        
        frc::SmartDashboard::PutNumber("limelight x", limelightOdometry.at(0));
        frc::SmartDashboard::PutNumber("limelight y", limelightOdometry.at(1));
    }
}

double Limelight::GetDistance()
{
    double tempDist = 0;
    double verticalAngle = 0;

    verticalAngle = (table->GetNumber("ty", 0.0) + limelightAngle) * (pi / 180);
    tempDist = (aprilTagHeight - limelightHeight) / (std::tan(verticalAngle)) * inchesToMeters;

    return tempDist;
}