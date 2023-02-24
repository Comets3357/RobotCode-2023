#include "RobotData.h"

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData)
{
    // frc::SmartDashboard::PutNumber("odometry x place", table->GetNumber("tid", 0.0));

    // if (frc::DriverStation::IsEnabled())
    // {
    //     llresults = LimelightHelpers::getLatestResults();
    // }

    try 
    {
        llresults = LimelightHelpers::getLatestResults();
        distanceToClosestTag = GetDistance();
        frc::SmartDashboard::PutNumber("something useful", -distanceToClosestTag);
        frc::SmartDashboard::PutBoolean("limelight up", true);
        frc::SmartDashboard::PutNumber("ll latency", LimelightHelpers::getLatency_Pipeline());
    }
    catch (...)
    {
        frc::SmartDashboard::PutBoolean("limelight up", false);
    }
    
    if (robotData.controlData.saResetOdometry)
    {

        frc::SmartDashboard::PutBoolean("I am here cause limelight", true);
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
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
            {
                if ((numberOfTagsInView == 1) && (tempX > 13.3))
                {
                    limelightData.limelightOdometryY = tempY;
                    limelightData.limelightOdometryX = tempX;
                }
                else if ((numberOfTagsInView == 2) && (tempX > 9.5)) 
                {
                    limelightData.limelightOdometryY = tempY;
                    limelightData.limelightOdometryX = tempX;
                }
                else if (numberOfTagsInView == 3)
                {
                    limelightData.limelightOdometryY = tempY;
                    limelightData.limelightOdometryX = tempX;
                }
                else
                {
                    limelightData.limelightOdometryY = 100;
                    limelightData.limelightOdometryX = 100;
                }
            }
            else
            {
                if ((numberOfTagsInView == 1) && (tempX < 3.2))
                {
                    limelightData.limelightOdometryY = tempY;
                    limelightData.limelightOdometryX = tempX;
                }
                else if ((numberOfTagsInView == 2) && (tempX < 7.1)) 
                {
                    limelightData.limelightOdometryY = tempY;
                    limelightData.limelightOdometryX = tempX;
                }
                else if (numberOfTagsInView == 3)
                {
                    limelightData.limelightOdometryY = tempY;
                    limelightData.limelightOdometryX = tempX;
                }
                else
                {
                    limelightData.limelightOdometryY = 100;
                    limelightData.limelightOdometryX = 100;
                }
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
    tempDist = (aprilTagHeight - limelightHeight) / (std::tan(verticalAngle));// * inchesToMeters;

    frc::SmartDashboard::PutNumber("ty", LimelightHelpers::getTY(""));
    return tempDist;
}