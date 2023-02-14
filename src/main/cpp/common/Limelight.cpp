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

        // if ((fabs(tempX - robotData.drivebaseData.odometryX) < 1) && (fabs(tempY - robotData.drivebaseData.odometryY) < 1))
        // {
            

            // if (((distanceToClosestTag < 40) && (numberOfTagsInView >= 1)) || 
            //     ((distanceToClosestTag > 40 && distanceToClosestTag < 80) && (numberOfTagsInView >= 2)))
            // {
            //     limelightData.limelightOdometryX = tempX;
            //     limelightData.limelightOdometryY = tempY;
            // }
            // else
            // {
            //     limelightData.limelightOdometryX = 100;
            //     limelightData.limelightOdometryY = 100;
            // }

            if ((numberOfTagsInView == 1) && tempX < 3.5)
            {
                limelightData.limelightOdometryX = tempX;
                limelightData.limelightOdometryY = tempY;  
            }
            else if (numberOfTagsInView == 2)
            {
                limelightData.limelightOdometryX = tempX;
                limelightData.limelightOdometryY = tempY;
            }
            else
            {
                limelightData.limelightOdometryX = 100;
                limelightData.limelightOdometryY = 100;
            }
        // }
        // else 
        // {
        //     limelightData.limelightOdometryX = 100;
        //     limelightData.limelightOdometryY = 100;
        // }
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