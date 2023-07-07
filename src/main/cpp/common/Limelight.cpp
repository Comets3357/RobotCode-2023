#include "RobotData.h"

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData)
{

    try 
    {
        llresults = LimelightHelpers::getLatestResults();
        // distanceToClosestTag = GetDistance();
        frc::SmartDashboard::PutBoolean("limelight active", true);
    }
    catch (...)
    {
        frc::SmartDashboard::PutBoolean("limelight active", false);
    }
    
    if (robotData.controlData.saResetOdometry)
    {
        try
        {   
            LimelightHelpers::setPipelineIndex("pipeline", 0);

            limelightOdometry.clear();

            limelightOdometry = llresults.targetingResults.botPose_wpiblue;
            numberOfTagsInView = llresults.targetingResults.FiducialResults.size();
            limelightData.latency = LimelightHelpers::getLatency_Pipeline() + LimelightHelpers::getLatency_Capture();

            pastX = tempX;
            pastY = tempY;

            tempX = limelightOdometry.at(0);
            tempY = limelightOdometry.at(1);

            frc::SmartDashboard::PutNumber("ll x", tempX);
            frc::SmartDashboard::PutNumber("ll y", tempY);

            if ((fabs(tempX - robotData.drivebaseData.odometryX) < 1) && (fabs(tempY - robotData.drivebaseData.odometryY) < 1) 
            && (pastX != tempX) && (pastY != tempY))
            {
                if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
                {

                    gyroRadians = units::radian_t{robotData.gyroData.rawYaw / 180 * M_PI};
                    gyroRotation = frc::Rotation2d{gyroRadians};


                    if ((numberOfTagsInView == 1) && (tempX > 13.3))
                    {
                        limelightData.limelightAllowedToReset = true;
                        limelightData.Odometry = frc::Pose2d{units::meter_t{tempX}, units::meter_t{tempY}, gyroRotation};
                    }
                    else if ((numberOfTagsInView == 2) && (tempX > 9.5)) 
                    {
                        limelightData.limelightAllowedToReset = true;
                        limelightData.Odometry = frc::Pose2d{units::meter_t{tempX}, units::meter_t{tempY}, gyroRotation };
                    }
                    else if (numberOfTagsInView == 3)
                    {
                        limelightData.limelightAllowedToReset = true;
                        limelightData.Odometry = frc::Pose2d{units::meter_t{tempX}, units::meter_t{tempY}, gyroRotation};
                    }
                    else
                    {
                        limelightData.limelightAllowedToReset = false;
                    }
                }
                else
                {

                    gyroRadians = units::radian_t{robotData.gyroData.rawYaw / 180 * M_PI};
                    gyroRotation = frc::Rotation2d{gyroRadians};

                    if ((numberOfTagsInView == 1) && (tempX < 3.2))
                    {
                        limelightData.limelightAllowedToReset = true;
                        limelightData.Odometry = frc::Pose2d{units::meter_t{tempX}, units::meter_t{tempY}, gyroRotation};
                    }
                    else if ((numberOfTagsInView == 2) && (tempX < 7.1)) 
                    {
                        limelightData.limelightAllowedToReset = true;
                        limelightData.Odometry = frc::Pose2d{units::meter_t{tempX}, units::meter_t{tempY}, gyroRotation};
                    }
                    else if (numberOfTagsInView == 3)
                    {
                        limelightData.limelightAllowedToReset = true;
                        limelightData.Odometry = frc::Pose2d{units::meter_t{tempX}, units::meter_t{tempY}, gyroRotation};
                    }
                    else
                    {
                        limelightData.limelightAllowedToReset = false;
                    }
                }
            }
            else 
            {
                limelightData.limelightAllowedToReset = false;
            }
        }
        catch (...)
        {

        }
    }

    limelightData.pastExtendAllow = robotData.limelightData.allowExtend;
    if (!robotData.controlData.saResetOdometry)
    {
    try
    {

        //if (robotData.endEffectorData.gamePieceType != NONE)
        // {
        //     LimelightHelpers::setPipelineIndex("", 1);
        // }
        // else
        // {
        //     LimelightHelpers::setPipelineIndex("", 0);
        // }

        // if (frc::DriverStation::IsTeleop())
        // {
        //     LimelightHelpers::setPipelineIndex("", 3);
        // }

        // if (robotData.controlData.saPositionHigh && robotData.endEffectorData.lastPieceType == CONE)
        // {
        //     LimelightHelpers::setPipelineIndex("pipeline", 2);
        //     limelightData.x = LimelightHelpers::getTX("");
        // }
            
            limelightData.x = LimelightHelpers::getTX("") +0.0;
            limelightData.hasTarget = LimelightHelpers::getTA("") > 0.0;

            if (!frc::DriverStation::IsAutonomous())
            {

                if (robotData.controlData.saPositionHigh)
                {
                    LimelightHelpers::setPipelineIndex("", 2);
                    //LimelightHelpers::setLEDMode_ForceOn("");
                    //LimelightHelpers::setBrightness("", 100);
                }
                else if (robotData.controlData.saPositionMid)
                {
                    LimelightHelpers::setPipelineIndex("", 1);
                    //LimelightHelpers::setLEDMode_ForceOn("");
                    LimelightHelpers::setBrightness("", 20);
                }
                else if (!robotData.drivebaseData.autoAllign)
                {
                    //LimelightHelpers::setPipelineIndex("", 0);
                    //LimelightHelpers::setLEDMode_ForceOff("");
                    LimelightHelpers::setBrightness("", 0);
                }
                LimelightHelpers::setLEDMode_PipelineControl("");

            }
            else
            {
                LimelightHelpers::setPipelineIndex("", 0);
                LimelightHelpers::setLEDMode_ForceOff("");
            }

    }
    catch(...)
    {
        
    }
}


}