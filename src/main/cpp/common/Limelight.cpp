#include "RobotData.h"

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData)
{

    try 
    {
        llresultsOne = LimelightHelpers::getLatestResults("limelight-1");
        frc::SmartDashboard::PutBoolean("limelight one active", true);
    }
    catch (...)
    {
        frc::SmartDashboard::PutBoolean("limelight one active", false);
    }

    try 
    {
        llresultsTwo = LimelightHelpers::getLatestResults("limelight-2");
        frc::SmartDashboard::PutBoolean("limelight two active", true);
    }
    catch (...)
    {
        frc::SmartDashboard::PutBoolean("limelight two active", false);
    }

    try
    {
        if (robotData.controlData.saResetOdometry)
        {
            LimelightHelpers::setPipelineIndex("limelight-1", 0);
            LimelightHelpers::setPipelineIndex("limelight-2", 0);

            limelightOdometry.clear();

            gyroRadians = units::radian_t{robotData.gyroData.rawYaw / 180 * M_PI};
            gyroRotation = frc::Rotation2d{gyroRadians + units::radian_t{M_PI}};

            limelightOneID = LimelightHelpers::getFiducialID("limelight-1");
            limelightTwoID = LimelightHelpers::getFiducialID("limelight-2");

            pastX = tempX;
            pastY = tempY;

            // red alliance
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
            {

                // robot facing with elevator limelight towards polls
                if (limelightOneID == 1 || limelightOneID == 2 || limelightOneID == 3 || limelightOneID == 4 ||
                    limelightTwoID == 5 || limelightTwoID == 6 || limelightTwoID == 7 || limelightTwoID == 8)
                {
                    // red side of field using elevator limelight
                    if (robotData.drivebaseData.odometryX > 8.3)
                    {
                        limelightOdometry = llresultsOne.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsOne.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-1") + LimelightHelpers::getLatency_Pipeline("limelight-1");

                        tempX = limelightOdometry.at(0);
                        tempY = limelightOdometry.at(1);

                        if ((((numberOfTagsInView == 1) && (tempX > 13.3)) ||
                            ((numberOfTagsInView == 2) && (tempX > 9.5)) ||
                            (numberOfTagsInView == 3)) && (pastX != tempX))
                        {
                            limelightData.limelightAllowedToReset = true;
                            limelightData.Odometry = frc::Pose2d{units::meter_t{tempX}, units::meter_t{tempY}, gyroRotation};
                        }
                        else
                        {
                            limelightData.limelightAllowedToReset = false;
                        }
                    }
                    else // blue side of field using bull bar limelight
                    {
                        limelightOdometry = llresultsTwo.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsTwo.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-2") + LimelightHelpers::getLatency_Pipeline("limelight-2");

                        tempX = limelightOdometry.at(0);
                        tempY = limelightOdometry.at(1);    

                        if ((((numberOfTagsInView == 1) && (tempX < 3.3)) ||
                            ((numberOfTagsInView == 2) && (tempX < 7.3)) ||
                            (numberOfTagsInView == 3)) && (pastX != tempX))
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
                else // robot facing with elevator limelight away from polls
                {
                    // blue side of field elevator limelight
                    if (robotData.drivebaseData.odometryX < 8.3)
                    {
                        limelightOdometry = llresultsOne.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsOne.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-1") + LimelightHelpers::getLatency_Pipeline("limelight-1");

                        tempX = limelightOdometry.at(0);
                        tempY = limelightOdometry.at(1);    

                        if ((((numberOfTagsInView == 1) && (tempX < 3.3)) ||
                            ((numberOfTagsInView == 2) && (tempX < 7.3)) ||
                            (numberOfTagsInView == 3)) && (pastX != tempX))
                        {
                            limelightData.limelightAllowedToReset = true;
                            limelightData.Odometry = frc::Pose2d{units::meter_t{tempX}, units::meter_t{tempY}, gyroRotation};
                        }
                        else
                        {
                            limelightData.limelightAllowedToReset = false;
                        }
                    }
                    else // red side of field using bull bar limelight
                    {
                        limelightOdometry = llresultsTwo.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsTwo.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-2") + LimelightHelpers::getLatency_Pipeline("limelight-2");

                        tempX = limelightOdometry.at(0);
                        tempY = limelightOdometry.at(1);    

                        if ((((numberOfTagsInView == 1) && (tempX > 13.3)) ||
                            ((numberOfTagsInView == 2) && (tempX > 9.5)) ||
                            (numberOfTagsInView == 3)) && (pastX != tempX))
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
            }
            else // blue alliance
            {
                // robot facing with elevator limelight towards polls
                if (limelightOneID == 5 || limelightOneID == 6 || limelightOneID == 7 || limelightOneID == 8 ||
                    limelightTwoID == 1 || limelightTwoID == 2 || limelightTwoID == 3 || limelightTwoID == 4)
                {
                    // red side of field using bull bar limelight
                    if (robotData.drivebaseData.odometryX > 8.3)
                    {
                        limelightOdometry = llresultsTwo.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsTwo.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-2") + LimelightHelpers::getLatency_Pipeline("limelight-2");

                        tempX = limelightOdometry.at(0);
                        tempY = limelightOdometry.at(1);

                        if ((((numberOfTagsInView == 1) && (tempX > 13.3)) ||
                            ((numberOfTagsInView == 2) && (tempX > 9.5)) ||
                            (numberOfTagsInView == 3)) && (pastX != tempX))
                        {
                            limelightData.limelightAllowedToReset = true;
                            limelightData.Odometry = frc::Pose2d{units::meter_t{tempX}, units::meter_t{tempY}, gyroRotation};
                        }
                        else
                        {
                            limelightData.limelightAllowedToReset = false;
                        }
                    }
                    else // blue side of field using elevator limelight
                    {
                        limelightOdometry = llresultsOne.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsOne.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-1") + LimelightHelpers::getLatency_Pipeline("limelight-1");

                        tempX = limelightOdometry.at(0);
                        tempY = limelightOdometry.at(1);    

                        if ((((numberOfTagsInView == 1) && (tempX < 3.3)) ||
                            ((numberOfTagsInView == 2) && (tempX < 7.3)) ||
                            (numberOfTagsInView == 3)) && (pastX != tempX))
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
                else // robot facing with elevator limelight away from polls
                {
                    // blue side of field using bull bar limelight
                    if (robotData.drivebaseData.odometryX < 8.3)
                    {
                        limelightOdometry = llresultsTwo.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsTwo.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-2") + LimelightHelpers::getLatency_Pipeline("limelight-2");

                        tempX = limelightOdometry.at(0);
                        tempY = limelightOdometry.at(1);    

                        if ((((numberOfTagsInView == 1) && (tempX < 3.3)) ||
                            ((numberOfTagsInView == 2) && (tempX < 7.3)) ||
                            (numberOfTagsInView == 3)) && (pastX != tempX))
                        {
                            limelightData.limelightAllowedToReset = true;
                            limelightData.Odometry = frc::Pose2d{units::meter_t{tempX}, units::meter_t{tempY}, gyroRotation};
                        }
                        else
                        {
                            limelightData.limelightAllowedToReset = false;
                        }
                    }
                    else // red side of field using elevator limelight
                    {
                        limelightOdometry = llresultsOne.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsOne.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-1") + LimelightHelpers::getLatency_Pipeline("limelight-1");

                        tempX = limelightOdometry.at(0);
                        tempY = limelightOdometry.at(1);    

                        if ((((numberOfTagsInView == 1) && (tempX > 13.3)) ||
                            ((numberOfTagsInView == 2) && (tempX > 9.5)) ||
                            (numberOfTagsInView == 3)) && (pastX != tempX))
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
            }
        }
    }
    catch(...)
    {

    }
    
    try
    {
        if (robotData.controlData.saPositionHigh || robotData.controlData.saPositionMid)
        {
            LimelightHelpers::setPipelineIndex("limelight-1", 2);
            limelightData.x = LimelightHelpers::getTX("limelight-1");
        }
    }
    catch(...)
    {
        
    }


}