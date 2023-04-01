#include "RobotData.h"

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData)
{

    try 
    {
        llresultsTwo = LimelightHelpers::getLatestResults("limelight-two");
        frc::SmartDashboard::PutBoolean("limelight one active", true);
    }
    catch (...)
    {
        frc::SmartDashboard::PutBoolean("limelight one active", false);
    }

    try 
    {
        llresultsOne = LimelightHelpers::getLatestResults("limelight");
        frc::SmartDashboard::PutBoolean("limelight two active", true);
    }
    catch (...)
    {
        frc::SmartDashboard::PutBoolean("limelight two active", false);
    }

    frc::SmartDashboard::PutBoolean("allowed to reset odom", limelightData.limelightAllowedToReset);
    frc::SmartDashboard::PutNumber("limelight x 1", llresultsOne.targetingResults.botPose_wpiblue.at(0));
    frc::SmartDashboard::PutNumber("limelight x 2", llresultsTwo.targetingResults.botPose_wpiblue.at(0));

    frc::SmartDashboard::PutNumber("FINAL LIMELIGHT X ODOM", tempX);
    frc::SmartDashboard::PutNumber("FINAL LIMELIGHT Y ODOM", tempY);

    frc::SmartDashboard::PutNumber("limelight id", limelightTwoID);

    try
    {
        if (robotData.controlData.saResetOdometry)
        {
            LimelightHelpers::setPipelineIndex("limelight-two", 0);
            LimelightHelpers::setPipelineIndex("limelight", 0);

            limelightOdometry.clear();

            limelightTwoID = LimelightHelpers::getFiducialID("limelight-two");
            limelightOneID = LimelightHelpers::getFiducialID("limelight");

            pastX = tempX;
            pastY = tempY;

            // red alliance
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
            {

                gyroRadians = units::radian_t{robotData.gyroData.rawYaw / 180 * M_PI};
                gyroRotation = frc::Rotation2d{gyroRadians};

                // robot facing with elevator limelight towards polls
                if (limelightOneID == 5 || limelightOneID == 6 || limelightOneID == 7 || limelightOneID == 8 ||
                    limelightTwoID == 1 || limelightTwoID == 2 || limelightTwoID == 3 || limelightTwoID == 4)
                {
                    // red side of field using elevator limelight
                    if (llresultsTwo.targetingResults.botPose_wpiblue.at(0) > 8.3)
                    {
                        limelightOdometry = llresultsTwo.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsTwo.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-two") + LimelightHelpers::getLatency_Pipeline("limelight-two");

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
                        limelightOdometry = llresultsOne.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsOne.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight") + LimelightHelpers::getLatency_Pipeline("limelight");

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
                    if (llresultsTwo.targetingResults.botPose_wpiblue.at(0) < 8.3)
                    {
                        limelightOdometry = llresultsTwo.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsTwo.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-two") + LimelightHelpers::getLatency_Pipeline("limelight-two");

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
                        limelightOdometry = llresultsOne.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsOne.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight") + LimelightHelpers::getLatency_Pipeline("limelight");

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
                
                gyroRadians = units::radian_t{robotData.gyroData.rawYaw / 180 * M_PI};
                gyroRotation = frc::Rotation2d{gyroRadians};

                // robot facing with elevator limelight towards polls
                if (limelightOneID == 1 || limelightOneID == 2 || limelightOneID == 3 || limelightOneID == 4 ||
                    limelightTwoID == 5 || limelightTwoID == 6 || limelightTwoID == 7 || limelightTwoID == 8)
                {
                    // red side of field using bull bar limelight
                    if (llresultsOne.targetingResults.botPose_wpiblue.at(0) > 8.3)
                    {
                        limelightOdometry = llresultsOne.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsOne.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight") + LimelightHelpers::getLatency_Pipeline("limelight");

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
                        limelightOdometry = llresultsTwo.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsTwo.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-two") + LimelightHelpers::getLatency_Pipeline("limelight-two");

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
                    if (llresultsOne.targetingResults.botPose_wpiblue.at(0) < 8.3)
                    {
                        limelightOdometry = llresultsOne.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsOne.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight") + LimelightHelpers::getLatency_Pipeline("limelight");

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
                        limelightOdometry = llresultsTwo.targetingResults.botPose_wpiblue;
                        numberOfTagsInView = llresultsTwo.targetingResults.FiducialResults.size();
                        limelightData.latency = LimelightHelpers::getLatency_Capture("limelight-two") + LimelightHelpers::getLatency_Pipeline("limelight-two");

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
        //LimelightHelpers::setPipelineIndex("limelight-two", 2);

        // limelightData.x = LimelightHelpers::getTX("limelight-two");
        // if (robotData.drivebaseData.autoAllign)
        // {
        //     if (!limelightData.cantSeeTop)
        //     {
        //         LimelightHelpers::setPipelineIndex("limelight-two", 2);
        //     }
            
        //     limelightData.x = LimelightHelpers::getTX("limelight-two");

        //     if (!LimelightHelpers::getLimelightNTTableEntry("limelight-two", "tv"))
        //     {
        //         LimelightHelpers::setPipelineIndex("limelight-two", 1);
        //         limelightData.cantSeeTop = true;
        //     }
        // }
        // else
        // {
        //     limelightData.cantSeeTop = false;
        // }
    }
    catch(...)
    {
        
    }


}