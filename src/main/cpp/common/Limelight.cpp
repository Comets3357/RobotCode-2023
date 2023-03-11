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
                    gyroRotation = frc::Rotation2d{gyroRadians + units::radian_t{M_PI}};


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

    try
    {
        if (robotData.controlData.mode == MODE_TELEOP_ADVANCED_SA)
        {
            if (robotData.controlData.saPositionHigh && robotData.endEffectorData.gamePieceType == CONE)
            {
                LimelightHelpers::setPipelineIndex("pipeline", 2);
                distanceFromTarget = GetDistance(higherPollHeight, upperAngleOffset);
                angleOff = LimelightHelpers::getTX("") * (pi / 180);

                if (std::abs(angleOff) < 18)
                {
                    distanceFromCenterOfRobot = std::sqrt((std::pow((distanceFromTarget * std::sin(angleOff)), 2)) + (std::pow(((distanceFromTarget * std::cos(angleOff)) + cameraDistanceFromCenter), 2)));
                    angleFromCenterOfRobot = std::atan((distanceFromTarget * std::sin(angleOff)) / ((distanceFromTarget * std::cos(angleOff)) + cameraDistanceFromCenter));

                    secondAngleFromCenter = std::atan((robotData.endEffectorData.gamePieceDistance) / (highEndEffectorDistanceFromCenter));

                    
                    frc::SmartDashboard::PutNumber("distance from center", distanceFromCenterOfRobot);
                    frc::SmartDashboard::PutNumber("angle from center", angleFromCenterOfRobot);
                    frc::SmartDashboard::PutNumber("second angle from center", secondAngleFromCenter);

                    if (angleOff > 0 && robotData.endEffectorData.gamePieceDistance > 0)
                    {
                        finalAngle = angleFromCenterOfRobot - secondAngleFromCenter;
                    }
                    else if (angleOff < 0 && robotData.endEffectorData.gamePieceDistance > 0)
                    {
                        finalAngle = -(std::abs(angleFromCenterOfRobot) + secondAngleFromCenter);
                    }
                    else if (angleOff > 0 && robotData.endEffectorData.gamePieceDistance < 0) 
                    {
                        finalAngle = angleFromCenterOfRobot + std::abs(secondAngleFromCenter);
                    }
                    else if (angleOff < 0 && robotData.endEffectorData.gamePieceDistance < 0)
                    {
                        finalAngle = angleFromCenterOfRobot - secondAngleFromCenter;
                    }    else
                    {
                        finalAngle = angleFromCenterOfRobot;
                    }

                    //limelightData.angleOffFromCenter = finalAngle * (180 / pi);
                    limelightData.angleOffFromCenter = angleOff * (180 / pi);
                    limelightData.distanceFromCenter = -(distanceFromCenterOfRobot - highEndEffectorDistanceFromCenter) * inchesToMeters;

                    // if (std::abs(limelightData.distanceFromCenter) < 0.2)
                    // {
                    //     limelightData.allowExtend = true;
                    // }
                    // else
                    // {
                    //     limelightData.allowExtend = false;
                    // }
                }
                else
                {
                    LimelightHelpers::setPipelineIndex("pipeline", 1);

                    distanceFromTarget = GetDistance(lowerPollHeight, lowerAngleOffset) + distanceBetweenMidAndTopPoll;
                    angleOff = LimelightHelpers::getTX("") * (pi / 180);

                    distanceFromCenterOfRobot = std::sqrt((std::pow((distanceFromTarget * std::sin(angleOff)), 2)) + (std::pow(((distanceFromTarget * std::cos(angleOff)) + cameraDistanceFromCenter), 2)));
                    angleFromCenterOfRobot = std::atan((distanceFromTarget * std::sin(angleOff)) / ((distanceFromTarget * std::cos(angleOff)) + cameraDistanceFromCenter));

                    secondAngleFromCenter = std::atan((robotData.endEffectorData.gamePieceDistance) / (highEndEffectorDistanceFromCenter));

                    if (angleOff > 0 && robotData.endEffectorData.gamePieceDistance > 0)
                    {
                        finalAngle = angleFromCenterOfRobot - secondAngleFromCenter;
                    }
                    else if (angleOff < 0 && robotData.endEffectorData.gamePieceDistance > 0)
                    {
                        finalAngle = -(std::abs(angleFromCenterOfRobot) + secondAngleFromCenter);
                    }
                    else if (angleOff > 0 && robotData.endEffectorData.gamePieceDistance < 0) 
                    {
                        finalAngle = angleFromCenterOfRobot + std::abs(secondAngleFromCenter);
                    }
                    else if (angleOff < 0 && robotData.endEffectorData.gamePieceDistance < 0)
                    {
                        finalAngle = angleFromCenterOfRobot - secondAngleFromCenter;
                    }
                    else
                    {
                        finalAngle = angleFromCenterOfRobot;
                    }

                    limelightData.angleOffFromCenter = finalAngle * (180 / pi);
                    limelightData.distanceFromCenter = (distanceFromCenterOfRobot - highEndEffectorDistanceFromCenter) * inchesToMeters;
                }


            }
            else if (robotData.controlData.saPositionMid && robotData.endEffectorData.gamePieceType == CONE)
            {
                LimelightHelpers::setPipelineIndex("pipeline", 1);

                distanceFromTarget = GetDistance(lowerPollHeight, lowerAngleOffset);
                angleOff = LimelightHelpers::getTX("") * (pi / 180);

                distanceFromCenterOfRobot = std::sqrt((std::pow((distanceFromTarget * std::sin(angleOff)), 2)) + (std::pow(((distanceFromTarget * std::cos(angleOff)) + cameraDistanceFromCenter), 2)));
                angleFromCenterOfRobot = std::atan((distanceFromTarget * std::sin(angleOff)) / ((distanceFromTarget * std::cos(angleOff)) + cameraDistanceFromCenter));

                secondAngleFromCenter = std::atan((robotData.endEffectorData.gamePieceDistance) / (midEndEffectorDistanceFromCenter));

                if (angleOff > 0 && robotData.endEffectorData.gamePieceDistance > 0)
                {
                    finalAngle = angleFromCenterOfRobot - secondAngleFromCenter;
                }
                else if (angleOff < 0 && robotData.endEffectorData.gamePieceDistance > 0)
                {
                    finalAngle = -(std::abs(angleFromCenterOfRobot) + secondAngleFromCenter);
                }
                else if (angleOff > 0 && robotData.endEffectorData.gamePieceDistance < 0) 
                {
                    finalAngle = angleFromCenterOfRobot + std::abs(secondAngleFromCenter);
                }
                else if (angleOff < 0 && robotData.endEffectorData.gamePieceDistance < 0)
                {
                    finalAngle = angleFromCenterOfRobot - secondAngleFromCenter;
                }
                else
                {
                    finalAngle = angleFromCenterOfRobot;
                }

                limelightData.angleOffFromCenter = finalAngle* (180 / pi);
                limelightData.distanceFromCenter = (distanceFromCenterOfRobot - midEndEffectorDistanceFromCenter) * inchesToMeters;
                
            }
            else if ((robotData.controlData.saPositionHigh || robotData.controlData.saPositionMid) && robotData.endEffectorData.gamePieceType == CUBE)
            {
                LimelightHelpers::setPipelineIndex("pipeline", 0);

                distanceFromTarget = 1000;

                if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
                {
                    if (llresults.targetingResults.botPose_wpiblue.at(0) > 14.8)
                    {
                        limelightData.allowExtend = true;
                    }
                    {
                        limelightData.allowExtend = false;
                    }
                }
                else
                {
                    if (llresults.targetingResults.botPose_wpiblue.at(0) < 1.7)
                    {
                        limelightData.allowExtend = true;
                    }
                    else
                    {
                        limelightData.allowExtend = false;
                    }
                }
            }
        }
        
    }
    catch(...)
    {
        
    }

    frc::SmartDashboard::PutNumber("distance", limelightData.distanceFromCenter);
    frc::SmartDashboard::PutNumber("angle off", limelightData.angleOffFromCenter);
    frc::SmartDashboard::PutBoolean("allow extend", limelightData.allowExtend);


}

double Limelight::GetDistance(double targetHeight, double vertAngleOffset)
{
    double tempDist = 0;
    double verticalAngle = 0;

    verticalAngle = (LimelightHelpers::getTY("") + (limelightAngle + vertAngleOffset)) * (pi / 180);
    tempDist = (targetHeight - limelightHeight) / (std::tan(verticalAngle));// * inchesToMeters;

    frc::SmartDashboard::PutNumber("ty", LimelightHelpers::getTY(""));
    return tempDist;
}