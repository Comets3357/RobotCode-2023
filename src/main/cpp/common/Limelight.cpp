#include "RobotData.h";

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData)
{
    // frc::SmartDashboard::PutNumber("odometry x place", table->GetNumber("tid", 0.0));

    limelightOdometry.clear();

    limelightOdometry = table->GetNumberArray("botpose_wpiblue", std::vector<double>(6));

    frc::SmartDashboard::PutNumber("limelight x", limelightOdometry.at(0));

    frc::SmartDashboard::PutNumber("limelight y", limelightOdometry.at(1));

    // botPose = table->GetDoubleArrayTopic("MEGA");

    // botPose
}