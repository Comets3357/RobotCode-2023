#include "RobotData.h";

void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData)
{
    frc::SmartDashboard::PutNumber("odometry x place", table->GetNumber("tx", 0.0));

    // botPose = table->GetDoubleArrayTopic("MEGA");

    // botPose
}