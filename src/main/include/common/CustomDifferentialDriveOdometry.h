#pragma once

#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <vector>
#include <cmath>

// This class utilizes the differential drive pose estimator to implement functionality
class CustomDifferentialDriveOdometry 
{
public:
    CustomDifferentialDriveOdometry() = default;

    ~CustomDifferentialDriveOdometry() = default;

    void SetPoseEstimator(frc::DifferentialDrivePoseEstimator &odometry);

    frc::Pose2d GetEstimatedPosition();
    
    frc::Pose2d UpdateWithTime(units::second_t curentTime,
                               const frc::Rotation2d& gyroAngle,
                               units::meter_t leftDistance,
                               units::meter_t rightDistance);

    // Still expects a UpdateWithTime to be called afterwards
    void SeedWithVisionMeasurement(const frc::Pose2d& visionPose,
                                   units::second_t timestamp);

private:

    struct HistoricalPose
    {
        frc::Pose2d pose;
        units::second_t timestamp;
    };

    std::vector<HistoricalPose> history;

    units::meter_t lastRightDistance;
    units::meter_t lastLeftDistance;

    frc::DifferentialDrivePoseEstimator* poseEstimator;

    void AddToHistory(frc::Pose2d& pose, units::second_t timestamp);

};
