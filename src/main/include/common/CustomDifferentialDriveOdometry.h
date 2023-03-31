#pragma once

#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <deque>
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

    const int MAX_HISTORY_SIZE = 50;

    struct HistoricalPose
    {
        frc::Pose2d pose;
        units::second_t timestamp;
    };

    std::deque<HistoricalPose> history;

    units::meter_t lastRightDistance;
    units::meter_t lastLeftDistance;

    frc::DifferentialDrivePoseEstimator *poseEstimator;

    void AddToHistory(frc::Pose2d& pose, 
                      units::second_t timestamp);

    int FindClosestIndex(units::second_t timestamp);

    void CorrectHistoryForError(units::meter_t xErr, 
                                units::meter_t yErr, 
                                int indexStart);

};