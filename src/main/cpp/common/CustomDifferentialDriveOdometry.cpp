#include "common/CustomDifferentialDriveOdometry.h"

void CustomDifferentialDriveOdometry::SetPoseEstimator(frc::DifferentialDrivePoseEstimator &odometry)
{
    poseEstimator = &odometry;
}

frc::Pose2d CustomDifferentialDriveOdometry::GetEstimatedPosition() 
{
    return poseEstimator->GetEstimatedPosition();
}

frc::Pose2d CustomDifferentialDriveOdometry::UpdateWithTime(units::second_t currentTime,
                                                      const frc::Rotation2d& gyroAngle,
                                                      units::meter_t leftDistance,
                                                      units::meter_t rightDistance)
{
    auto newPose = poseEstimator->UpdateWithTime(currentTime, gyroAngle, leftDistance, rightDistance);
    AddToHistory(newPose, currentTime);
    lastRightDistance = rightDistance;
    lastLeftDistance = leftDistance;
    return newPose;
}

void CustomDifferentialDriveOdometry::SeedWithVisionMeasurement(const frc::Pose2d& visionPose,
                                                          units::second_t timestamp)
{
    frc::Pose2d newPose;

    if (history.size() > 0)
    {
        // find the historical pose with closest timestamp
        double minDeltaAbs = 3357;
        int closestIndex = 0;
        for (int i = 0; i < history.size(); i++)
        {
            auto deltaAbs = std::abs((double)(history[i].timestamp - timestamp));
            if (deltaAbs < minDeltaAbs)
            {
                closestIndex = i;
                minDeltaAbs = deltaAbs;
            }
        }

        // find the relative transform between this pose and latest pose
        auto xTransform = history.back().pose.X() - history[closestIndex].pose.X();
        auto yTransform = history.back().pose.Y() - history[closestIndex].pose.Y();

        // Apply transform to vision pose
        auto xPos = visionPose.X() + xTransform;
        auto yPos = visionPose.Y() + yTransform;

        frc::Pose2d seededPose(xPos, yPos, visionPose.Rotation());

        newPose = seededPose;

        // clear the history in effort to keep the memory usage down
        history.clear();
    }
    else
    {
        // Don't have history to use to update esimate of current pose
        // ... default to trusting the vision pose
        newPose = visionPose;
    }

    poseEstimator->ResetPosition(newPose.Rotation(), lastLeftDistance, lastRightDistance, newPose);
}

void CustomDifferentialDriveOdometry::AddToHistory(frc::Pose2d& pose, units::second_t timestamp)
{
    CustomDifferentialDriveOdometry::HistoricalPose historicalPose{pose, timestamp};
    history.emplace_back(historicalPose);
}