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
        int closestIndex = FindClosestIndex(timestamp);

        auto xError = visionPose.X() - history[closestIndex].pose.X();
        auto yError = visionPose.Y() - history[closestIndex].pose.Y();

        CorrectHistoryForError(xError, yError, closestIndex);

        newPose = history.back().pose;
    }
    else
    {
        // Don't have history to use to update esimate of current pose
        // ... default to trusting the vision pose
        newPose = visionPose;
    }

    poseEstimator->ResetPosition(newPose.Rotation(), lastLeftDistance, lastRightDistance, newPose);
}

void CustomDifferentialDriveOdometry::AddToHistory(frc::Pose2d& pose, 
                                                   units::second_t timestamp)
{
    CustomDifferentialDriveOdometry::HistoricalPose historicalPose{pose, timestamp};   
    history.emplace_back(historicalPose);

    if (history.size() > MAX_HISTORY_SIZE)
    {
        history.pop_front();
    }
}

int CustomDifferentialDriveOdometry::FindClosestIndex(units::second_t timestamp)
{
    double minDeltaAbs = 3357;
    int closestIndex = 0;

    for (int i = history.size() - 1; i >= 0; i--)
    {
        auto deltaAbs = std::abs((double)(history[i].timestamp - timestamp));
        if (deltaAbs < minDeltaAbs)
        {
            closestIndex = i;
            minDeltaAbs = deltaAbs;
        }
        else
        {
            break;
        }
    }

    return closestIndex;
}

void CustomDifferentialDriveOdometry::CorrectHistoryForError(units::meter_t xErr, 
                                                             units::meter_t yErr, 
                                                             int indexStart)
{
    for (int i = indexStart; i < history.size(); i++)
    {
        history[i].pose = frc::Pose2d{history[i].pose.X() + xErr, history[i].pose.Y() + yErr, history[i].pose.Rotation()};
    }
}