#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SPI.h>
#include "AHRS.h"

struct GyroData
{
    double rawYaw = 0;
    double rawPitch = 0;
    double rawRoll = 0;
    double angularMomentum;
    double rotationalRate;

    double velocity = 0;

    frc::Rotation2d gyroRot2d;
};

class Gyro
{

public:
    void RobotInit();
    void AutonomousInit(GyroData &gyroData);
    void TeleopInit(GyroData &gyroData);
    void RobotPeriodic(GyroData &gyroData);

private:
    AHRS gyro{frc::SPI::Port::kMXP};

};