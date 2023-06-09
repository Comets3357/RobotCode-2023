#include "common/Gyro.h"
#include "frc/DriverStation.h"

void Gyro::RobotInit() {
    gyro.ZeroYaw();
    gyro.SetAngleAdjustment(0);
}

void Gyro::TeleopInit(GyroData &gyroData) {
    gyro.ZeroYaw();
    
    // gyroData.rawYaw = 0;
    // gyroData.rawPitch = 0;
    // gyroData.rawRoll = 0;
}

void Gyro::AutonomousInit(GyroData &gyroData) {
    gyro.ZeroYaw();
    gyroData.rawYaw = 0;
    gyroData.rawPitch = 0;
    gyroData.rawRoll = 0;
    gyroData.angularMomentum = 0;
}

void Gyro::RobotPeriodic(GyroData &gyroData) {

    gyroData.rawYaw = -gyro.GetAngle();
    gyroData.rawPitch = gyro.GetPitch();
    gyroData.rawRoll = gyro.GetRoll();
    gyroData.angularMomentum = gyro.GetRawGyroY();
    gyroData.rotationalRate = gyro.GetRawGyroZ();
    frc::SmartDashboard::PutNumber("GYRO VELOCITY Z", gyro.GetVelocityZ());
    frc::SmartDashboard::PutNumber("GYRO VELOCITY Y", gyro.GetVelocityY());
    frc::SmartDashboard::PutNumber("GYRO VELOCITY X", gyro.GetVelocityX());

    frc::SmartDashboard::PutNumber("angleMomentum",gyroData.angularMomentum);
    frc::SmartDashboard::PutNumber("yaw",gyro.GetYaw());
    // frc::smartDashboard::PutNumber("pitch",gyroData.rawPitch);
    // frc::smartDashboard::PutNumber("roll",gyroData.rawRoll);

    frc::SmartDashboard::PutNumber("rawYaw", gyroData.rawYaw);
    frc::SmartDashboard::PutNumber("rawPitch", gyroData.rawPitch);
    frc::SmartDashboard::PutNumber("rawRoll", gyroData.rawRoll);

    // frc::SmartDashboard::PutNumber("")

}
