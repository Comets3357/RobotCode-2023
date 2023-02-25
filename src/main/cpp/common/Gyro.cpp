#include "common/Gyro.h"
#include "frc/DriverStation.h"

void Gyro::RobotInit() {
    //gyro.Calibrate();
    gyro.ZeroYaw();
    // if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
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

    // frc::SmartDashboard::PutNumber("angleMomentum",gyroData.rotationalRate);
    frc::SmartDashboard::PutNumber("yaw",gyro.GetYaw());
    // frc::smartDashboard::PutNumber("pitch",gyroData.rawPitch);
    // frc::smartDashboard::PutNumber("roll",gyroData.rawRoll);

    frc::SmartDashboard::PutNumber("rawYaw", gyroData.rawYaw);
    frc::SmartDashboard::PutNumber("rawPitch", gyroData.rawPitch);
    frc::SmartDashboard::PutNumber("rawRoll", gyroData.rawRoll);

    // frc::SmartDashboard::PutNumber("")

}
