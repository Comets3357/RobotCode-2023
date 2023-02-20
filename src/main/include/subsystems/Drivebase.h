#pragma once

#include "Constants.h"
#include "auton/Auton.h"
#include "common/Gyro.h"

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
// #include <wpi/uv/Error.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/controller/RamseteController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <deque>

#define M_PI 3.14159265358979323846

struct RobotData;

enum DriveMode {
    DRIVEMODE_JOYSTICK,
    DRIVEMODE_TURNINPLACE,
    DRIVEMODE_BREAK,
    DRIVEMODE_TRAJECTORY,
    DRIVEMODE_VECTOR,
    DRIVEMODE_CHARGE_STATION_TRAVERSE,
    DRIVEMODE_AUTO_BALANCE
};

struct DrivebaseData
{
    DriveMode driveMode = DRIVEMODE_BREAK;

     // in meters eventually
    double currentLDBPos = 0.0; // ticks
    double currentRDBPos = 0.0; // ticks

    double lDriveVel = 0.0; // meters per second
    double rDriveVel = 0.0; // meters per second
    double avgDriveVel = 0.0;   // meters per second
    bool dbStationaryForShot = false;   // used to determine if robot is stationary enough to fire

    // odometry
    frc::Pose2d currentPose{};
    frc::Pose2d shooterRejectTestPose{};
    double odometryX;
    double odometryY;
    double odometryYaw;

    double turretEjectAngle;
};

class Drivebase
{

public:
    void RobotInit(const RobotData &robotData);
    void TeleopInit(const RobotData &robotData);
    void AutonomousInit(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData);
    void RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData, GyroData &gyroData, ControlData &controlData);
    void TestPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData);
    void DisabledInit();
    void DisabledPeriodic();

private:

    double GetTurnMax();
    void updateData(const RobotData &robotData, DrivebaseData &drivebaseData);
    void teleopControl(const RobotData &robotData, DrivebaseData &drivebaseData, GyroData &gyroData, ControlData &controlData);
    void autonControl(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData, GyroData &gyroData);

    // odometry
    void updateOdometry(const RobotData &robotData, DrivebaseData &drivebaseData);
    void resetOdometry(const frc::Pose2d &pose, double resetAngle);
    void resetOdometry(double x, double y, double radians, const RobotData &robotData);
    // void resetOdometry(double x, double y, double tanX, double tanY, const RobotData &robotData);
    void zeroEncoders();
    
    void calcTurretEjectAngle(DrivebaseData &drivebaseData);

    void sendStartPointChooser();

    void setVelocity(double leftVel, double rightVel);
    void setPercentOutput(double leftVBus, double rightVBus);

    frc::Pose2d getPose(double x, double y, double deg);

    void getNextAutonStep(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData);

    void turnInPlaceAuton(double degrees, const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData);
    void turnInPlaceTeleop(double degrees, const RobotData &robotData);

    bool allValuesWithin(std::deque<double> deque, double tolerance);

    double getEncoderDistance(double encoderPosition);


    frc::SendableChooser<frc::Pose2d> startPointChooser;

    const units::radian_t kZeroAngle{0.0};
    units::meter_t meterX{3.167};
    units::meter_t meterY{7.492};
    const frc::Translation2d testTrans{meterX, meterY};
    units::radian_t zeroRadians{0};
    const frc::Rotation2d testRot{zeroRadians};
    const frc::Pose2d kZeroPose{testTrans, testRot};
    frc::DifferentialDriveOdometry odometry{testRot, units::meter_t{0.0}, units::meter_t{0.0}};
    const units::meter_t kTrackWidth{0.55};
    frc::DifferentialDriveKinematics kinematics{kTrackWidth};
    frc::Trajectory trajectory{};
    frc::RamseteController ramseteController{};

    frc::Field2d field;

    double trajectorySecOffset = 0;
    
    double turnInPlaceDegrees = 0;
    double breakEndSec = 0;
    std::deque<double> lastDegrees; // storage of the last n degree differences given to either turnInPlace functions

    bool odometryInitialized;   // used to make sure odometry is only reset once

    // meters per second to ticks per decisecond conversion factor for 6 in wheels
    // const double mpsToTpds = (6.0 / 0.1524) * (1 / (6.0 * M_PI)) * (64.0 / 8.0) * (2048.0) * (0.1);
    // const double metersToTicks = (6.0 / 0.1524) * (1 / (6.0 * M_PI)) * (64.0 / 8.0) * (2048.0);

    // meters per second to ticks per decisecond converstion factor for 4 in wheels
    // const double mpsToTpds = (4.0 / 0.1016) * (1 / (4.0 * M_PI)) * (44.0 / 9.0) * (2048.0) * (0.1);
    const double mpsToRpm = 12.500044*60.0;// * 0.837*0.9375;//(1.0/((1.0/1.0)*(1.0/4.0)*((4*M_PI)/1)*(1.0/39.0)*(1.0/60.0)));
    const double rotationsToMeters = 1.0/(12.500044);//*0.837*0.9375);//(1.0/4.0)*((4.0*M_PI)/1.0)*(1.0/39.3701);

    // forwards are leads
    rev::CANSparkMax dbL{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax dbLF{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder dbLEncoder = dbL.GetEncoder();
    rev::SparkMaxPIDController dbLPIDController = dbL.GetPIDController();

    rev::CANSparkMax dbR{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax dbRF{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder dbREncoder = dbR.GetEncoder();
    rev::SparkMaxPIDController dbRPIDController = dbR.GetPIDController();

    double drivebaseMultiplier = 1;

    int ChargeStationTraverseStep = 0;

};