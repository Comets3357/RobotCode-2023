#pragma once

#include "Constants.h"
#include "auton/Auton.h"
#include "common/Gyro.h"
#include "common/CustomDifferentialDriveOdometry.h"

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
// #include <wpi/uv/Error.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
// #include <frc/kinematics/DifferentialDrivePoseEstimator.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
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

#include <frc/trajectory/TrajectoryGenerator.h> 
#include <frc/trajectory/TrajectoryConfig.h> 
// #include <frc/geometry/Translation2d.h>

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <math.h>

#define M_PI 3.14159265358979323846

struct RobotData;

enum DriveMode {
    DRIVEMODE_JOYSTICK,
    DRIVEMODE_TURNINPLACE,
    DRIVEMODE_BREAK,
    DRIVEMODE_TRAJECTORY,
    DRIVEMODE_VECTOR,
    DRIVEMODE_CHARGE_STATION_TRAVERSE,
    DRIVEMODE_AUTO_BALANCE,
    DRIVEMODE_HIT_CHARGE_STATION,
    DRIVEMODE_TURN_TO_HEADING,
    DRIVEMODE_AUTOALLIGN
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

    bool allowBullBarExtend = true;

    bool autoAllign = false;

    bool allowEject = false;

    bool dontRunAnything = false;

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
    void DisabledPeriodic(const RobotData &robotData);

private:

    double GetTurnMax();
    void updateData(const RobotData &robotData, DrivebaseData &drivebaseData);
    void teleopControl(const RobotData &robotData, DrivebaseData &drivebaseData, GyroData &gyroData, ControlData &controlData);
    void autonControl(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData, GyroData &gyroData, ControlData &controlData);

    // odometry
    void updateOdometry(const RobotData &robotData, DrivebaseData &drivebaseData);
    void resetOdometry(const frc::Pose2d &pose, double gyroAngle);
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
    units::meter_t meterX{0};
    units::meter_t meterY{0};
    const frc::Translation2d testTrans{meterX, meterY};
    units::radian_t zeroRadians{0};
    const frc::Rotation2d testRot{zeroRadians};
    const frc::Pose2d kZeroPose{testTrans, testRot};
    // frc::DifferentialDrivePoseEstimator odometry{testRot, units::meter_t{0.0}, units::meter_t{0.0}};
    const units::meter_t kTrackWidth{0.48};
    frc::DifferentialDriveKinematics kinematics{kTrackWidth};
    frc::DifferentialDrivePoseEstimator odometry{kinematics, testRot, units::meter_t{0.0}, units::meter_t{0.0}, kZeroPose};    
    frc::Trajectory trajectory{};
    frc::RamseteController ramseteController{};

    CustomDifferentialDriveOdometry customOdometry;

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
    const double mpsToRpm = 11.107086*60.0;// * 0.837*0.9375;//(1.0/((1.0/1.0)*(1.0/4.0)*((4*M_PI)/1)*(1.0/39.0)*(1.0/60.0)));
    const double rotationsToMeters = 1.0/(11.107086);//*0.837*0.9375);//(1.0/4.0)*((4.0*M_PI)/1.0)*(1.0/39.3701);
    const double degreesToMeters = 2.05/360.0;

    // forwards are leads
    rev::CANSparkMax dbL{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax dbLF{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder dbLEncoder = dbL.GetEncoder();
    rev::SparkMaxPIDController dbLPIDController = dbL.GetPIDController();

    rev::CANSparkMax dbR{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax dbRF{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder dbREncoder = dbR.GetEncoder();
    rev::SparkMaxPIDController dbRPIDController = dbR.GetPIDController();

    frc::TrapezoidProfile<units::meters>::Constraints constraints{units::velocity::meters_per_second_t{5}, units::acceleration::meters_per_second_squared_t{7}};
    double currentVelocity = 0.0;
    frc::TrapezoidProfile<units::meters> drivebaseProfile
    {
        frc::TrapezoidProfile<units::meters>::Constraints{units::velocity::meters_per_second_t{0}, units::acceleration::meters_per_second_squared_t{0}},
        frc::TrapezoidProfile<units::meters>::State{units::meter_t{0}, units::meters_per_second_t{0}},
        frc::TrapezoidProfile<units::meters>::State{units::meter_t{0}, units::meters_per_second_t{0}}
    };
    int allignState = 0;
    units::time::second_t elapsedTime{0};
    frc::TrapezoidProfile<units::meters>::State currentState{units::meter_t{0}, units::meters_per_second_t{0}};
    frc::TrapezoidProfile<units::meters>::State endState{units::meter_t{0}, units::meters_per_second_t{0}};


    bool profileCreated = false;
    double startTime = 0.0;
    double startPosition = 0.0;
    double endPosition = 0.0;

    double drivebaseMultiplier = 1;

    int ChargeStationTraverseStep = 0;

    bool forward = true;

    double angleOff = 0;
    double distanceOff = 0;

    double chargeStationBackoffBeginTime = 0;
    double chargeStationBeginFailSafe = 0;

    int substationStep = 0;

    std::vector<frc::Translation2d> interiorWaypoints;
    frc::Pose2d endPoint;
    frc::TrajectoryConfig config{7_mps, 2.8_mps_sq};



    double minLimelightAutoAllign = -4;
    double maxLimelightAutoAllign = 4;
    double minConeDistanceAutoAllign = 0.07;
    double maxConeDistanceAutoAllign = 0.35;

    double midToHighPoleLength = 16.16;
    double distanceToMidPole = 29.16;

    double targetLimelightValue = 0;
    bool overrideAutoAllign = false;
    double deltaTime = 0;

    bool lastMid = false;
    bool lastHigh = false;


    // bool autoAllign = false;
};