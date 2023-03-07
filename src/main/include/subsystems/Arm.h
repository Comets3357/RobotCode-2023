#pragma once

#include "Constants.h"
#include "auton/Auton.h"

#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycle.h>
#include <frc/DigitalInput.h>
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

struct ArmData
{
    bool wristAbsoluteInitialized = false;
    bool pivotAbsoluteInitialized = false;

    double pivotAngle = 0;
    double wristAngle = 0;

    bool wristSafePosition = false;
    bool wristSafeCubeDetectionPosition = false;

    bool cubeIntakeRunning = false;
    bool coneIntakeRunning = false;
    bool humanPlayerConeIntakeRunning = false;
    bool uprightConeIntakeRunning = false;

    bool wristAndArmInPositionForBullBarIntake = false;

};

enum ArmRunMode
{
    ARM_ABSOLUTE_RUN,
    ARM_RELATIVE_RUN,
    ARM_NONE
};

class Arm 
{
public:
    void RobotInit(const RobotData &robotData, ArmData &armData);
    void RobotPeriodic(const RobotData &robotData, ArmData &armData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, ArmData &armData);
    void UpdateData(const RobotData &robotData, ArmData &armData);

    int ran = 0;

private:

units::angle::degree_t pivotMaxAcceleration{700_deg};

    void EnableWristSoftLimits();
    void DisableWristSoftLimits();

    void EnablePivotSoftLimits();
    void DisablePivotSoftLimits();

    void AdvancedSemiAuto(const RobotData &robotData, ArmData &armData);
    void SemiAuto(const RobotData &robotData, ArmData &armData);
    void Manual(const RobotData &robotData, ArmData &armData);

    void ZeroRelativePositionWrist(ArmData &armData);
    void ZeroRelativePositionPivot(ArmData &armData);

    bool IsWristAbolsoluteEncoderInitialized(ArmData &armData);
    bool IsPivotAbolsoluteEncoderInitialized(ArmData &armData);

    void ForceZeroWrist();
    void ForceZeroPivot();

    void RotatePivot(double targetDegree, const RobotData& robotData, double timeOffset);
    void RotateWrist(double targetDegree, const RobotData& robotData, double timeOffset);

    bool pivotProfileActive = false;
    double pivotProfileStartPos = 0;
    double pivotProfileEndPos = 0;
    double pivotProfileStartTime = 0;    
    frc::TrapezoidProfile<units::degree> pivotProfile
    {
        frc::TrapezoidProfile<units::degrees>::Constraints{0_deg_per_s, 0_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{0}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{0}, units::angular_velocity::degrees_per_second_t{0}}
    };

    double pivotFeedForwardA = 1;
    double pivotFeedForwardB = 1;
    double pivotFeedForwardC = 1;

    bool wristProfileActive = false;
    double wristProfileStartPos = 0;
    double wristProfileEndPos = 0;
    double wristProfileStartTime = 0;    
    frc::TrapezoidProfile<units::degree> wristProfile
    {
        frc::TrapezoidProfile<units::degrees>::Constraints{0_deg_per_s, 0_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{0}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{0}, units::angular_velocity::degrees_per_second_t{0}}
    };

    double wristFeedForwardA = 1;
    double wristFeedForwardB = 1;
    double wristFeedForwardC = 1;

    bool pivotForceZeroed;
    bool wristForceZeroed;

    ArmRunMode pivotRunMode = ARM_NONE;
    ArmRunMode wristRunMode = ARM_NONE;
    
    // joint Pivot Initialization
    rev::CANSparkMax armWrist = rev::CANSparkMax(armWristID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder armWristRelativeEncoder = armWrist.GetEncoder(); // Relative Encoder
    rev::SparkMaxPIDController armWristPIDController = armWrist.GetPIDController(); // PID Controller
    rev::SparkMaxAbsoluteEncoder armWristAbsoluteEncoder = armWrist.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    rev::CANSparkMax armPivot = rev::CANSparkMax(armPivotID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder armPivotRelativeEncoder = armPivot.GetEncoder(); // Relative Encoder
    rev::SparkMaxPIDController armPivotPIDController = armPivot.GetPIDController(); // PID Controller
    rev::SparkMaxAbsoluteEncoder armPivotAbsoluteEncoder = armPivot.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    // Wrist Encoder Min and Max Values 
    double armWristMaxPosition = 250; // TODO: fix this value when we get subsystem
    double armWristMinPosition = 10; // TODO: fix this value when we get subsystem

    // Pivot Encoder Min and Max Values
    double armPivotMaxPosition = 170;
    double armPivotMinPosition = 10;

    bool pivotSoftLimitsToggled = false;
    bool wristSoftLimitsToggled = false;

    bool readyRunBasedOffBullBar = false;
    bool bullBarIn = false;

    bool coneIntakeToggle = false;
    bool cubeIntakeToggle = false;
    bool humanPlayerIntakeToggle = false;
    bool uprightConeIntakeToggle = false;
    bool coneFlipPosition = false;

    bool endEffectorGamePiece = false;
    bool endEffectorGamePiecePastRead = false;

    bool wristInPositionForArm = false;
    bool wristInPositionForArmPastRead = false;

    bool inAuton = false;
};