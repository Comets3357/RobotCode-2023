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

struct ArmData
{
    bool wristAbsoluteInitialized = false;
    bool pivotAbsoluteInitialized = false;

    double pivotAngle = 0;
    double wristAngle = 0;
};

enum ArmRunMode
{
    ABSOLUTE_RUN,
    RELATIVE_RUN,
    NONE
};

class Arm
{
public:
    void RobotInit(ArmData &armData);
    void RobotPeriodic(const RobotData &robotData, ArmData &armData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, ArmData &armData);
    void UpdateData(const RobotData &robotData, ArmData &armData);

private:

    void EnableWristSoftLimits();
    void DisableWristSoftLimits();

    void EnablePivotSoftLimits();
    void DisablePivotSoftLimits();

    void SemiAuto(const RobotData &robotData, ArmData &armData);
    void Manual(const RobotData &robotData, ArmData &armData);

    void ZeroRelativePositionWrist(ArmData &armData);
    void ZeroRelativePositionPivot(ArmData &armData);

    bool IsWristAbolsoluteEncoderInitialized(ArmData &armData);
    bool IsPivotAbolsoluteEncoderInitialized(ArmData &armData);

    void ForceZeroWrist();
    void ForceZeroPivot();

    bool pivotForceZeroed;
    bool wristForceZeroed;

    ArmRunMode pivotRunMode = NONE;
    ArmRunMode wristRunMode = NONE;
    
    // joint Pivot Initialization
    rev::CANSparkMax armWrist = rev::CANSparkMax(armWristID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder armWristRelativeEncoder = armWrist.GetEncoder(); // Relative Encoder
    rev::SparkMaxPIDController armWristPIDController = armWrist.GetPIDController(); // PID Controller
    rev::SparkMaxAbsoluteEncoder armWristAbsoluteEncoder = armWrist.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    rev::CANSparkMax armPivot = rev::CANSparkMax(armWristID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder armPivotRelativeEncoder = armPivot.GetEncoder(); // Relative Encoder
    rev::SparkMaxPIDController armPivotPIDController = armPivot.GetPIDController(); // PID Controller
    rev::SparkMaxAbsoluteEncoder armPivotAbsoluteEncoder = armPivot.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    // Wrist Encoder Min and Max Values 
    double armWristMaxPosition = 13; // TODO: fix this value when we get subsystem
    double armWristMinPosition = 0; // TODO: fix this value when we get subsystem

    // Pivot Encoder Min and Max Values
    double armPivotMaxPosition = 0;
    double armPivotMinPosition = 0;

    bool pivotSoftLimitsToggled = false;
    bool wristSoftLimitsToggled = false;
};