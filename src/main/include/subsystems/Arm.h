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
    bool wristInitialized = false;
    bool pivotInitialized = false;

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

    void ToggleSoftLimits();

    void EnableSoftLimits(ArmData &armData);
    void DisableSoftLimits();

    void SemiAuto(const RobotData &robotData, ArmData &armData);
    void Manual(const RobotData &robotData, ArmData &armData);

    void SetAngleOfWrist(ArmData &armData, double desiredAngle);
    void SetAngleOfPivot(ArmData &armData, double desiredAngle);

    void ZeroRelativePositionWrist(ArmData &armData);
    void ZeroRelativePositionPivot(ArmData &armData);

    bool IsWristAbolsoluteEncoderInitialized(ArmData &armData);
    bool IsPivotAbolsoluteEncoderInitialized(ArmData &armData);
    
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
    double armWristRelativeMaxPosition = 13; // TODO: fix this value when we get subsystem
    double armWristRelativeMinPosition = 0; // TODO: fix this value when we get subsystem

    double armWristAbsoluteMaxPosition = 0.93418697534; // TODO: fix this value when we get subsystem
    double armWristAbosluteMinPosition = 0.14207; // TODO: fix this value when we get subsystem

    // Pivot Encoder Min and Max Values
    double armPivotRelativeMaxPosition = 0;
    double armPivotRelativeMinPosition = 0;

    double armPivotAbsoluteMaxPosition = 0;
    double armPivotAbsoluteMinPosition = 0;

    // angles for pivot and wrist
    double armPivotMaxAngle = 0;
    double armPivotMinAngle = 0;

    double armWristMaxAngle = 0;
    double armWristMinAngle = 0;

    bool softLimitsToggled = false;
};