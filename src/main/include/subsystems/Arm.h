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

};

class Arm
{
public:
    void RobotInit();
    void RobotPeriodic();

private:
    double absoluteToRelative(double currentPos);
    double angleToRelative(double angle);
    double angleToAbsolute(double angle);

    void RotateArmToAngle(double angle);
    void RotateArmAngle(double angle);
    void ZeroArm();
    void ToggleSoftLimits();
    void ArmWrist(double wristPosition);
    
    // Intake Pivot Initialization
    rev::CANSparkMax armWrist = rev::CANSparkMax(armWristID, rev::CANSparkMax::MotorType::kBrushless);
    rev::SparkMaxRelativeEncoder armWristRelativeEncoder = armWrist.GetEncoder(); // Relative Encoder
    rev::SparkMaxPIDController armWristPIDController = armWrist.GetPIDController(); // PID Controller
    frc::DigitalInput m_input{intakeAbsoluteEncoderPort};
    frc::DutyCycle armWristAbsoluteEncoder = frc::DutyCycle{m_input}; // Absolute Encoder

    // Encoder Min and Max Values 
    double armWristRelativeMaxPosition = 13; // TODO: fix this value when we get subsystem
    double armWristRelativeMinPosition = 0; // TODO: fix this value when we get subsystem

    double armWristAbsoluteMaxPosition = 0.93418697534; // TODO: fix this value when we get subsystem
    double armWristAbosluteMinPosition = 0.14207; // TODO: fix this value when we get subsystem

    bool softLimitsToggled = false;
};