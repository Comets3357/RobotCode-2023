#include "subsystems/Arm.h"
#include "RobotData.h"
#include <math.h>

void Arm::RobotInit(ArmData &armData)
{
    // Wrist Initialization
    armWristPIDController.SetP(0.1, 0);
    armWristPIDController.SetI(0, 0);
    armWristPIDController.SetD(0, 0);
    armWristPIDController.SetIZone(0, 0);
    armWristPIDController.SetFF(0, 0);

    armWristPIDController.SetOutputRange(-1, 1, 0);
    armWrist.EnableVoltageCompensation(10.5);
    armWrist.SetSmartCurrentLimit(45);

    armWristAbsoluteEncoder.SetZeroOffset(1);
    armWristAbsoluteEncoder.SetPositionConversionFactor(1);
    armWristRelativeEncoder.SetPosition(1);

    armWristPIDController.SetFeedbackDevice(armWristAbsoluteEncoder);

    armWrist.BurnFlash();

    armPivotPIDController.SetP(0.1, 0);
    armPivotPIDController.SetI(0, 0);
    armPivotPIDController.SetD(0, 0);
    armPivotPIDController.SetIZone(0, 0);
    armPivotPIDController.SetFF(0, 0);

    armPivotPIDController.SetOutputRange(-1, 1, 0);
    armPivot.EnableVoltageCompensation(10.5);
    armPivot.SetSmartCurrentLimit(45);

    armPivotAbsoluteEncoder.SetZeroOffset(1);
    armPivotAbsoluteEncoder.SetPositionConversionFactor(1);
    armPivotRelativeEncoder.SetPosition(1);

    armPivotPIDController.SetFeedbackDevice(armPivotAbsoluteEncoder);

    armPivot.BurnFlash();

    ZeroRelativePositionWrist(armData);
    ZeroRelativePositionPivot(armData);


    //Trapezoid Profile

    // ToggleSoftLimits();
}



void Arm::ZeroRelativePositionWrist(ArmData& armData)
{
    if (IsWristAbolsoluteEncoderInitialized(armData))
    {
        armWristRelativeEncoder.SetPosition(armWristAbsoluteEncoder.GetPosition());
    }
}

void Arm::ZeroRelativePositionPivot(ArmData& armData)
{
    if (IsPivotAbolsoluteEncoderInitialized(armData))
    {
        armPivotRelativeEncoder.SetPosition(armPivotAbsoluteEncoder.GetPosition());
    }
}

bool Arm::IsWristAbolsoluteEncoderInitialized(ArmData& armData)
{
    if (armWristAbsoluteEncoder.GetPosition() >= 0.01)
    {
        armData.wristAbsoluteInitialized = true;
        wristRunMode = ABSOLUTE_RUN;
    }
    else 
    {
        armData.wristAbsoluteInitialized = false;
        wristRunMode = NONE;
    }

    return armData.wristAbsoluteInitialized;
}

bool Arm::IsPivotAbolsoluteEncoderInitialized(ArmData& armData)
{
    if (armPivotAbsoluteEncoder.GetPosition() >= 0.01)
    {
        armData.pivotAbsoluteInitialized = true;
        pivotRunMode = ABSOLUTE_RUN;
    }
    else 
    {
        armData.pivotAbsoluteInitialized = false;
        pivotRunMode = NONE;
    }

    return armData.pivotAbsoluteInitialized;
}

void Arm::RobotPeriodic(const RobotData &robotData, ArmData &armData)
{
   // changing controls based off the mode robot is in
    switch (robotData.controlData.mode) 
    {
        case MODE_TELEOP_MANUAL:
            Manual(robotData, armData);
            break;
        case MODE_TELEOP_SA:
            SemiAuto(robotData, armData);
            break;
        default:
            SemiAuto(robotData, armData);
            break;
    }

    if (armPivotRelativeEncoder.GetVelocity() <= 1) // && inRelativeMode
    {
        ZeroRelativePositionPivot(armData);
    }

    if (armWristRelativeEncoder.GetVelocity() <= 1) // && inRelativeMode
    {
        ZeroRelativePositionWrist(armData);
    }
}

void Arm::SemiAuto(const RobotData &robotData, ArmData &armData)
{

    if (!wristSoftLimitsToggled)
    {
        EnableWristSoftLimits();
    }

    if (!pivotSoftLimitsToggled)
    {
        EnablePivotSoftLimits();
    }

    if (armData.pivotAbsoluteInitialized && pivotRunMode != ABSOLUTE_RUN)
    {
        pivotRunMode = ABSOLUTE_RUN;
        armPivotPIDController.SetFeedbackDevice(armPivotAbsoluteEncoder);
        pivotForceZeroed = false;
    }
    else if (pivotForceZeroed && pivotRunMode != RELATIVE_RUN)
    {
        pivotRunMode = RELATIVE_RUN;
        armPivotPIDController.SetFeedbackDevice(armPivotRelativeEncoder);
    }

    if (armData.wristAbsoluteInitialized && wristRunMode != ABSOLUTE_RUN)
    {
        wristRunMode = ABSOLUTE_RUN;
        armWristPIDController.SetFeedbackDevice(armWristAbsoluteEncoder);
        wristForceZeroed = false;
    }
    else if (wristForceZeroed && wristRunMode != RELATIVE_RUN)
    {
        pivotRunMode = RELATIVE_RUN;
        armWristPIDController.SetFeedbackDevice(armWristRelativeEncoder);
    }

    if (pivotRunMode != NONE && wristRunMode != NONE)
    {
        if (robotData.controlData.saArmIntakePosition)
        {
        // SetAngleOfWrist(armData, 0);
        // SetAngleOfPivot(armData, 0);
        }
        if (robotData.controlData.saMoveArm)
        {
            
        }
    }
    else
    {
        armWrist.Set(0);
        armPivot.Set(0);
    }

    if (profileActive)
    {
        units::time::second_t elapsedTime{robotData.timerData.secSinceEnabled - profileStartTime};
        auto setpoint = profile.Calculate(elapsedTime);
        double feedForward = a * sin(((b * armPivotAbsoluteEncoder.GetPosition()) + c) / 180.0 * 3.14159265358979);

        armPivotPIDController.SetReference(setpoint.position.value(), rev::CANSparkMax::ControlType::kPosition, feedForward);
    }
    
}

void Arm::RotatePivot(double r, RobotData& robotData)
{
    

    profileActive = true;
    profileStartTime = robotData.timerData.secSinceEnabled;
    profileStartPos = armPivotAbsoluteEncoder.GetPosition();
    profileEndPos = armPivotAbsoluteEncoder.GetPosition() + r;

    profile = frc::TrapezoidProfile<units::degrees>
    {
        frc::TrapezoidProfile<units::degrees>::Constraints{1_deg_per_s, 0.5_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{profileStartPos}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{profileEndPos}, units::angular_velocity::degrees_per_second_t{0}}
    };

}

void Arm::Manual(const RobotData &robotData, ArmData &armData)
{
    if (wristSoftLimitsToggled)
    {
        DisableWristSoftLimits();
    }

    if (pivotSoftLimitsToggled)
    {
        DisablePivotSoftLimits();
    }

    if (robotData.controlData.mMovePivot)
    {
        armPivot.Set(robotData.controllerData.sLYStick);
    }
    else
    {
        armPivot.Set(0);
    }

    if (robotData.controlData.mMoveWrist)
    {
        armWrist.Set(robotData.controllerData.sRYStick);
    }
    else
    {
        armWrist.Set(0);
    }


    if (robotData.controlData.mForceZeroPivot)
    {
        ForceZeroPivot();
    }

    if (robotData.controlData.mForceZeroWrist)
    {
        ForceZeroWrist();
    }
}

void Arm::DisabledInit()
{

}

void Arm::DisabledPeriodic(const RobotData &robotData, ArmData &armData)
{

}
void Arm::UpdateData(const RobotData &robotData, ArmData &armData)
{
    
}

void Arm::DisableWristSoftLimits()
{
    armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);

    wristSoftLimitsToggled = false;
}

void Arm::DisablePivotSoftLimits()
{
    armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);

    pivotSoftLimitsToggled = false;
}

void Arm::EnableWristSoftLimits()
{

    armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armWristMinPosition - 0.1);
    armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armWristMaxPosition + 0.1);

    armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    wristSoftLimitsToggled = true;
}

void Arm::EnablePivotSoftLimits()
{

    armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armPivotMinPosition - 0.1);
    armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armPivotMaxPosition + 0.1);

    armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    armPivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    pivotSoftLimitsToggled = true;
}

void Arm::ForceZeroPivot()
{
    armPivotRelativeEncoder.SetPosition(armPivotMinPosition);
    pivotForceZeroed = true;
}

void Arm::ForceZeroWrist()
{
    armWristRelativeEncoder.SetPosition(armWristMinPosition);
    wristForceZeroed = true;
}