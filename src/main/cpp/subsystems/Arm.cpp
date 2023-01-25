#include "subsystems/Arm.h"
#include "RobotData.h"

void Arm::RobotInit(ArmData &armData)
{
    // Wrist Initialization
    armWristPIDController.SetP(0.1, 0);
    armWristPIDController.SetI(0, 0);
    armWristPIDController.SetD(0, 0);
    armWristPIDController.SetIZone(0, 0);
    armWristPIDController.SetFF(0, 0);

    armWristPIDController.SetOutputRange(-0.2, 0.2, 0);
    armWrist.EnableVoltageCompensation(10.5);
    armWrist.SetSmartCurrentLimit(45);

    armWristAbsoluteEncoder.SetPositionConversionFactor(1);
    armWristAbsoluteEncoder.SetZeroOffset(1);
    armWristRelativeEncoder.SetPosition(1);

    armWristPIDController.SetFeedbackDevice(armWristAbsoluteEncoder);

    armWrist.BurnFlash();

    armPivotPIDController.SetP(0.1, 0);
    armPivotPIDController.SetI(0, 0);
    armPivotPIDController.SetD(0, 0);
    armPivotPIDController.SetIZone(0, 0);
    armPivotPIDController.SetFF(0, 0);

    armPivotPIDController.SetOutputRange(-0.2, 0.2, 0);
    armPivot.EnableVoltageCompensation(10.5);
    armPivot.SetSmartCurrentLimit(45);

    armPivotAbsoluteEncoder.SetPositionConversionFactor(1);
    armPivotAbsoluteEncoder.SetZeroOffset(1);
    // armPivotRelativeEncoder.SetPosition(1);

    armPivotPIDController.SetFeedbackDevice(armPivotAbsoluteEncoder);

    armPivot.BurnFlash();

    ZeroRelativePositionWrist(armData);
    ZeroRelativePositionPivot(armData);


    //Trapezoid Profile

    // ToggleSoftLimits();
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

/* --------------------------------------------------------------------------------------------------------------------------
*                                   WRIST SEMI AUTO FEEDBACK DEVICES
*  --------------------------------------------------------------------------------------------------------------------------
*/

    if (!wristSoftLimitsToggled)
    {
        EnableWristSoftLimits();
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

/* -----------------------------------------------------------------------------------------------------------------------------
*                                   PIVOT SEMI AUTO FEEDBACK DEVICES
*  -----------------------------------------------------------------------------------------------------------------------------
*/ 
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

/* --------------------------------------------------------------------------------------------------------------------------
*                                   SEMI AUTO FULL FUNCTIONALITY CODE
*  --------------------------------------------------------------------------------------------------------------------------
*/

    if (pivotRunMode != NONE && wristRunMode != NONE)
    {
        if (robotData.controlData.saArmIntakePosition)
        {
        // SetAngleOfWrist(armData, 0);
        // SetAngleOfPivot(armData, 0);
        }
        else if (robotData.controlData.saArmPositionTwo)
        {

        }
        else if (robotData.controlData.saArmPositionThree)
        {

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
/* --------------------------------------------------------------------------------------------------------------------------
*                                   W RIZZ TRAP MOVEMENTS
*  --------------------------------------------------------------------------------------------------------------------------
*/

    if (wristProfileActive)
    {
        units::time::second_t elapsedTime{robotData.timerData.secSinceEnabled - pivotProfileStartTime};
        auto setPoint = wristProfile.Calculate(elapsedTime);
        double feedForward = wristFeedForwardA * sin(((wristFeedForwardB * armPivotAbsoluteEncoder.GetPosition()) + wristFeedForwardC) / 180.0 * 3.14159265358979);

        armWristPIDController.SetReference(setPoint.position.value(), rev::CANSparkMax::ControlType::kPosition, feedForward);

        if (wristProfile.IsFinished(elapsedTime))
        {
            wristProfileActive = false;
        }
    }

    if (pivotProfileActive)
    {
        units::time::second_t elapsedTime{robotData.timerData.secSinceEnabled - pivotProfileStartTime};
        auto setpoint = pivotProfile.Calculate(elapsedTime);
        double feedForward = pivotFeedForwardA * sin(((pivotFeedForwardB * armPivotAbsoluteEncoder.GetPosition()) + pivotFeedForwardC) / 180.0 * 3.14159265358979);

        armPivotPIDController.SetReference(setpoint.position.value(), rev::CANSparkMax::ControlType::kPosition, feedForward);

        if (pivotProfile.IsFinished(elapsedTime))
        {
            pivotProfileActive = false;
        }
    }
}

void Arm::Manual(const RobotData &robotData, ArmData &armData)
{
    // if (wristSoftLimitsToggled)
    // {
    //     DisableWristSoftLimits();
    // }

    // if (pivotSoftLimitsToggled)
    // {
    //     DisablePivotSoftLimits();
    // }

    EnablePivotSoftLimits();
    EnableWristSoftLimits();

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

void Arm::RotatePivot(double targetDegree, RobotData& robotData)
{
    pivotProfileActive = true;
    pivotProfileStartTime = robotData.timerData.secSinceEnabled;

    if (pivotRunMode == ABSOLUTE_RUN)
    {
        pivotProfileStartPos = armPivotAbsoluteEncoder.GetPosition();
        pivotProfileEndPos = targetDegree;
    }
    else
    {
        pivotProfileStartPos = armPivotRelativeEncoder.GetPosition();
        pivotProfileEndPos = targetDegree;
    }

    pivotProfile = frc::TrapezoidProfile<units::degrees>
    {
        frc::TrapezoidProfile<units::degrees>::Constraints{1_deg_per_s, 0.5_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{pivotProfileStartPos}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{pivotProfileEndPos}, units::angular_velocity::degrees_per_second_t{0}}
    };

}

void Arm::RotateWrist(double targetDegree, RobotData& robotData)
{
    wristProfileActive = true;
    wristProfileStartTime = robotData.timerData.secSinceEnabled;
    
    if (wristRunMode == RELATIVE_RUN)
    {
        wristProfileStartPos = armWristAbsoluteEncoder.GetPosition();
        wristProfileEndPos = targetDegree;
    }
    else
    {
        wristProfileStartPos = armWristRelativeEncoder.GetPosition();
        wristProfileEndPos = targetDegree;
    }

    wristProfile = frc::TrapezoidProfile<units::degrees>
    {
        frc::TrapezoidProfile<units::degrees>::Constraints{1_deg_per_s, 0.5_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{wristProfileStartPos}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{wristProfileEndPos}, units::angular_velocity::degrees_per_second_t{0}}
    };
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