#include "subsystems/Arm.h"
#include "RobotData.h"

void Arm::RobotInit(ArmData &armData)
{
    // Wrist Initialization
    armWristPIDController.SetP(0.1, 0); // 0.35
    armWristPIDController.SetI(0, 0);
    armWristPIDController.SetD(0, 0);
    armWristPIDController.SetIZone(0, 0);
    armWristPIDController.SetFF(0, 0);

    armWristPIDController.SetOutputRange(-1, 1, 0);
    armWrist.EnableVoltageCompensation(10.5);
    armWrist.SetSmartCurrentLimit(20);
    armWrist.SetInverted(false);
    armWrist.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    armWristAbsoluteEncoder.SetInverted(false);
    armWristAbsoluteEncoder.SetPositionConversionFactor(360);
    armWristAbsoluteEncoder.SetZeroOffset(9.299444);
    armWristRelativeEncoder.SetPositionConversionFactor(360.0/82.09);//90.453243);//4.2976522);
    armWristRelativeEncoder.SetPosition(10);

    armWristRelativeEncoder.SetPosition(10);

    armWristPIDController.SetFeedbackDevice(armWristRelativeEncoder);

    armWrist.BurnFlash();

    armPivotPIDController.SetP(0.04833*2, 0);
    armPivotPIDController.SetI(0, 0);
    armPivotPIDController.SetD(0, 0);
    armPivotPIDController.SetIZone(0, 0);
    armPivotPIDController.SetFF(0, 0);

    armPivotPIDController.SetOutputRange(-1, 1, 0);
    armPivot.EnableVoltageCompensation(10.5);
    armPivot.SetSmartCurrentLimit(45);

    armPivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    armPivotAbsoluteEncoder.SetInverted(true);
    armPivotAbsoluteEncoder.SetPositionConversionFactor(360);
    armPivotAbsoluteEncoder.SetZeroOffset(99.1 + 3.644505);
    armPivotRelativeEncoder.SetPositionConversionFactor(1.565569);
    armPivotRelativeEncoder.SetPosition(10);

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
    
    frc::SmartDashboard::PutNumber("current rev for wrist", armWristRelativeEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("current run mode", wristRunMode);
    frc::SmartDashboard::PutNumber("wristAbs", armWristAbsoluteEncoder.GetPosition());

    frc::SmartDashboard::PutNumber("arm angle relative", armPivotRelativeEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("arm angle abs", armPivotAbsoluteEncoder.GetPosition());
    // if (armPivotRelativeEncoder.GetVelocity() <= 1) // && inRelativeMode
    // {
    //     ZeroRelativePositionPivot(armData);
    // }

    // if (armWristRelativeEncoder.GetVelocity() <= 1) // && inRelativeMode
    // {
    //     ZeroRelativePositionWrist(armData);
    // }

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

    if (armData.wristAbsoluteInitialized && wristRunMode != ARM_ABSOLUTE_RUN)
    {
        wristRunMode = ARM_ABSOLUTE_RUN;
        armWristPIDController.SetFeedbackDevice(armWristAbsoluteEncoder);
        wristForceZeroed = false;
    }
    else if (wristForceZeroed && wristRunMode != ARM_RELATIVE_RUN)
    {
        wristRunMode = ARM_RELATIVE_RUN;
        armWristPIDController.SetFeedbackDevice(armWristRelativeEncoder);
    }

    wristRunMode = ARM_RELATIVE_RUN;
    armWristPIDController.SetFeedbackDevice(armWristRelativeEncoder);

/* -----------------------------------------------------------------------------------------------------------------------------
*                                   PIVOT SEMI AUTO FEEDBACK DEVICES
*  -----------------------------------------------------------------------------------------------------------------------------
*/ 
    if (!pivotSoftLimitsToggled)
    {
        EnablePivotSoftLimits();
    }

    if (armData.pivotAbsoluteInitialized && pivotRunMode != ARM_ABSOLUTE_RUN)
    {
        pivotRunMode = ARM_ABSOLUTE_RUN;
        armPivotPIDController.SetFeedbackDevice(armPivotAbsoluteEncoder);
        pivotForceZeroed = false;
    }
    else if (pivotForceZeroed && pivotRunMode != ARM_RELATIVE_RUN)
    {
        pivotRunMode = ARM_RELATIVE_RUN;
        armPivotPIDController.SetFeedbackDevice(armPivotRelativeEncoder);
    }
    

    pivotRunMode = ARM_RELATIVE_RUN;
    armPivotPIDController.SetFeedbackDevice(armPivotRelativeEncoder);

/* --------------------------------------------------------------------------------------------------------------------------
*                                   SEMI AUTO FULL FUNCTIONALITY CODE
*  --------------------------------------------------------------------------------------------------------------------------
*/
    tempVar = controllerFlipped;
    controllerFlipped = robotData.controlData.saConeIntake || robotData.controlData.saCubeIntake;

    endEffectorGamePiecePastRead = endEffectorGamePiece;
    if (robotData.endEffectorData.gamePieceType == CONE || robotData.endEffectorData.gamePieceType == CUBE)
    {
        endEffectorGamePiece = true;
    }
    else
    {
        endEffectorGamePiece = false;
    }

    wristInPositionForArmPastRead = wristInPositionForArm;
    wristInPositionForArm = armWristRelativeEncoder.GetPosition() < 100;
    
    if (robotData.controlData.saPositionHumanPlayer)
    {
        RotateWrist(50, robotData, 0);
        RotatePivot(30, robotData, 0);
    }

    if (pivotRunMode != ARM_NONE || wristRunMode != ARM_NONE)
    {
        if (armWristRelativeEncoder.GetPosition() < 25)
        {
            armData.wristSafeCubeDetectionPosition = false;
        }
        else
        {
            armData.wristSafeCubeDetectionPosition = true;
        }
        switch (robotData.endEffectorData.gamePieceType)
        {

            case CONE:
            
                if (robotData.controlData.saPositionMid)
                {
                    // armWristPIDController.SetReference(120, rev::ControlType::kPosition);
                    // armPivotPIDController.SetReference(100, rev::ControlType::kPosition);
                    RotateWrist(30, robotData, 0);
                    RotatePivot(146, robotData, 0);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);
                // SetAngleOfWrist(armData, 0);
                //RotateWrist(120, robotData);

                // SetAngleOfPivot(armData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {

                    RotateWrist(30, robotData, 1);
                    RotatePivot(147, robotData, 1);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);

                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(11, robotData, 0);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);
                }
                break;

            case CUBE:

                if (robotData.controlData.saPositionMid)
                {
                    // armWristPIDController.SetReference(120, rev::ControlType::kPosition);
                    // armPivotPIDController.SetReference(100, rev::ControlType::kPosition);
                    RotateWrist(50, robotData, 0);
                    RotatePivot(146, robotData, 0);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);
                    frc::SmartDashboard::PutBoolean("I AMM GETTTING HERE", true);
                // SetAngleOfWrist(armData, 0);
                //RotateWrist(120, robotData);

                // SetAngleOfPivot(armData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {

                    RotateWrist(36.6, robotData, 1);
                    RotatePivot(140, robotData, 1);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);

                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(11, robotData, 0);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);
                    
                }
                break;

            case NONE:

                if (robotData.controlData.saPositionMid)
                {
                    // armWristPIDController.SetReference(120, rev::ControlType::kPosition);
                    // armPivotPIDController.SetReference(100, rev::ControlType::kPosition);
                    RotateWrist(30, robotData, 0);
                    RotatePivot(146, robotData, 0);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);
                // SetAngleOfWrist(armData, 0);
                //RotateWrist(120, robotData);

                // SetAngleOfPivot(armData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {

                    RotateWrist(30, robotData, 1);
                    RotatePivot(147, robotData, 1);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);

                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(11, robotData, 0);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);
                }
                break;

            default:

                if (robotData.controlData.saPositionMid)
                {
                    // armWristPIDController.SetReference(120, rev::ControlType::kPosition);
                    // armPivotPIDController.SetReference(100, rev::ControlType::kPosition);
                    RotateWrist(30, robotData, 0);
                    RotatePivot(146, robotData, 0);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);
                // SetAngleOfWrist(armData, 0);
                //RotateWrist(120, robotData);

                // SetAngleOfPivot(armData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {

                    RotateWrist(30, robotData, 1);
                    RotatePivot(147, robotData, 1);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);

                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(11, robotData, 0);
                    // ZeroRelativePositionWrist(armData);
                    // ZeroRelativePositionPivot(armData);
                }
                break;
        }
        
        // if (!robotData.endEffectorData.isCone || !robotData.endEffectorData.isCube)
        // {
            if (robotData.controlData.saConeIntake)
            {
                if (robotData.bullBarData.bullBarSafePosition)
                {
                    if (readyRunBasedOffBullBar != robotData.bullBarData.bullBarSafePosition)
                    {
                        RotatePivot(10, robotData, 0);
                        RotateWrist(132, robotData, 0);
                    }
                }
                else if (!robotData.bullBarData.bullBarSafePosition && (tempVar != controllerFlipped))
                {
                    RotateWrist(50, robotData, 0);
                }
                readyRunBasedOffBullBar = robotData.bullBarData.bullBarSafePosition;
            }
            else if (!robotData.controlData.saCubeIntake)
            {
                if (tempVar != controllerFlipped)
                {
                    
                    RotateWrist(30, robotData, 0);
                    // RotatePivot(25, robotData);
                }

                if (wristInPositionForArmPastRead != wristInPositionForArm && armWristRelativeEncoder.GetPosition() < 100)
                {
                    RotatePivot(11, robotData, 0);
                }
            }
            
            if (robotData.controlData.saCubeIntake)
            {
                if (tempVar != controllerFlipped)
                {
                    RotatePivot(41, robotData, 0);
                    RotateWrist(199.5+3, robotData, 0);
                }
                readyRunBasedOffBullBar = robotData.bullBarData.bullBarSafePosition;
            }
            else if (!robotData.controlData.saConeIntake)
            {
                if (tempVar != controllerFlipped)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(55, robotData, 0);
                }

                if ((wristInPositionForArmPastRead != wristInPositionForArm) && armWristRelativeEncoder.GetPosition() < 100)
                {
                    //RotateWrist(30, robotData, 0);
                    RotatePivot(11, robotData, 0);
                }
            }
        // }
        // else
        // {
        //     if (endEffectorGamePiecePastRead != endEffectorGamePiece)
        //     {
        //         RotatePivot(25, robotData);
        //         RotateWrist(15, robotData);
        //     }
        // }


        if (armWristRelativeEncoder.GetPosition() < 50 || armPivotRelativeEncoder.GetPosition() > 45)
            {
                armData.wristSafePosition = true;
            }
            else 
            {
                armData.wristSafePosition = false;
            }
            armData.wristSafePosition = true;
        
        frc::SmartDashboard::PutBoolean("arm in safe position", armData.wristSafePosition);

        frc::SmartDashboard::PutNumber("WRIST CURRENT POSITION", armWristRelativeEncoder.GetPosition());

        


    }
    else
    {
        armWrist.Set(0);
        armPivot.Set(0);
    }

    if (robotData.controllerData.sXBtn)
    {
        ZeroRelativePositionPivot(armData);
        ZeroRelativePositionWrist(armData);
    }
/* --------------------------------------------------------------------------------------------------------------------------
*                                   TRAP MOVEMENTS
*  --------------------------------------------------------------------------------------------------------------------------
*/

    frc::SmartDashboard::PutNumber("TRAP Active", wristProfileActive);
    frc::SmartDashboard::PutNumber("TRAP elapsed time", robotData.timerData.secSinceEnabled - pivotProfileStartTime);

    if (wristProfileActive && robotData.timerData.secSinceEnabled > wristProfileStartTime)
    {

        units::time::second_t elapsedTime{robotData.timerData.secSinceEnabled - wristProfileStartTime};
        auto setPoint = wristProfile.Calculate(elapsedTime);
        //double feedForward = wristFeedForwardA * sin(((wristFeedForwardB * armPivotAbsoluteEncoder.GetPosition()) + wristFeedForwardC) / 180.0 * 3.14159265358979);

        armWristPIDController.SetReference(setPoint.position.value(), rev::CANSparkMax::ControlType::kPosition);
        frc::SmartDashboard::PutNumber("TRAP Wrist", setPoint.position.value());
        frc::SmartDashboard::PutNumber("TRAP Active", wristProfileActive);
        frc::SmartDashboard::PutNumber("TRAP Wrist Elapsed Time", (double)elapsedTime);

        if (wristProfile.IsFinished(elapsedTime))
        {
            wristProfileActive = false;
        }
        
    }

    if (pivotProfileActive && robotData.timerData.secSinceEnabled > pivotProfileStartTime)
    {
        units::time::second_t elapsedTime{robotData.timerData.secSinceEnabled - pivotProfileStartTime};
        auto setpoint = pivotProfile.Calculate(elapsedTime);

        armPivotPIDController.SetReference(setpoint.position.value(), rev::CANSparkMax::ControlType::kPosition);
        frc::SmartDashboard::PutNumber("TRAP Arm", setpoint.position.value());

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

    if (pivotSoftLimitsToggled)
    {
        DisablePivotSoftLimits();
    }

    // armWristPIDController.SetReference(50, rev::ControlType::kPosition);



    // frc::SmartDashboard::PutBoolean("i AM HERE IN WRIST", true);

    if (robotData.controlData.mMovePivot)
    {
        armPivot.Set(robotData.controllerData.sLYStick * 0.3);
    }
    else
    {
        armPivot.Set(0);
    }

    if (robotData.controlData.mMoveWrist)
    {
        armWrist.Set(robotData.controllerData.sRYStick * 0.25);
    }
    else
    {
        armWrist .Set(0);
    }

    // armWrist.Set(robotData.controllerData.sRYStick * 0.25);

    // frc::SmartDashboard::PutNumber("RIGHT JOYSTICK", robotData.controllerData.sRYStick);


    if (robotData.controlData.mForceZeroPivot)
    {
        ForceZeroPivot();
    }

    if (robotData.controlData.mForceZeroWrist)
    {
        ForceZeroWrist();
    }
}

void Arm::RotatePivot(double targetDegree, const RobotData& robotData, double timeOffset)
{
    pivotProfileActive = true;
    pivotProfileStartTime = robotData.timerData.secSinceEnabled + timeOffset;

    if (pivotRunMode == ARM_ABSOLUTE_RUN)
    {
        pivotProfileStartPos = armPivotRelativeEncoder.GetPosition();
        pivotProfileEndPos = targetDegree;
    }
    else
    {
        pivotProfileStartPos = armPivotRelativeEncoder.GetPosition();
        pivotProfileEndPos = targetDegree;
    }

    pivotProfile = frc::TrapezoidProfile<units::degrees>
    {
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 450_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{pivotProfileEndPos}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{pivotProfileStartPos}, units::angular_velocity::degrees_per_second_t{0}}
    };

}

void Arm::RotateWrist(double targetDegree, const RobotData& robotData, double timeOffset)
{
    wristProfileActive = true;
    wristProfileStartTime = robotData.timerData.secSinceEnabled + timeOffset;
    
    if (wristRunMode == ARM_RELATIVE_RUN)
    {
        wristProfileStartPos = armWristRelativeEncoder.GetPosition();
        wristProfileEndPos = targetDegree;
    }
    else
    {
        wristProfileStartPos = armWristRelativeEncoder.GetPosition();
        wristProfileEndPos = targetDegree;
    }

    wristProfile = frc::TrapezoidProfile<units::degrees>
    {
        frc::TrapezoidProfile<units::degrees>::Constraints{300_deg_per_s, 250_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{wristProfileEndPos}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{wristProfileStartPos}, units::angular_velocity::degrees_per_second_t{0}}
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
        wristRunMode = ARM_ABSOLUTE_RUN;
    }
    else 
    {
        armData.wristAbsoluteInitialized = false;
        wristRunMode = ARM_NONE;
    }

    return armData.wristAbsoluteInitialized;
}

bool Arm::IsPivotAbolsoluteEncoderInitialized(ArmData& armData)
{
    if (armPivotAbsoluteEncoder.GetPosition() >= 0.01)
    {
        armData.pivotAbsoluteInitialized = true;
        pivotRunMode = ARM_ABSOLUTE_RUN;
    }
    else 
    {
        armData.pivotAbsoluteInitialized = false;
        pivotRunMode = ARM_NONE;
    }

    return armData.pivotAbsoluteInitialized;
}

void Arm::DisabledInit()
{

}

void Arm::DisabledPeriodic(const RobotData &robotData, ArmData &armData)
{
    ZeroRelativePositionPivot(armData);
    ZeroRelativePositionWrist(armData);
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

    armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armWristMinPosition + 5);
    armWrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armWristMaxPosition - 5);

    armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    armWrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    wristSoftLimitsToggled = true;
}

void Arm::EnablePivotSoftLimits()
{

    armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armPivotMinPosition + 3);
    armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armPivotMaxPosition - 3);

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