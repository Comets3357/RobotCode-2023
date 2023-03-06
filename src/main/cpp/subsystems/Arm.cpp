#include "subsystems/Arm.h"
#include "RobotData.h"

void Arm::RobotInit(const RobotData &robotData, ArmData &armData)
{
    // Wrist Initialization
    armWristPIDController.SetP(robotData.configData.armConfigData.wristP, 0); // 0.35
    armWristPIDController.SetI(0, 0);
    armWristPIDController.SetD(0, 0);
    armWristPIDController.SetIZone(0, 0);
    armWristPIDController.SetFF(0, 0);
    armWristPIDController.SetOutputRange(-1, 1, 0);
    
    armWrist.EnableVoltageCompensation(robotData.configData.armConfigData.voltageComp);
    armWrist.SetSmartCurrentLimit(robotData.configData.armConfigData.wristCurrentLimit);
    armWrist.SetInverted(false);//robotData.configData.armConfigData.wristRelativeInverted); 
    armWrist.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    armWristAbsoluteEncoder.SetInverted(false);//robotData.configData.armConfigData.wristAbsoluteInverted);
    armWristAbsoluteEncoder.SetPositionConversionFactor(robotData.configData.armConfigData.wristAbsoluteConversion);
    armWristAbsoluteEncoder.SetZeroOffset(robotData.configData.armConfigData.wristAbsoluteOffset);

    armWristRelativeEncoder.SetPositionConversionFactor(robotData.configData.armConfigData.wristRelativeConversion);
    armWristRelativeEncoder.SetPosition(10);

    // armWristPIDController.SetFeedbackDevice(armWristRelativeEncoder);

    armWrist.BurnFlash();

    armPivotPIDController.SetP(robotData.configData.armConfigData.pivotP, 0);
    armPivotPIDController.SetI(0, 0);
    armPivotPIDController.SetD(0, 0);
    armPivotPIDController.SetIZone(0, 0);
    armPivotPIDController.SetFF(0, 0);

    armPivotPIDController.SetOutputRange(-1, 1, 0);
    armPivot.EnableVoltageCompensation(robotData.configData.armConfigData.voltageComp);
    armPivot.SetSmartCurrentLimit(robotData.configData.armConfigData.pivotCurrentLimit);
    armPivot.SetInverted(false);//robotData.configData.armConfigData.pivotRelativeInverted);
    armPivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    armPivotAbsoluteEncoder.SetInverted(true);//robotData.configData.armConfigData.pivotAbsoluteInverted);
    armPivotAbsoluteEncoder.SetPositionConversionFactor(robotData.configData.armConfigData.pivotAbsoluteConversion);
    armPivotAbsoluteEncoder.SetZeroOffset(robotData.configData.armConfigData.pivotAbsoluteOffset);
    armPivotRelativeEncoder.SetPositionConversionFactor(robotData.configData.armConfigData.pivotRelativeConversion);
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
    frc::SmartDashboard::PutNumber("elbowSpeed", armPivot.Get());
    frc::SmartDashboard::PutNumber("wristSpeed", armWrist.Get());

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

    if (robotData.controllerData.sLStickBtn)
    {
        ZeroRelativePositionPivot(armData);
    }

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
    coneIntakeToggle = armData.coneIntakeRunning;
    cubeIntakeToggle = armData.cubeIntakeRunning;
    uprightConeIntakeToggle = armData.uprightConeIntakeRunning;
    humanPlayerIntakeToggle = armData.humanPlayerConeIntakeRunning;

    armData.coneIntakeRunning = robotData.controlData.saConeIntake;
    armData.cubeIntakeRunning = robotData.controlData.saCubeIntake;
    armData.uprightConeIntakeRunning = robotData.controlData.saUprightConeIntake;
    armData.humanPlayerConeIntakeRunning = robotData.controlData.saPositionHumanPlayer;

    endEffectorGamePiecePastRead = endEffectorGamePiece;

    wristInPositionForArmPastRead = wristInPositionForArm;
    wristInPositionForArm = armWristRelativeEncoder.GetPosition() < 100;
    if (frc::DriverStation::IsTeleop())
    {
        inAuton = false;
        if ((std::abs(armWristRelativeEncoder.GetPosition() - 202.5) < 15) && (std::abs(armPivotRelativeEncoder.GetPosition() - 42) < 7))
        {
            armData.wristAndArmInPositionForBullBarIntake = true;
        }
        else
        {
            armData.wristAndArmInPositionForBullBarIntake = false;
        }
    }
    else
    {
        armData.wristAndArmInPositionForBullBarIntake = true;
        if (frc::DriverStation::IsAutonomous())
        {
            inAuton = true;
        }
    }


    if (robotData.endEffectorData.gamePieceType == CONE || robotData.endEffectorData.gamePieceType == CUBE)
    {
        endEffectorGamePiece = true;
    }
    else
    {
        endEffectorGamePiece = false;
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
        if (robotData.controlData.saPositionHumanPlayer)
        {
            if (humanPlayerIntakeToggle != armData.humanPlayerConeIntakeRunning)
            {
                RotateWrist(29, robotData, 0);
                RotatePivot(10, robotData, 0);
            }
            
        }
        if (robotData.controlData.saSetUpPosition)
        {
            RotateWrist(30, robotData, 0);
            RotatePivot(80, robotData, 0);
        }
        switch (robotData.endEffectorData.gamePieceType)
        {

            case CONE:
            
                if (robotData.controlData.saPositionMid)
                {
                    RotateWrist(10, robotData, 0);
                    RotatePivot(145, robotData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {
                    RotateWrist(18, robotData, 0);
                    RotatePivot(147, robotData, 0);
                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(11, robotData, 0);
                }
                else if (robotData.controlData.saPositionLow)
                {
                    RotatePivot(16, robotData, 0);
                    RotateWrist(130, robotData, 0);
                }
                break;

            case CUBE:

                if (robotData.controlData.saPositionMid)
                {
                    RotateWrist(50, robotData, 0);
                    RotatePivot(146, robotData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {

                    RotateWrist(45, robotData, 0);
                    RotatePivot(146, robotData, 0);

                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(11, robotData, 0);  
                }
                else if (robotData.controlData.saPositionLow)
                {
                    RotatePivot(16, robotData, 0);
                    RotateWrist(130, robotData, 0);
                }
                break;

            case NONE:

                if (robotData.controlData.saPositionMid)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(146, robotData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {
                    RotateWrist(30, robotData, 0.2);
                    RotatePivot(140, robotData, 0.2);
                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(11, robotData, 0);
                }
                else if (robotData.controlData.saPositionLow)
                {
                    RotatePivot(16, robotData, 0);
                    RotateWrist(130, robotData, 0);
                }
                break;

            default:

                if (robotData.controlData.saPositionMid)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(146, robotData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {
                    RotateWrist(30, robotData, 0.2);
                    RotatePivot(140, robotData, 0.2);
                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(30, robotData, 0);
                    RotatePivot(11, robotData, 0);
                }
                else if (robotData.controlData.saPositionLow)
                {
                    RotatePivot(16, robotData, 0);
                    RotateWrist(130, robotData, 0);
                }
                break;
        }

        if (robotData.controlData.saConeIntake)
        {
            if (robotData.bullBarData.bullBarSafePosition)
            {
                if (readyRunBasedOffBullBar != robotData.bullBarData.bullBarSafePosition)
                {
                    RotatePivot(10, robotData, 0);
                    RotateWrist(120, robotData, 0);
                }
            }
            else if (!robotData.bullBarData.bullBarSafePosition && (coneIntakeToggle != armData.coneIntakeRunning))
            {
                RotateWrist(50, robotData, 0);
            }
            readyRunBasedOffBullBar = robotData.bullBarData.bullBarSafePosition;
        }
        else if (!robotData.controlData.saCubeIntake)
        {
            if (cubeIntakeToggle != armData.cubeIntakeRunning)
            {
                RotateWrist(20, robotData, 0);
            }

            if (wristInPositionForArmPastRead != wristInPositionForArm && armWristRelativeEncoder.GetPosition() < 100)
            {
                RotatePivot(20, robotData, 0);
            }
        }

        if (robotData.controlData.saUprightConeIntake)
        {
            if (robotData.bullBarData.bullBarUprightConeSafePosition)
            {
                if (bullBarIn != robotData.bullBarData.bullBarUprightConeSafePosition)
                {
                    RotatePivot(16, robotData, 0);
                    RotateWrist(130, robotData, 0);
                }
            }
            else if (!robotData.bullBarData.bullBarUprightConeSafePosition && (uprightConeIntakeToggle != armData.uprightConeIntakeRunning))
            {
                RotateWrist(50, robotData, 0);
            }
            bullBarIn = robotData.bullBarData.bullBarUprightConeSafePosition;
        }
        else if (!robotData.controlData.saUprightConeIntake)
        {
            bullBarIn = false;
            if (uprightConeIntakeToggle != armData.uprightConeIntakeRunning)
            {
                RotateWrist(30, robotData, 0);
                RotatePivot(11, robotData, 0);
            }
        }

        if (robotData.controlData.saCubeIntake)
        {
            if (cubeIntakeToggle != armData.cubeIntakeRunning)
            {
                RotatePivot(42, robotData, 0);
                RotateWrist(199.5+3, robotData, 0);
            }
            readyRunBasedOffBullBar = robotData.bullBarData.bullBarSafePosition;
        }
        else if (!robotData.controlData.saConeIntake)
        {
            if (coneIntakeToggle != armData.coneIntakeRunning)
            {
                RotateWrist(30, robotData, 0);
                RotatePivot(55, robotData, 0);
            }

            if ((wristInPositionForArmPastRead != wristInPositionForArm) && armWristRelativeEncoder.GetPosition() < 100)
            {
                pivotMaxAcceleration = 100_deg;
                RotatePivot(11, robotData, 0);
                pivotMaxAcceleration = 700_deg;
            }
        }

        if (armWristRelativeEncoder.GetPosition() < 50 || armPivotRelativeEncoder.GetPosition() > 70)
        {
            armData.wristSafePosition = true;
        }
        else 
        {
            armData.wristSafePosition = false;
        }
        
        frc::SmartDashboard::PutBoolean("arm in safe position", armData.wristSafePosition);
        frc::SmartDashboard::PutNumber("WRIST CURRENT POSITION", armWristRelativeEncoder.GetPosition());
    }
    else
    {
        armWrist.Set(0);
        armPivot.Set(0);
    }

    // if (robotData.controllerData.sXBtn)
    // {
    //     ZeroRelativePositionPivot(armData);
    //     ZeroRelativePositionWrist(armData);
    // }

    // if ((robotData.endEffectorData.pastReadOfGamePiece != NONE) && (robotData.endEffectorData.gamePieceType == NONE))
    // {
    //     RotateWrist(30, robotData, 0.25);
    //     RotatePivot(11, robotData, 0.25);
    // }
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
    if (wristSoftLimitsToggled)
    {
        DisableWristSoftLimits();
    }

    if (pivotSoftLimitsToggled)
    {
        DisablePivotSoftLimits();
    }
    // EnablePivotSoftLimits();
    // EnablePivotSoftLimits();

    if (robotData.controlData.mMovePivot)
    {
        armPivot.Set(robotData.controllerData.sRYStick * 0.3);
    }
    else
    {
        armPivot.Set(0);
    }

    if (robotData.controlData.mMoveWrist)
    {
        armWrist.Set(robotData.controllerData.sLYStick * 0.25);
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
        frc::TrapezoidProfile<units::degrees>::Constraints{720_deg_per_s, pivotMaxAcceleration/(1_s * 1_s)},
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
        frc::TrapezoidProfile<units::degrees>::Constraints{400_deg_per_s, 350_deg/(1_s * 1_s)},
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
    if (!inAuton)
    {
        ZeroRelativePositionPivot(armData);
        ZeroRelativePositionWrist(armData);
    }

    frc::SmartDashboard::PutBoolean("Pivot Relative Inverted", !armPivot.GetInverted());
    frc::SmartDashboard::PutBoolean("Pivot Absolute Inverted", armPivotAbsoluteEncoder.GetInverted());
    frc::SmartDashboard::PutBoolean("Wrist Relative Inverted", !armWrist.GetInverted());
    frc::SmartDashboard::PutBoolean("Wrist Absolute Inverted", !armWristAbsoluteEncoder.GetInverted());
    
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

    armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, armPivotMinPosition + 5);
    armPivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, armPivotMaxPosition - 5);

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