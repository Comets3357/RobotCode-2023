#include "subsystems/Arm.h"
#include "RobotData.h"

void Arm::RobotInit(const RobotData &robotData, ArmData &armData)
{
    if (
        armWristPIDController.GetP() != robotData.configData.armConfigData.wristP ||
        armWrist.GetInverted() != robotData.configData.armConfigData.wristRelativeInverted ||
        armWrist.GetIdleMode() != rev::CANSparkMax::IdleMode::kBrake ||
        armWristAbsoluteEncoder.GetInverted() != robotData.configData.armConfigData.wristAbsoluteInverted || 
        armWristAbsoluteEncoder.GetPositionConversionFactor() != robotData.configData.armConfigData.wristAbsoluteConversion ||
        armWristAbsoluteEncoder.GetZeroOffset() != robotData.configData.armConfigData.wristAbsoluteOffset ||
        armWristRelativeEncoder.GetPositionConversionFactor() != robotData.configData.armConfigData.wristRelativeConversion
    )
    {
        // Wrist Initialization

        armWrist.RestoreFactoryDefaults();

        armWristPIDController.SetP(robotData.configData.armConfigData.wristP, 0); // 0.35
        armWristPIDController.SetI(0, 0);
        armWristPIDController.SetD(0, 0);
        armWristPIDController.SetIZone(0, 0);
        armWristPIDController.SetFF(0, 0);
        armWristPIDController.SetOutputRange(-1, 1, 0);
        
        armWrist.EnableVoltageCompensation(robotData.configData.armConfigData.voltageComp);
        armWrist.SetSmartCurrentLimit(robotData.configData.armConfigData.wristCurrentLimit);
        armWrist.SetInverted(robotData.configData.armConfigData.wristRelativeInverted); 
        armWrist.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

        armWristAbsoluteEncoder.SetInverted(robotData.configData.armConfigData.wristAbsoluteInverted);
        armWristAbsoluteEncoder.SetPositionConversionFactor(robotData.configData.armConfigData.wristAbsoluteConversion);
        armWristAbsoluteEncoder.SetZeroOffset(robotData.configData.armConfigData.wristAbsoluteOffset);

        armWristRelativeEncoder.SetPositionConversionFactor(robotData.configData.armConfigData.wristRelativeConversion);
        
        armWrist.BurnFlash();
    }


    if (
        armPivotPIDController.GetP() != robotData.configData.armConfigData.pivotP ||
        armPivot.GetInverted() != robotData.configData.armConfigData.pivotRelativeInverted ||
        armPivot.GetIdleMode() != rev::CANSparkMax::IdleMode::kBrake ||
        armPivotAbsoluteEncoder.GetInverted() != robotData.configData.armConfigData.pivotAbsoluteInverted || 
        armPivotAbsoluteEncoder.GetPositionConversionFactor() != robotData.configData.armConfigData.pivotAbsoluteConversion ||
        armPivotAbsoluteEncoder.GetZeroOffset() != robotData.configData.armConfigData.pivotAbsoluteOffset ||
        armPivotRelativeEncoder.GetPositionConversionFactor() != robotData.configData.armConfigData.pivotRelativeConversion
    )
    {

        armPivot.RestoreFactoryDefaults();

        armPivotPIDController.SetP(robotData.configData.armConfigData.pivotP, 0);
        armPivotPIDController.SetI(0, 0);
        armPivotPIDController.SetD(0, 0);
        armPivotPIDController.SetIZone(0, 0);
        armPivotPIDController.SetFF(0, 0);

        armPivotPIDController.SetOutputRange(-1, 1, 0);
        armPivot.EnableVoltageCompensation(robotData.configData.armConfigData.voltageComp);
        armPivot.SetSmartCurrentLimit(robotData.configData.armConfigData.pivotCurrentLimit);
        armPivot.SetInverted(robotData.configData.armConfigData.pivotRelativeInverted);
        armPivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

        armPivotAbsoluteEncoder.SetInverted(robotData.configData.armConfigData.pivotAbsoluteInverted);
        armPivotAbsoluteEncoder.SetPositionConversionFactor(robotData.configData.armConfigData.pivotAbsoluteConversion);
        armPivotAbsoluteEncoder.SetZeroOffset(robotData.configData.armConfigData.pivotAbsoluteOffset);
        armPivotRelativeEncoder.SetPositionConversionFactor(robotData.configData.armConfigData.pivotRelativeConversion);
        armPivotRelativeEncoder.SetPosition(10);

        armPivot.BurnFlash();
    }

    ZeroRelativePositionWrist(armData);
    ZeroRelativePositionPivot(armData);

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
        case MODE_TELEOP_DISABLE_BEAMS:
            SemiAuto(robotData, armData);
            break;
        default:
            SemiAuto(robotData, armData);
            break;
    }

    frc::SmartDashboard::PutBoolean("INPOS", armData.armInPosition);
    
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
    armData.pastDoubleSub = armData.currentDoubleSub;
    uprightConeIntakeToggle = armData.uprightConeIntakeRunning;
    humanPlayerIntakeToggle = armData.humanPlayerConeIntakeRunning;

    armData.coneIntakeRunning = robotData.controlData.saConeIntake;
    armData.cubeIntakeRunning = robotData.controlData.saCubeIntake;
    armData.currentDoubleSub = robotData.controlData.saDoubleSubCone;
    armData.uprightConeIntakeRunning = robotData.controlData.saUprightConeIntake;
    armData.humanPlayerConeIntakeRunning = robotData.controlData.saPositionHumanPlayer;

    endEffectorGamePiecePastRead = endEffectorGamePiece;

    wristInPositionForArmPastRead = wristInPositionForArm;
    wristInPositionForArm = armWristRelativeEncoder.GetPosition() < 100;
    if (frc::DriverStation::IsTeleop())
    {
        inAuton = false;
        if ((std::abs(armWristRelativeEncoder.GetPosition() - 202.5) < 20) && (std::abs(armPivotRelativeEncoder.GetPosition() - 42) < 10))
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
                RotateWrist(28, robotData, 0);
                RotatePivot(10, robotData, 0);
            }
            
        }
        if (robotData.controlData.saSetUpPosition)
        {
            RotateWrist(30, robotData, 0);
            RotatePivot(52, robotData, 0);
        }
        switch (robotData.endEffectorData.lastPieceType)
        {

            case CONE:
            
                if (robotData.controlData.saPositionMid)
                {
                    RotateWrist(robotData.configData.armConfigData.wristConeMidPosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotConeMidPosition, robotData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {
                    RotateWrist(robotData.configData.armConfigData.wristConeHighPosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotConeHighPosition, robotData, 0);
                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(robotData.configData.armConfigData.wristHomePosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotHomePosition, robotData, 0);
                }
                else if (robotData.controlData.saPositionLow)
                {
                    RotatePivot(robotData.configData.armConfigData.pivotLowPosition, robotData, 0);
                    RotateWrist(robotData.configData.armConfigData.wristLowPosition, robotData, 0);
                }
                break;

            case CUBE:

                if (robotData.controlData.saPositionMid)
                {
                    RotateWrist(robotData.configData.armConfigData.wristCubeMidPosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotCubeMidPosition, robotData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {

                    RotateWrist(robotData.configData.armConfigData.wristCubeHighPosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotCubeHighPosition, robotData, 0);

                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(robotData.configData.armConfigData.wristHomePosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotHomePosition, robotData, 0);  
                }
                else if (robotData.controlData.saPositionLow)
                {
                    RotatePivot(robotData.configData.armConfigData.pivotLowPosition, robotData, 0);
                    RotateWrist(robotData.configData.armConfigData.wristLowPosition, robotData, 0);
                }
                break;

            case NONE:

                if (robotData.controlData.saPositionMid)
                {
                    RotateWrist(robotData.configData.armConfigData.wristConeMidPosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotConeMidPosition, robotData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {
                    RotateWrist(robotData.configData.armConfigData.wristConeHighPosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotConeHighPosition, robotData, 0);
                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(robotData.configData.armConfigData.wristHomePosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotHomePosition, robotData, 0);
                }
                else if (robotData.controlData.saPositionLow)
                {
                    RotatePivot(robotData.configData.armConfigData.pivotLowPosition, robotData, 0);
                    RotateWrist(robotData.configData.armConfigData.wristLowPosition, robotData, 0);
                }
                break;

            default:

                if (robotData.controlData.saPositionMid)
                {
                    RotateWrist(robotData.configData.armConfigData.wristConeMidPosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotConeMidPosition, robotData, 0);
                }
                else if (robotData.controlData.saPositionHigh)
                {
                    RotateWrist(robotData.configData.armConfigData.wristConeHighPosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotConeHighPosition, robotData, 0);
                }
                else if (robotData.controlData.saHomePosition)
                {
                    RotateWrist(robotData.configData.armConfigData.wristHomePosition, robotData, 0);
                    RotatePivot(robotData.configData.armConfigData.pivotHomePosition, robotData, 0);
                }
                else if (robotData.controlData.saPositionLow)
                {
                    RotatePivot(robotData.configData.armConfigData.pivotLowPosition, robotData, 0);
                    RotateWrist(robotData.configData.armConfigData.wristLowPosition, robotData, 0);
                }
                break;
        }

        if (robotData.controlData.saConeIntake)
        {
            if (robotData.bullBarData.bullBarSafePosition)
            {
                if (readyRunBasedOffBullBar != robotData.bullBarData.bullBarSafePosition)
                {
                    RotatePivot(robotData.configData.armConfigData.pivotConeIntakePosition, robotData, 0);
                    RotateWrist(robotData.configData.armConfigData.wristConeIntakePosition, robotData, 0);
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
                RotateWrist(25, robotData, 0);
                RotatePivot(60, robotData, 0.1);
            }

            if ((wristInPositionForArmPastRead != wristInPositionForArm) && armWristRelativeEncoder.GetPosition() < 100)
            {
                pivotMaxAcceleration = 100_deg;
                RotatePivot(11, robotData, 0);
                pivotMaxAcceleration = 700_deg;
            }
        }

        if (robotData.controlData.saUprightConeIntake)
        {
            if (robotData.bullBarData.bullBarUprightConeSafePosition)
            {
                if (bullBarIn != robotData.bullBarData.bullBarUprightConeSafePosition)
                {
                    RotatePivot(robotData.configData.armConfigData.pivotUprightConePosition, robotData, 0);
                    RotateWrist(robotData.configData.armConfigData.wristUprightConePosition, robotData, 0);
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
                RotatePivot(robotData.configData.armConfigData.pivotCubeIntakePosition, robotData, 0);
                RotateWrist(robotData.configData.armConfigData.wristCubeIntakePosition, robotData, 0);
            }
            readyRunBasedOffBullBar = robotData.bullBarData.bullBarSafePosition;
        }
        else if (!robotData.controlData.saConeIntake)
        {
            if (coneIntakeToggle != armData.coneIntakeRunning)
            {
                RotateWrist(40, robotData, 0);
                RotatePivot(55, robotData, 0);
            }

            if ((wristInPositionForArmPastRead != wristInPositionForArm) && armWristRelativeEncoder.GetPosition() < 100)
            {
                pivotMaxAcceleration = 100_deg;
                RotatePivot(11, robotData, 0);
                pivotMaxAcceleration = 700_deg;
            }
        }

        if (robotData.controlData.saDoubleSubCone)
        {
            if (robotData.armData.currentDoubleSub != robotData.armData.pastDoubleSub)
            {
                RotateWrist(175, robotData, 0);
                RotatePivot(83, robotData, 0);
            }
        }
        else if (!robotData.controlData.saDoubleSubCone)
        {
            if (robotData.armData.pastDoubleSub != armData.currentDoubleSub)
            {
                RotateWrist(25, robotData, 0);
                RotatePivot(90, robotData, 0.1);
            }

            if ((wristInPositionForArmPastRead != wristInPositionForArm) && armWristRelativeEncoder.GetPosition() < 100)
            {
                pivotMaxAcceleration = 200_deg;
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

        wristElapsedTime = units::time::second_t{robotData.timerData.secSinceEnabled - wristProfileStartTime};
        auto setPoint = wristProfile.Calculate(wristElapsedTime);

        armWristPIDController.SetReference(setPoint.position.value(), rev::CANSparkMax::ControlType::kPosition);
        frc::SmartDashboard::PutNumber("TRAP Wrist", setPoint.position.value());
        frc::SmartDashboard::PutNumber("TRAP Active", wristProfileActive);
        frc::SmartDashboard::PutNumber("TRAP Wrist Elapsed Time", (double)wristElapsedTime);

        if (wristProfile.IsFinished(wristElapsedTime))
        {
            wristProfileActive = false;
            wristElapsedTime = units::time::second_t{0};
        }
        
    }

    if (pivotProfileActive && robotData.timerData.secSinceEnabled > pivotProfileStartTime)
    {
        pivotElapsedTime = units::time::second_t{robotData.timerData.secSinceEnabled - pivotProfileStartTime};
        auto setpoint = pivotProfile.Calculate(pivotElapsedTime);

        armPivotPIDController.SetReference(setpoint.position.value(), rev::CANSparkMax::ControlType::kPosition);
        frc::SmartDashboard::PutNumber("TRAP Arm", setpoint.position.value());

        if (pivotProfile.IsFinished(pivotElapsedTime))
        {
            pivotProfileActive = false;
            pivotElapsedTime = units::time::second_t{0};
        }
    }

    if (!pivotProfileActive && !wristProfileActive) armData.armInPosition = true;
    else armData.armInPosition = false;

    if (robotData.controllerData.sLStickBtn)
    {
        zeroing = true;
        armPivot.Set(0);
        zeroStartTime = robotData.timerData.secSinceEnabled;
        ZeroRelativePositionPivot(armData);
    }

    if (zeroing)
    {
        armPivot.Set(0);
    }

    if (zeroing && robotData.timerData.secSinceEnabled-1.0 > zeroStartTime)
    {
        zeroing = false;
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
        frc::TrapezoidProfile<units::degrees>::Constraints{600_deg_per_s, 550_deg/(1_s * 1_s)},
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
    if (!inAuton && (armPivotRelativeEncoder.GetPosition() < 120))
    {
        ZeroRelativePositionPivot(armData);
        ZeroRelativePositionWrist(armData);
    }

    frc::SmartDashboard::PutBoolean("Pivot Relative Inverted", !armPivot.GetInverted());
    frc::SmartDashboard::PutBoolean("Pivot Absolute Inverted", armPivotAbsoluteEncoder.GetInverted());
    frc::SmartDashboard::PutBoolean("Wrist Relative Inverted", armWrist.GetInverted());
    frc::SmartDashboard::PutBoolean("Wrist Absolute Inverted", armWristAbsoluteEncoder.GetInverted());
    
}
void Arm::UpdateData(const RobotData &robotData, ArmData &armData)
{
    // frc::SmartDashboard::PutBoolean("Arm Pivot Initialized", robotData.armData.pivotAbsoluteInitialized);
    // frc::SmartDashboard::PutBoolean("Arm Wrist Initialized", robotData.armData.wristAbsoluteInitialized);
    // forceZeroPivot = frc::SmartDashboard::GetBoolean("Force Zero Pivot", false);
    // forceZeroWrist = frc::SmartDashboard::GetBoolean("Force Zero Wrist", false);
    // frc::SmartDashboard::PutBoolean("Pivot Zeroed", pivotForceZeroed);
    // frc::SmartDashboard::PutBoolean("Wrist Zeroed", wristForceZeroed);
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