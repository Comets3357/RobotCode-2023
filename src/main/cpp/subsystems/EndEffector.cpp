#include "subsystems/EndEffector.h"
#include "RobotData.h"
#include <cmath>

void EndEffector::RobotInit(const RobotData &robotData)
{
    // End Effector Rollers
    endEffectorRollers.RestoreFactoryDefaults();
    endEffectorRollers.SetInverted(robotData.configData.endEffectorConfigData.invertRollers);
    endEffectorRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    endEffectorRollers.SetSmartCurrentLimit(robotData.configData.endEffectorConfigData.currentLimit);
    endEffectorRollers.EnableVoltageCompensation(robotData.configData.endEffectorConfigData.voltageComp);

    coneLimitSwitch.EnableLimitSwitch(false);
    cubeLimitSwitch.EnableLimitSwitch(false);
    
    endEffectorRollers.BurnFlash();

}

void EndEffector::RobotPeriodic(const RobotData &robotData, EndEffectorData &endEffectorData)
{
    // changing controls based off the mode robot is in
    switch (robotData.controlData.mode) 
    {
        case MODE_TELEOP_MANUAL:
            Manual(robotData, endEffectorData);
            break;
        case MODE_TELEOP_SA:
            SemiAuto(robotData, endEffectorData);
            break;
        default:
            SemiAuto(robotData, endEffectorData);
            break;
    }

    endEffectorData.pastReadOfGamePiece = endEffectorData.gamePieceType;

    if (cubeLimitSwitch.Get())
    {
        if (robotData.armData.wristSafeCubeDetectionPosition)
        {
            endEffectorData.lastPieceType = CUBE;
            endEffectorData.gamePieceType = CUBE;
        }
        else if (!robotData.armData.wristSafeCubeDetectionPosition && coneLimitSwitch.Get())
        {
            endEffectorData.lastPieceType = CONE;
            endEffectorData.gamePieceType = CONE;            
        }
        else
        {
            endEffectorData.gamePieceType = NONE;
        }
    }
    else if (coneLimitSwitch.Get())
    {
        endEffectorData.lastPieceType = CONE;
        endEffectorData.gamePieceType = CONE;
    }
    else if (!robotData.controlData.saConeIntake || !robotData.controlData.saCubeIntake || !robotData.controlData.saElevatorSetHumanPlayerPosition || !robotData.controlData.saUprightConeIntake)
    {
        
        endEffectorData.gamePieceType = NONE;
    }


}

void EndEffector::SemiAuto(const RobotData &robotData, EndEffectorData &endEffectorData)
{
    endEffectorData.armRetractRequest = false;
    if (robotData.controlData.saPositionHumanPlayer)
    {
        if (endEffectorData.gamePieceType != CONE)
        {
            SetEndEffectorRollerSpeed(EndEffectorRollerInwardSpeed);
        }
        else
        {
            SetEndEffectorRollerSpeed(-0.05);
        }
        
    }
    else if (robotData.controlData.saConeIntake || robotData.controlData.saUprightConeIntake) 
    {
        if (endEffectorData.gamePieceType != CONE)
        {
            SetEndEffectorRollerSpeed(EndEffectorRollerInwardSpeed);    
        }

    }
    else if (robotData.controlData.saIntakeBackwards) 
    {
        switch (robotData.endEffectorData.lastPieceType)
        {
            case CONE:
                SetEndEffectorRollerSpeed(EndEffectorRollerOutwardSpeed);  
                break;
            case CUBE:
                SetEndEffectorRollerSpeed(-EndEffectorRollerOutwardSpeed);
                break;
        }
        // if (endEffectorData.gamePieceType == NONE)
        // {
        //     eject = true;
        // }
    }
    else if (robotData.controlData.saCubeIntake)
    {
        if (endEffectorData.gamePieceType != CUBE)
        {
            SetEndEffectorRollerSpeed(EndEffectorRollerCubeInwardSpeed);
        }
    }
    else
    {
        switch (robotData.endEffectorData.gamePieceType)
        {
            case CONE:
                SetEndEffectorRollerSpeed(-0.05);
                break;
            case CUBE:
                SetEndEffectorRollerSpeed(0.08);
                break;
            case NONE:
                SetEndEffectorRollerSpeed(0.0);
                break;
            default:
                SetEndEffectorRollerSpeed(0.0);
                break;
        }
    }

    if (robotData.controllerData.sLYStick > 0.9)
    {
        SetEndEffectorRollerSpeed(-EndEffectorRollerOutwardSpeed);
    }
    else if (robotData.controllerData.sLYStick < -0.9)
    {
        SetEndEffectorRollerSpeed(EndEffectorRollerOutwardSpeed);
    }
    // if (eject == true)
    // {
    //     endEffectorData.armRetractRequest = true;
    //     eject = false;
    // }
    frc::SmartDashboard::PutNumber("BHASIUDGUISAD", endEffectorData.gamePieceType);
}

void EndEffector::Manual(const RobotData &robotData, EndEffectorData &endEffectorData)
{

    if (robotData.controlData.mEndEffectorRollersIn) 
    {
        SetEndEffectorRollerSpeed(EndEffectorRollerInwardSpeed);
    }
    else if (robotData.controlData.mEndEffectorRollersOut) 
    {
        SetEndEffectorRollerSpeed(EndEffectorRollerOutwardSpeed);
    }
    else 
    {
        SetEndEffectorRollerSpeed(0);
    }
}

/*
* @param rollerSpeed Desired endEffector roller speed (0 - 1)
*/
void EndEffector::SetEndEffectorRollerSpeed(double rollerSpeed)
{
    endEffectorRollers.Set(rollerSpeed);
}

void EndEffector::DisabledPeriodic()
{
    frc::SmartDashboard::PutBoolean("End Effector Inverted", endEffectorRollers.GetInverted());
}