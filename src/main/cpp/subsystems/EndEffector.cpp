#include "subsystems/EndEffector.h"
#include "RobotData.h"
#include <cmath>

void EndEffector::RobotInit()
{
    // End Effector Rollers
    endEffectorRollers.RestoreFactoryDefaults();
    endEffectorRollers.SetInverted(true);
    endEffectorRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    endEffectorRollers.SetSmartCurrentLimit(45);
    endEffectorRollers.EnableVoltageCompensation(10.5);

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

    // if (coneLimitSwitch.Get())
    // {
    //     endEffectorData.isCone = true;
    //     endEffectorData.isCube = false;
    // }
    // else if (cubeLimitSwitch.Get())
    // {
    //     endEffectorData.isCone = false;
    //     endEffectorData.isCube = true;
    // }
    // else
    // {
    //     endEffectorData.isCone = false;
    //     endEffectorData.isCube = false;
    // }


}

void EndEffector::SemiAuto(const RobotData &robotData, EndEffectorData &endEffectorData)
{
    if (robotData.controlData.saPositionHumanPlayer)
    {
        SetEndEffectorRollerSpeed(0.5);
    }
    else if (robotData.controlData.saConeIntake) 
    {
        SetEndEffectorRollerSpeed(EndEffectorRollerInwardSpeed);

    }
    else if (robotData.controlData.saIntakeBackwards) 
    {
        if (robotData.armData.isNotCone)
        {
            SetEndEffectorRollerSpeed(-EndEffectorRollerOutwardSpeed);
        }
        else
        {
            SetEndEffectorRollerSpeed(EndEffectorRollerOutwardSpeed);   
        }
    }
    else if (robotData.controlData.saCubeIntake)
    {
        SetEndEffectorRollerSpeed(EndEffectorRollerCubeInwardSpeed);
    }
    
    else
    {
        if (robotData.armData.isNotCone)
        {
            SetEndEffectorRollerSpeed(0.05);
        }
        else
        {
            SetEndEffectorRollerSpeed(-0.05);
        }
        
    }
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
