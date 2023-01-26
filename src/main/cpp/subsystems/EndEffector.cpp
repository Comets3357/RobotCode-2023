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
}

void EndEffector::SemiAuto(const RobotData &robotData, EndEffectorData &endEffectorData)
{
    if (robotData.controlData.saConeIntake) 
    {
        SetEndEffectorRollerSpeed(EndEffectorRollerInwardSpeed);

    }
    else if (robotData.controlData.saIntakeBackwards) 
    {
        SetEndEffectorRollerSpeed(EndEffectorRollerOutwardSpeed);
    }
    else if (robotData.controlData.saCubeIntake)
    {
        SetEndEffectorRollerSpeed(EndEffectorRollerCubeInwardSpeed);
    }
    else
    {
        SetEndEffectorRollerSpeed(0);
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
