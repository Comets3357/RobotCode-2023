#include "subsystems/EndEffector.h"
#include "RobotData.h"
#include <cmath>

void EndEffector::RobotInit()
{
    // Intake Rollers
    intakeRollers.RestoreFactoryDefaults();
    intakeRollers.SetInverted(true);
    intakeRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    intakeRollers.SetSmartCurrentLimit(45);
    intakeRollers.EnableVoltageCompensation(10.5);
    intakeRollers.BurnFlash();

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
        SetIntakeRollerSpeed(intakeRollerInwardSpeed);

    }
    else if (robotData.controlData.saIntakeBackwards) 
    {
        SetIntakeRollerSpeed(intakeRollerOutwardSpeed);
    }
    else if (robotData.controlData.saCubeIntake)
    {
        SetIntakeRollerSpeed(intakeRollerCubeInwardSpeed);
    }
    else
    {
        SetIntakeRollerSpeed(0);
    }
}

void EndEffector::Manual(const RobotData &robotData, EndEffectorData &endEffectorData)
{

    if (robotData.controlData.mIntakeRollersIn) 
    {
        SetIntakeRollerSpeed(intakeRollerInwardSpeed);
    }
    else if (robotData.controlData.mIntakeRollersOut) 
    {
        SetIntakeRollerSpeed(intakeRollerOutwardSpeed);
    }
}

/*
* @param rollerSpeed Desired intake roller speed (0 - 1)
*/
void EndEffector::SetIntakeRollerSpeed(double rollerSpeed)
{
    intakeRollers.Set(rollerSpeed);
}
