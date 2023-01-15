#include "subsystems/Intake.h"
#include "RobotData.h"
#include <cmath>

void Intake::RobotInit()
{
    // Intake Rollers
    intakeRollers.RestoreFactoryDefaults();
    intakeRollers.SetInverted(true);
    intakeRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    intakeRollers.SetSmartCurrentLimit(45);
    intakeRollers.EnableVoltageCompensation(10.5);
    intakeRollers2.RestoreFactoryDefaults();
    intakeRollers2.SetInverted(true);
    intakeRollers2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    intakeRollers2.SetSmartCurrentLimit(45);
    intakeRollers2.EnableVoltageCompensation(10.5);
}

void Intake::RobotPeriodic(const RobotData &robotData, IntakeData &intakeData)
{
    // changing controls based off the mode robot is in
    switch (robotData.controlData.mode) 
    {
        case MODE_TELEOP_MANUAL:
            Manual(robotData, intakeData);
            break;
        case MODE_TELEOP_SA:
            SemiAuto(robotData, intakeData);
            break;
        default:
            SemiAuto(robotData, intakeData);
            break;
    }
}

void Intake::SemiAuto(const RobotData &robotData, IntakeData &intakeData)
{
    if (robotData.controlData.saCubeIntake) 
    {
        IntakeRollers(intakeRollerInwardSpeed);

    }
    else if (robotData.controlData.saIntakeBackwards) 
    {
        IntakeRollers(intakeRollerOutwardSpeed);
    }
    else if (robotData.controlData.saCubeIntake)
    {
        IntakeRollers(intakeRollerCubeInwardSpeed);
    }
    else
    {
        IntakeRollers(0);
    }
}

void Intake::Manual(const RobotData &robotData, IntakeData &intakeData)
{
    if (robotData.controlData.mIntakeRollersIn) 
    {
        IntakeRollers(intakeRollerInwardSpeed);
    }
    else if (robotData.controlData.mIntakeRollersOut) 
    {
        IntakeRollers(intakeRollerOutwardSpeed);
    }
}

/*
* @param rollerSpeed Desired intake roller speed (0 - 1)
*/
void Intake::IntakeRollers(double rollerSpeed)
{
    intakeRollers.Set(rollerSpeed);
    intakeRollers2.Set(-rollerSpeed);
}
