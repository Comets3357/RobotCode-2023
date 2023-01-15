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
    intakeRollers2.RestoreFactoryDefaults();
    intakeRollers2.SetInverted(true);
    intakeRollers2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    intakeRollers2.SetSmartCurrentLimit(45);
    intakeRollers2.EnableVoltageCompensation(10.5);

    // Intake Pivot
    intakePivotPIDController.SetP(0.1, 0);
    intakePivotPIDController.SetI(0, 0);
    intakePivotPIDController.SetD(0, 0);
    intakePivotPIDController.SetIZone(0, 0);
    intakePivotPIDController.SetFF(0, 0);
    intakePivotPIDController.SetOutputRange(-1, 1, 0);
    intakePivot.EnableVoltageCompensation(10.5);
    intakePivot.SetSmartCurrentLimit(20);
    intakePivot.BurnFlash();

    // Callibrating Relative Based On Absolute Position
    ZeroIntake();
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

    // reseeding the position of relative if the motor isn't running
    if (intakePivotRelativeEncoder.GetVelocity() < 1)
    {
        ZeroIntake();
    }

}

void EndEffector::SemiAuto(const RobotData &robotData, EndEffectorData &endEffectorData)
{
    if (!softLimitsToggled) 
    {
        ToggleSoftLimits();
    }

    if (robotData.controlData.saIntaking) 
    {
        IntakeRollers(intakeRollerInwardSpeed);
        IntakePivot(intakePivotRelativeMaxPosition);

    }
    else if (robotData.controlData.saIntakeBackwards) 
    {
        IntakeRollers(intakeRollerOutwardSpeed);
        IntakePivot(intakePivotRelativeMaxPosition);
    }
    else if (robotData.controlData.saCubeIntake)
    {
        IntakeRollers(intakeRollerCubeInwardSpeed);
        IntakePivot(intakeRollerCubeInwardSpeed);
    }
    else
    {
        IntakeRollers(0);
        IntakePivot(intakePivotRelativeMinPosition);
    }
}

void EndEffector::Manual(const RobotData &robotData, EndEffectorData &endEffectorData)
{
    if (softLimitsToggled) 
    {
        ToggleSoftLimits();
    }

    if (robotData.controlData.mIntakeDown)
    {
        intakePivot.Set(-0.3);
    }
    else if (robotData.controlData.mIntakeUp)
    {
        intakePivot.Set(0.3);
    }

    if (robotData.controlData.mIntakeRollersIn) 
    {
        IntakeRollers(intakeRollerInwardSpeed);
    }
    else if (robotData.controlData.mIntakeRollersOut) 
    {
        IntakeRollers(intakeRollerOutwardSpeed);
    }

    if (robotData.controlData.mForceZeroIntake) 
    {
        ZeroIntake();
    }
}

/*
* @param rollerSpeed Desired intake roller speed (0 - 1)
*/
void EndEffector::IntakeRollers(double rollerSpeed)
{
    intakeRollers.Set(rollerSpeed);
    intakeRollers2.Set(-rollerSpeed);
}

/*
* @param pivotPosition Desired relative encoder pivot position
*/
void EndEffector::IntakePivot(double pivotPosition)
{
    intakePivotPIDController.SetReference(pivotPosition, rev::CANSparkMax::ControlType::kPosition);
}

/*
* @param currentAbsolutePosition Converts absolute encoder position to relative encoder position
*/
double EndEffector::AbsoluteToRelative(double currentAbsolutePosition) 
{
    double slope = (intakePivotRelativeMaxPosition - intakePivotRelativeMinPosition) / (intakePivotAbsoluteMaxPosition - intakePivotAbosluteMinPosition);
    double b = intakePivotRelativeMinPosition - (slope * intakePivotAbosluteMinPosition);
    return ((slope * currentAbsolutePosition) + b);
}

/*
* @note Toggles the soft limits on and off
* @note for when code switches between manual
* @note and semi automatic
*/
void EndEffector::ToggleSoftLimits() 
{
    if (softLimitsToggled)
    {
        intakePivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
        intakePivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
    }
    else if (!softLimitsToggled) 
    {
        intakePivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        intakePivot.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

        intakePivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, intakePivotRelativeMinPosition - 0.1);
        intakePivot.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, intakePivotRelativeMaxPosition + 0.1);
    }
}

/*
* @note Reseeds relative position based
* @note on absolute encoder position
*/
void EndEffector::ZeroIntake() 
{
    intakePivotRelativeEncoder.SetPosition(AbsoluteToRelative(intakePivotAbsoluteEncoder.GetOutput()));
}