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
    intakePivotRelativeEncoder.SetPosition(AbsoluteToRelative(intakePivotAbsoluteEncoder.GetOutput()));

}

void Intake::RobotPeriodic(const RobotData &robotData, IntakeData &intakeData)
{

}

void Intake::IntakeRollers(double rollerSpeed)
{
    intakeRollers.Set(rollerSpeed);
}

void Intake::IntakePivot(double pivotPosition)
{
    if (pivotPosition >= intakePivotAbosluteMinPosition && pivotPosition <= intakePivotAbsoluteMaxPosition)
    {
        intakePivotRelativeEncoder.SetPosition(pivotPosition);
    }
}

void Intake::SemiAuto()
{
    if (intakePivotRelativeEncoder.GetVelocity() < 1)
    {
        intakePivotRelativeEncoder.SetPosition(AbsoluteToRelative(intakePivotAbsoluteEncoder.GetOutput()));
    }
}

void Intake::Manual()
{
    // Filler
}

double Intake::AbsoluteToRelative(double currentAbsolutePosition) 
{
    double slope = (intakePivotRelativeMaxPosition - intakePivotRelativeMinPosition) / (intakePivotAbsoluteMaxPosition - intakePivotAbosluteMinPosition);
    double b = intakePivotRelativeMinPosition - (slope * intakePivotAbosluteMinPosition);
    return ((slope * currentAbsolutePosition) + b);
}

