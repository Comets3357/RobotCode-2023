#pragma once

struct ArmData
{

};

class Arm
{
public:
    void RobotInit();
    void RobotPeriodic();

private:
    double absoluteToRelative(double currentPos);
    double angleToRelative(double angle);
    double angleToAbsolute(double angle);

    void RotateArmToAngle(double angle);
    void RotateArmAngle(double angle);

};