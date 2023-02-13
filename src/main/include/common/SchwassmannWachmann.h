#pragma once

#include <frc/DriverStation.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

struct RobotData;

struct SchwassmannWachmannData
{
    double finalLeftSpeed;
    double finalRightSpeed;
};

class SchwassmannWachmann
{

public:
    void RobotInit(const RobotData &robotData, SchwassmannWachmannData &schwassmannWachmannData); 
private:


};

