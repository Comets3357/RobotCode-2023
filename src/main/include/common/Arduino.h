#pragma once

#include <frc/SerialPort.h>
#include <time.h>

#include "RobotData.h"

struct ArduinoData
{
       
};

class Arduino
{
public:
    void RobotPeriodic(const RobotData &robotData, ArduinoData arduinoData);
    void DisabledPeriodic();
    void RobotInit();

    int failedTransfers = 0;
private:

    bool arduinoWorking = true;
    int colorCode;

    frc::SerialPort *arduino;

};