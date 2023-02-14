#pragma once

#include <frc/SerialPort.h>
#include <time.h>

struct RobotData;

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

    void UpdateData();
    bool arduinoWorking = false;
    int attemptsToConnectToArduino = 0;
    int colorCode;

    frc::SerialPort *arduino;

};