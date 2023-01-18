#include "common/Arduino.h"

void Arduino::RobotInit()
{
    try
    {
        {
            arduinoPort = new frc::SerialPort(9600, frc::SerialPort::Port::kUSB);
        }
    }
    catch(...)
    {
        arduinoWorking = false;
    }
    
}

void Arduino::RobotPeriodic(const RobotData &robotData, ArduinoData arduinoData)
{
    if (frc::DriverStation::IsEnabled() && arduinoWorking)
    {

        //code
        colorCode = 0;
        
        //if statements
        

        char value[1] = {(char)colorCode};

        try
        {
            arduinoPort->Write(value, 1);
        }
        catch(...)
        {
            failedTransfers++;
        }
        

        //read data
        try
        {
            // if (arduinoPort->GetBytesReceived() >= 1)
            // {
            //     arduinoPort->Read(colors, 1);
            //     arduinoPort->Reset();
            // }
        }
        catch(...)
        {
            failedTransfers++;
        }
        
    }
}

void Arduino::DisabledPeriodic()
{
    if (arduinoWorking)
    {
        colorCode = 0;
        char value[1] = {(char)colorCode};

        try {   
            arduinoPort->Write(value, 1);
        } catch (...){
            failedTransfers += 1;
        }
    }
}