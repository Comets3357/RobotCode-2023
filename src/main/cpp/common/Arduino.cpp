#include "common/Arduino.h"

void Arduino::RobotInit()
{
    try
    {
        {
            arduino = new frc::SerialPort(9600, frc::SerialPort::Port::kUSB);
        }
    }
    catch(...)
    {
        arduinoWorking = false;
    }
    
}

void Arduino::RobotPeriodic(const RobotData &robotData, ArduinoData arduinoData)
{

    char colorCode;

    char value[1];


    if (frc::DriverStation::IsEnabled() && arduinoWorking)
    {

        //code
        
        
        //if statements
        if (!robotData.armData.pivotAbsoluteInitialized || !robotData.armData.wristAbsoluteInitialized || !robotData.bullBarData.bullBarAbsoluteEncoderInitialized || !robotData.elevatorData.elevatorAbsoluteEncoderInitialized) //system fault
        {
            colorCode = 'm';
        }
        else if (false)//autobalance not yet implemented
        {
            colorCode = 'l';
        }
        else if (robotData.endEffectorData.gamePieceType != NONE)//have game piece
        {
            colorCode = 'k';
        }
        else if (robotData.armData.coneIntakeRunning)//intaking cone
        {
            colorCode = 'j';
        }
        else if (robotData.armData.cubeIntakeRunning)//intaking cube
        {
            colorCode = 'i';
        }

        if (robotData.controlData.saConeCall)//cone call
        {
            colorCode = 'g';
        }
        else if (robotData.controlData.saCubeCall)//cube call
        {
            colorCode = 'e';
        }

        else if (robotData.controlData.saFastConeCall)//fast cone call
        {
            colorCode = 'h';
        }
        else if (robotData.controlData.saFastCubeCall)//fast cube call
        {
            colorCode = 'f';
        }

        
        

        

        
        
    }
    else
    {
        if (false)//potato
        {
            colorCode = 'd';
        }
        else if ((!robotData.armData.pivotAbsoluteInitialized || !robotData.armData.wristAbsoluteInitialized || !robotData.bullBarData.bullBarAbsoluteEncoderInitialized || !robotData.elevatorData.elevatorAbsoluteEncoderInitialized) )//bad
        {
            colorCode = 'b';//systems bad
        }
        else
        {
            colorCode = 'c';//systems good
        }
    }

    value[1] = colorCode;

    try
    {
        arduino->Write(value, 1);
    }
    catch(...)
    {
        failedTransfers++;
    }

}

