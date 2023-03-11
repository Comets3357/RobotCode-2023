#include "common/Arduino.h"
#include "RobotData.h"

void Arduino::RobotInit()
{
    while (!arduinoWorking && (attemptsToConnectToArduino < 10))
    {
        try
        {
            arduino = new frc::SerialPort(9600, frc::SerialPort::Port::kUSB);
            arduinoWorking = true;
        }
        catch(...)
        {
            arduinoWorking = false;
            attemptsToConnectToArduino++;
        }
    }
    
}

void Arduino::RobotPeriodic(const RobotData &robotData, ArduinoData arduinoData)
{

    char colorCode;

    char value[1];

    frc::SmartDashboard::PutNumber("Arduino failed transfers", failedTransfers);
    frc::SmartDashboard::PutBoolean("ARDUINO WORKING", arduinoWorking);

    if (frc::DriverStation::IsEnabled() && arduinoWorking)
    {

        //code
        
        
        //if statements
        if (!robotData.armData.pivotAbsoluteInitialized 
        || !robotData.armData.wristAbsoluteInitialized 
        || !robotData.bullBarData.bullBarAbsoluteEncoderInitialized 
        || !robotData.elevatorData.elevatorAbsoluteEncoderInitialized) //system fault
        {
            //colorCode = 'm';
        }
        else if (false)//autobalance not yet implemented
        {
            colorCode = 'l';
        }
        
 if (robotData.endEffectorData.gamePieceType != NONE)//have game piece
        {
            colorCode = 'k';
        }
        else if (robotData.controlData.saConeCall)//cone call
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
        else if (robotData.armData.coneIntakeRunning || robotData.controlData.saConeFlipPosition || robotData.controlData.saUprightConeIntake || robotData.controlData.saPositionHumanPlayer)//intaking cone
        {
            colorCode = 'j';
        }
        else if (robotData.armData.cubeIntakeRunning)//intaking cube
        {
            colorCode = 'i';
        }
        
        else
        {
            colorCode = 'n';
        }
        

    }
    else if (arduinoWorking)
    {
        if (false)//potato
        {
            colorCode = 'd';
        }
        else if ((!robotData.armData.pivotAbsoluteInitialized 
               || !robotData.armData.wristAbsoluteInitialized 
               || !robotData.bullBarData.bullBarAbsoluteEncoderInitialized 
               || !robotData.elevatorData.elevatorAbsoluteEncoderInitialized) )//bad
        {
            colorCode = 'b';//systems bad
        }
        else
        {
            colorCode = 'c';//systems good
        }
    }

    value[0] = colorCode;

    frc::SmartDashboard::PutNumber("LED MODE", value[1]);

    try
    {
        arduino->Write(value, 1);
    }
    catch(...)
    {
        failedTransfers++;
    }
    UpdateData();
}

void Arduino::UpdateData()
{
    // frc::SmartDashboard::PutBoolean("System Fault", colorCode == 'b');
    // frc::SmartDashboard::PutBoolean("Call for Piece", colorCode == 'g' || colorCode == 'e' || colorCode == 'h' || colorCode == 'f');
    frc::SmartDashboard::PutBoolean("Have Gamepiece", colorCode == 'k');
}
