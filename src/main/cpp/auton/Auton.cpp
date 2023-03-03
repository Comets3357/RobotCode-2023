#include "auton/Auton.h"
#include "RobotData.h"

void Auton::RobotInit(ControlData &controlData, AutonData &autonData) {
    sendAutonSelectionChooser();

    controlData.saResetOdometry = false;
    controlData.saForceRunBullBar = false;
    
}

// creates pathGroup vector (list of strings that are interpretted by drivebase)
void Auton::AutonomousInit(AutonData &autonData)
{
    autonData.autonStep = -1;   // starts at -1 so that the stepper function can advance it to index 0 the first time

    // directory to deploy folder on roborio
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

    autonData.autonRoutineName = autonChooser.GetSelected();
    fs::path autonDirectory = deployDirectory / "Autos" /  autonData.autonRoutineName;
    frc::SmartDashboard::PutString("autonDirectory", autonDirectory.string());

    std::ifstream inFile;
    inFile.open(autonDirectory.string());

    autonData.pathGroup.clear();

    if (inFile.fail()) {
    frc::SmartDashboard::PutString("fail", "failed");
    } else {
        std::string str;
        while (getline(inFile, str)) {
            frc::SmartDashboard::PutString("str", str);
            autonData.pathGroup.push_back(str);
        }
    }

    // remove newline char from all but the final line
    for (size_t i = 0; i < autonData.pathGroup.size(); i++) {
        std::string correctPathName = autonData.pathGroup[i];

        frc::SmartDashboard::PutBoolean("int bool" + std::to_string(i), correctPathName[correctPathName.length() - 1] == 13);
        frc::SmartDashboard::PutNumber(std::to_string(i) + "int", correctPathName[correctPathName.length() - 1]);

        // if the last char in the string is a newline, delete it for proper auton selection processing
        if (int(correctPathName[correctPathName.length() - 1]) == 13) {
            correctPathName = correctPathName.substr(0, correctPathName.length() - 1);  // get rid of hidden newline from file line read
        }
        
        autonData.pathGroup[i] = correctPathName;

        frc::SmartDashboard::PutString(std::to_string(i), autonData.pathGroup[i]);
        
        
        frc::SmartDashboard::PutString(std::to_string(i), autonData.pathGroup[i]);        
    }
}

void Auton::sendAutonSelectionChooser() {

    autonChooser.AddOption("potato", "potato"); // 1

    autonChooser.AddOption("SinglePlace", "SinglePlace"); // 2

    autonChooser.AddOption("TwoBlueLoadingClimb", "TwoBlueLoadingClimb"); // 3
    autonChooser.AddOption("TwoBlueLoadingNoClimb", "TwoBlueLoadingNoClimb"); // 4

    //autonChooser.AddOption("TwoBlueBumpClimb", "TwoBlueBumpClimb"); // 5
    //autonChooser.AddOption("TwoBlueBumpNoClimb", "TwoBlueBumpNoClimb"); // 6

    autonChooser.AddOption("BlueChargeStation", "BlueChargeStation"); // 7

    autonChooser.AddOption("TwoRedLoadingClimb", "TwoRedLoadingClimb"); // 8
    autonChooser.AddOption("TwoRedLoadingNoClimb", "TwoRedLoadingNoClimb"); // 9

    autonChooser.AddOption("testplace", "testplace");

    //autonChooser.AddOption("TwoRedBumpClimb", "TwoRedBumpClimb"); // 10
    //autonChooser.AddOption("TwoRedBumpNoClimb", "TwoRedBumpNoClimb"); // 11

    autonChooser.AddOption("RedChargeStation", "RedChargeStation"); // 12
    autonChooser.AddOption("RedChargeStation1.5", "RedChargeStation1.5"); //13

    frc::SmartDashboard::PutData("Select Auton:", &autonChooser);
}


void Auton::AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControlData &controlData, ControllerData &controllerData)
{
    // frc::smartDashboard::PutString("autonRoutineName", autonData.autonRoutineName);

    frc::SmartDashboard::PutNumber("Auton Step", step);

    controlData.mode = MODE_TELEOP_SA;

    if (autonData.autonRoutineName == "TwoRedLoadingClimb" || autonData.autonRoutineName == "TwoBlueLoadingClimb") // 2 - TEST BLUE
    {
        Loading(robotData, controlData, controllerData);
    }
    if (autonData.autonRoutineName == "RedChargeStation1.5" || autonData.autonRoutineName == "BlueChargeStation1.5") // 2 - TEST BLUE
    {
        OneMiddleClimb(robotData, controlData, controllerData);
    }
    else if (autonData.autonRoutineName == "SinglePlace") // 3 - TEST
    {
        placeCone(robotData, controlData, controllerData);
    }
    else if (autonData.autonRoutineName == "RedChargeStation" || autonData.autonRoutineName == "BlueChargeStation") // 5 - TEST
    {
        TwoMiddleClimb(robotData, controlData, controllerData);
    }
    else if (autonData.autonRoutineName == "TwoRedBumpClimb" || autonData.autonRoutineName == "TwoBlueBumpClimb") // 7 - TEST
    {
        Bump(robotData, controlData, controllerData);
    }
    else if (autonData.autonRoutineName == "potato") // 8 - TEST
    {
        potato(robotData, controlData, controllerData);
    }
    else if (autonData.autonRoutineName == "TwoRedLoadingNoClimb" || autonData.autonRoutineName == "TwoBlueLoadingNoClimb") // 10 - TEST
    {
        LoadingNoClimb(robotData, controlData, controllerData);
    }
    else if (autonData.autonRoutineName == "TwoRedBumpNoClimb" || autonData.autonRoutineName == "TwoBlueBumpNoClimb") // 12 - TEST 
    {
        BumpNoClimb(robotData, controlData, controllerData);
    }
    else if (autonData.autonRoutineName == "testplace")
    {
        testplace(robotData, controlData, controllerData);
    }


    if (robotData.endEffectorData.gamePieceType == CONE || robotData.endEffectorData.gamePieceType == CUBE)
    {
        controlData.saConeIntake = false;
        controlData.saCubeIntake = false;
        controlData.saUprightConeIntake = false;
    }

}

void Auton::testplace(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled-0.1;

    switch (step)
    {
    case (0):   
        controlData.saPositionHigh = true;
        step++;
        break;
    case(1):
        controlData.saPositionHigh = false;
        if (sec > 1.0) step++;
        break;
    case(2):
        controlData.saIntakeBackwards = true;
        if (sec > 1.3) step++;
        break;
    case(3): 
        controlData.saHomePosition = true;
        step++;
        break;
    case(4):
        controlData.saHomePosition = false;
        if (sec > 1.5) controlData.saIntakeBackwards = false;
        break;
    }    
}


void Auton::potato(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled;

    controlData.saResetOdometry = false;
}

void Auton::placeCone(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled;

    switch (step)
    {
    case (0):
        
        controlData.saPositionHigh = true;
        step++;
        break;
    
    case(1):
        controlData.saPositionHigh = false;
        if (sec > 3) step++;
        break;
    case(2):
        controlData.saIntakeBackwards = true;
        if (sec > 5) step++;
        break;
    case(3):
        controlData.saIntakeBackwards = false;
        controlData.saHomePosition = true;
        step++;
        break;
    case(4):
        controlData.saHomePosition = false;
        break;
    }
}

void Auton::TwoMiddleClimb(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled -0.1;

    if ((sec > 9.5) && (sec < 10.7))
    {
        controlData.saResetOdometry = true;
    }
    else
    {
        controlData.saResetOdometry = false;
    }

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        switch (step)
        {
            case (0):
            
            controlData.saPositionHigh = true;
            step++;
            break;
        
            case(1):
                controlData.saPositionHigh = false;
                if (sec > 1.0) step++;
                break;
            case(2):
                controlData.saIntakeBackwards = true;
                if (sec > 1.2) step++;
                break;
            case(3):
                controlData.saIntakeBackwards = false;
                controlData.saHomePosition = true;
                step++;
                break;
            case(4):
                controlData.saHomePosition = false;
                if (sec > 4.0) step++;
                break;
            case 5:
                controlData.saCubeIntake = true;
                if (sec > 6.5) step++;
                break;
            case 6:
                controlData.saCubeIntake = false;
                if (sec > 10.0) step++;
                break;
            case 7:
                controlData.saPositionHigh = true;
                step++;
                break;
            case 8:
                controlData.saPositionHigh = false;
                if (sec > 11.2) step++;
                break;
            case 9: 
                controlData.saIntakeBackwards = true;
                if (sec > 11.5) step++;
                break;
            case 10:
                controlData.saIntakeBackwards = false;
                controlData.saHomePosition = true;
                step++;
                break;
            case 11:
                controlData.saHomePosition = false;
                break;
        }   
    }
    else
    {
            switch (step)
        {
            case (0):
            
            controlData.saPositionHigh = true;
            step++;
            break;
        
            case(1):
                controlData.saPositionHigh = false;
                if (sec > 1.0) step++;
                break;
            case(2):
                controlData.saIntakeBackwards = true;
                if (sec > 1.2) step++;
                break;
            case(3):
                controlData.saIntakeBackwards = false;
                controlData.saHomePosition = true;
                step++;
                break;
            case(4):
                controlData.saHomePosition = false;
                if (sec > 3.6) step++;
                break;
            case 5:
                controlData.saCubeIntake = true;
                if (sec > 6.1) step++;
                break;
            case 6:
                controlData.saCubeIntake = false;
                if (sec > 9.6) step++;
                break;
            case 7:
                controlData.saPositionHigh = true;
                step++;
                break;
            case 8:
                controlData.saPositionHigh = false;
                if (sec > 10.8) step++;
                break;
            case 9: 
                controlData.saIntakeBackwards = true;
                if (sec > 11.1) step++;
                break;
            case 10:
                controlData.saIntakeBackwards = false;
                controlData.saHomePosition = true;
                step++;
                break;
            case 11:
                controlData.saHomePosition = false;
                break;
        }
    }
}

void Auton::OneMiddleClimb(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled - 0.1;

 

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        switch (step)
        {
            case (0):
            
            controlData.saPositionHigh = true;
            step++;
            break;
        
            case(1):
                controlData.saPositionHigh = false;
                if (sec > 2.0) step++;
                break;
            case(2):
                controlData.saIntakeBackwards = true;
                if (sec > 3.0) step++;
                break;
            case(3):
                controlData.saIntakeBackwards = false;
                controlData.saHomePosition = true;
                step++;
                break;
            case(4):
                controlData.saHomePosition = false;
                //if (sec > 4.0) step++;
                break;
            /*case 5:
                controlData.saCubeIntake = true;
                if (sec > 6.5) step++;
                break;
            case 6:
                controlData.saCubeIntake = false;
                if (sec > 10.0) step++;
                break;*/
            
        }   
    }
    else
    {
            switch (step)
        {
            case (0):
            
            controlData.saPositionHigh = true;
            step++;
            break;
        
            case(1):
                controlData.saPositionHigh = false;
                if (sec > 2.0) step++;
                break;
            case(2):
                controlData.saIntakeBackwards = true;
                if (sec > 3.0) step++;
                break;
            case(3):
                controlData.saIntakeBackwards = false;
                controlData.saHomePosition = true;
                step++;
                break;
            case(4):
                controlData.saHomePosition = false;
                //if (sec > 4.0) step++;
                break;
            /*case 5:
                controlData.saCubeIntake = true;
                if (sec > 6.5) step++;
                break;
            case 6:
                controlData.saCubeIntake = false;
                if (sec > 10.0) step++;
                break;*/
            
        }   
    }
}

void Auton::Loading(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled-0.1;

    switch (step)
    {
    case (0):   
        controlData.saPositionHigh = true;
        step++;
        break;
    case(1):
        controlData.saPositionHigh = false;
        if (sec > 1.0) step++;
        break;
    case(2):
        controlData.saIntakeBackwards = true;
        if (sec > 1.3) step++;
        break;
    case(3): 
        controlData.saHomePosition = true;
        step++;
        break;
    case(4):
        controlData.saHomePosition = false;
        if (sec > 1.5) controlData.saIntakeBackwards = false;
        if (sec > 2)step++;
        break;
    case(5):
        controlData.saCubeIntake = true;
        if (sec > 5) step++;
        break;
        case(6):
        controlData.saCubeIntake = false;
        if (sec > 6) step++;
        break;
    case (7):
        controlData.saPositionHigh = true;
        step++;
        break;
    case(8):
        controlData.saPositionHigh = false;
        if (sec > 7.2) step++;
        break;
    case(9):
        controlData.saIntakeBackwards = true;
        if (sec > 7.25) step++;
        break;
    case(10):

        
        controlData.saHomePosition = true;
        step++;
        break;
    case(11):
        if (sec > 7.5)
        {
            controlData.saIntakeBackwards = false;
        }
        controlData.saHomePosition = false;
        if (sec > 8)step++;
        break;
        case(12):
        controlData.saConeIntake = true;
        if (sec > 11) step++;
        break;
        case(13):
        controlData.saConeIntake = false;
        break;
    }
    // intake
}

void Auton::Bump(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled-0.1;

    controlData.saResetOdometry = true;

    switch (step)
    {
    case (0):
        controlData.saPositionHigh = true;
        step++;
        break;
    case(1):
        controlData.saPositionHigh = false;
        if (sec > 1.1) step++;
        break;
    case(2):
        controlData.saIntakeBackwards = true;
        if (sec > 1.3) step++;
        break;
    case(3):
        controlData.saHomePosition = true;
        step++;
        break;
    case(4):
        controlData.saHomePosition = false;
        if (sec > 1.5) controlData.saIntakeBackwards = false;
        if (sec > 2)step++;
        break;
    case(5):
        controlData.saCubeIntake = true;
        if (sec > 5) step++;
        break;
        case(6):
        controlData.saCubeIntake = false;
        if (sec > 6) step++;
        break;
    case (7):
        controlData.saPositionHigh = true;
        step++;
        break;
    case(8):
        controlData.saPositionHigh = false;
        if (sec > 7) step++;
        break;
    case(9):
        controlData.saIntakeBackwards = true;
        if (sec > 7.25) step++;
        break;
    case(10):
        controlData.saIntakeBackwards = false;
        controlData.saHomePosition = true;
        step++;
        break;
    case(11):
        controlData.saHomePosition = false;
        if (sec > 8)step++;
        break;
    case(12):
        controlData.saConeIntake = true;
        if (sec > 11) step++;
        break;
    case(13):
        controlData.saConeIntake = false;
        break;
    }
}

void Auton::LoadingNoClimb(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled-0.1;

    switch (step)
    {
    case (0):   
        controlData.saPositionHigh = true;
        step++;
        break;
    case(1):
        controlData.saPositionHigh = false;
        if (sec > 1.0) step++;
        break;
    case(2):
        controlData.saIntakeBackwards = true;
        if (sec > 1.3) step++;
        break;
    case(3): 
        controlData.saHomePosition = true;
        step++;
        break;
    case(4):
        controlData.saHomePosition = false;
        if (sec > 1.5) controlData.saIntakeBackwards = false;
        if (sec > 2)step++;
        break;
    case(5):
        controlData.saCubeIntake = true;
        if (sec > 5) step++;
        break;
        case(6):
        controlData.saCubeIntake = false;
        if (sec > 6) step++;
        break;
    case (7):
        controlData.saPositionHigh = true;
        step++;
        break;
    case(8):
        controlData.saPositionHigh = false;
        if (sec > 7.2) step++;
        break;
    case(9):
        controlData.saIntakeBackwards = true;
        if (sec > 7.25) step++;
        break;
    case(10):

        
        controlData.saHomePosition = true;
        step++;
        break;
    case(11):
        if (sec > 7.5)
        {
            controlData.saIntakeBackwards = false;
        }
        controlData.saHomePosition = false;
        if (sec > 8)step++;
        break;
    case(12):
        controlData.saConeIntake = true;
        if (sec > 11) step++;
        break;
    case(13):
        controlData.saConeIntake = false;
        if (sec > 13.6) step++;
        break;
    case 14:
        controlData.saPositionHigh = true;
        step++;
        break;
    case 15:
        controlData.saPositionHigh = false;
        step++;
        break;
    case 16:
        break;
    }
}

void Auton::BumpNoClimb(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled-0.1;

    controlData.saResetOdometry = true;

    switch(step)
    {
    case (0):   
        controlData.saPositionHigh = true;
        step++;
        break;
    case(1):
        controlData.saPositionHigh = false;
        if (sec > 1.1) step++;
        break;
    case(2):
        controlData.saIntakeBackwards = true;
        if (sec > 1.3) step++;
        break;
    case(3): 
        controlData.saHomePosition = true;
        step++;
        break;
    case(4):
        controlData.saHomePosition = false;
        if (sec > 1.5) controlData.saIntakeBackwards = false;
        if (sec > 2)step++;
        break;
    case(5):
        controlData.saCubeIntake = true;
        if (sec > 5) step++;
        break;
        case(6):
        controlData.saCubeIntake = false;
        if (sec > 6) step++;
        break;
    case (7):
        controlData.saPositionHigh = true;
        step++;
        break;
    case(8):
        controlData.saPositionHigh = false;
        if (sec > 7) step++;
        break;
    case(9):
        controlData.saIntakeBackwards = true;
        if (sec > 7.25) step++;
        break;
    case(10):
        controlData.saIntakeBackwards = false;
        controlData.saHomePosition = true;
        step++;
        break;
    case(11):
        controlData.saHomePosition = false;
        if (sec > 8)step++;
        break;
    case(12):
        controlData.saConeIntake = true;
        if (sec > 11) step++;
        break;
    case(13):
        controlData.saConeIntake = false;
        if (sec > 13.6) step++;
        break;
    case 14:
        controlData.saPositionHigh = true;
        step++;
        break;
    case 15:
        controlData.saPositionHigh = false;
        step++;
        break;
    case 16:
        break;
    }
}