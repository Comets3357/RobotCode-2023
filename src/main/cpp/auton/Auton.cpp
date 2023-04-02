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

    // autonChooser.AddOption("TwoBlueLoadingClimb", "TwoBlueLoadingClimb"); // 3
    // autonChooser.AddOption("TwoBlueLoadingNoClimb", "TwoBlueLoadingNoClimb"); // 4

    //autonChooser.AddOption("TwoBlueBumpClimb", "TwoBlueBumpClimb"); // 5
    autonChooser.AddOption("TwoBlueBumpNoClimb", "TwoBlueBumpNoClimb"); // 6

    // autonChooser.AddOption("BlueChargeStation", "BlueChargeStation"); // 7

    // autonChooser.AddOption("TwoRedLoadingClimb", "TwoRedLoadingClimb"); // 8
    // autonChooser.AddOption("TwoRedLoadingNoClimb", "TwoRedLoadingNoClimb"); // 9

    // autonChooser.AddOption("testplace", "testplace");

    //autonChooser.AddOption("TwoRedBumpClimb", "TwoRedBumpClimb"); // 10
    autonChooser.AddOption("TwoRedBumpNoClimb", "TwoRedBumpNoClimb"); // 11

    // autonChooser.AddOption("RedChargeStation", "RedChargeStation"); // 12
    autonChooser.AddOption("PlaceAndBalance", "PlaceAndBalance"); //13

    autonChooser.AddOption("TraverseChargeStation", "TraverseChargeStation");

    autonChooser.AddOption("ThreeRedLoading", "ThreeRedLoading");
    autonChooser.AddOption("ThreeBlueLoading", "ThreeBlueLoading");

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
    if (autonData.autonRoutineName == "PlaceAndBalance" || autonData.autonRoutineName == "BlueChargeStation1.5") // 2 - TEST BLUE
    {
        autonData.autonToggle = true;
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
    else if (autonData.autonRoutineName == "TraverseChargeStation")
    {
        autonData.autonToggle = false;
        Traverse(robotData, controlData, controllerData);
    }
    else if (autonData.autonRoutineName == "ThreeRedLoading" || autonData.autonRoutineName == "ThreeBlueLoading")
    {
        ThreeLoading(robotData, controlData, controllerData);
    }


    if (robotData.endEffectorData.gamePieceType == CONE || robotData.endEffectorData.gamePieceType == CUBE)
    {
        controlData.saConeIntake = false;
        controlData.saCubeIntake = false;
        controlData.saUprightConeIntake = false;
    }

}

void Auton::ThreeLoading(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled-0.8;

    switch (step)
    {
    case (0):   
        controlData.saPositionHigh = true;
        step++;
        break;
    case(1):
        controlData.saPositionHigh = false;
        if (sec > 0.6) step++;
        break;
    case(2):
        controlData.saIntakeBackwards = true;
        if (sec > 1.15) step++;
        break;
    case(3): 
        controlData.saHomePosition = true;
        step++;
        break;
    case(4):
        controlData.saHomePosition = false;
        if (sec > 1.35) controlData.saIntakeBackwards = false;
        if (sec > 1.85)step++;
        break;
    case(5):
        controlData.saCubeIntake = true;
        if (sec > 4.35) step++;
        break;
        case(6):
        if (sec > 4.6 && sec < 4.74)
        {
            controllerData.sLYStick = -0.6;
        }
        else
        {
            controllerData.sLYStick = 0.0;
        }
        controlData.saCubeIntake = false;
        if (sec > 5.85) step++;
        break;
    case (7):
        controlData.saPositionHigh = true;
        step++;
        break;
    case(8):
        controlData.saPositionHigh = false;
        if (sec > 7.05) step++;
        break;
    case(9):
        controllerData.sLYStick = 0.6;
        if (sec > 7.4) step++;
        break;
    case(10):

        
        controlData.saHomePosition = true;
        step++;
        break;
    case(11):
        if (sec > 7.65)
        {
            controllerData.sLYStick = 0.0;
        }
        controlData.saHomePosition = false;
        if (sec > 8.15)step++;
        break;
    case(12):
        controlData.saCubeIntake = true;
        if (sec > 11.15) step++;
        break;
    case(13):
        controlData.saCubeIntake = false;
        if (sec > 11.5 && sec < 11.6)
        {
            controllerData.sLYStick = -0.6;
        }
        else
        {
            controllerData.sLYStick = 0.0;
        }

        if (sec > 13.0) step++;
        break;
    case 14:
        controlData.saPositionMid = true;
        step++;
        break;
    case 15:
        controlData.saPositionMid = false;
        if (sec > 14.0) step++;
        // step++;
        break;
    case 16:
        controllerData.sLYStick = 0.6;
        if (sec > 14.18) step++;
        break;
    case 17:
        controlData.saHomePosition = true;
        controllerData.sLYStick = 0.0;
        step++;
        break;
    case 18:
        controlData.saHomePosition = false;
        break;
    }
}

void Auton::Traverse(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled;

    switch (step)
    {
        case 0:
            controlData.saPositionHigh = true;
            step++;
            break;
        case(1):
            controlData.saPositionHigh = false;
            if (sec > 1.75) step++;
            break;
        case(2):
            controlData.saIntakeBackwards = true;
            if (sec > 2.3) step++;
            break;
        case(3): 
            controlData.saHomePosition = true;
            controlData.saIntakeBackwards = false;
            step++;
            break;
        case(4):
            controlData.saHomePosition = false;
            if (sec > 5.6) step++;
            break;
        if (robotData.drivebaseData.allowBullBarExtend)
        {
            case 5:
                controlData.saCubeIntake = true;
                if (sec > 7.4 && sec < 7.55)
                {
                    controllerData.sLYStick = -0.6;
                }
                else
                {
                    controllerData.sLYStick = 0.0;
                    controlData.saCubeIntake = true;
                }
                if (sec > 8.3) step++;
                break;
            case 6:
                
                controlData.saCubeIntake = false;
                // controlData.saHomePosition = true;
                step++;
                break;
            case 7:
                // controlData.saHomePosition = false;
                break;
        }
        
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
    double sec = robotData.timerData.secSinceEnabled-1.0;

    switch (step)
    {
    case (0):   
        controlData.saPositionHigh = true;
        step++;
        break;
    case(1):
        controlData.saPositionHigh = false;
        if (sec > 0.75) step++;
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
        if (sec > 4.5) step++;
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
        if (sec > 7.05) step++;
        break;
    case(9):
        // controlData.saIntakeBackwards = true;
        controllerData.sLYStick = -0.7;
        if (sec > 7.4) step++;
        break;
    case(10):

        
        controlData.saHomePosition = true;
        step++;
        break;
    case(11):
        if (sec > 7.65)
        {
            // controlData.saIntakeBackwards = false;
            controllerData.sLYStick = 0.0;
        }
        controlData.saHomePosition = false;
        if (sec > 8.15)step++;
        break;
    case(12):
        controlData.saConeIntake = true;
        if (sec > 11.15) step++;
        break;
    case(13):
        controlData.saConeIntake = false;
        if (sec > 13.75) step++;
        break;
    case 14:
        //controlData.saPositionHigh = true;
        step++;
        break;
    case 15:
        //controlData.saPositionHigh = false;
        step++;
        break;
    case 16:
        break;
    }
}

void Auton::BumpNoClimb(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled;

    controlData.saResetOdometry = true;

    switch (step)
    {
    case (0):   
        controlData.saPositionMid = true;
        step++;
        break;
    case(1):
        controlData.saPositionMid = false;
        if (sec > 0.5) step++;
        break;
    case(2):
        controlData.saIntakeBackwards = true;
        if (sec > 0.7) step++;
        break;
    case(3): 
        controlData.saHomePosition = true;
        step++;
        break;
    case(4):
        controlData.saHomePosition = false;
        if (sec > 1.0) controlData.saIntakeBackwards = false;
        if (sec > 1.6)step++;
        break;
    case(5):
        controlData.saCubeIntake = true;
        if (sec > 4.0 && sec < 4.15)
        {
            controllerData.sLYStick = -0.6;
        }
        else
        {
            controllerData.sLYStick = 0.0;
        }
        if (sec > 4.6) step++;
        break;
        case(6):
        
        controlData.saCubeIntake = false;
        if (sec > 5.85) step++;
        break;
    case (7):
        controlData.saPositionHigh = true;
        step++;
        break;
    case(8):
        controlData.saPositionHigh = false;
        if (sec > 7.0) step++;
        break;
    case(9):
        controllerData.sLYStick = 0.6;
        if (sec > 7.4) step++;
        break;
    case(10):
        controllerData.sLYStick = 0.0;
        controlData.saHomePosition = true;
        step++;
        break;
    case(11):
        controlData.saHomePosition = false;
        if (sec > 8.7)step++;
        break;
    case(12):
        controlData.saCubeIntake = true;

        if (sec > 11.1 && sec < 11.2)
        {
            controllerData.sLYStick = -0.6;
        }
        else
        {
            controllerData.sLYStick = 0.0;
        }

        if (sec > 11.8) step++;
        break;
    case(13):
        controlData.saCubeIntake = false;
        if (sec > 13.05) 
        {
            controlData.saPositionMid = true;
            step++;
        }
        // if (sec > 14.1) step++;
        break;
    case 14:
        controlData.saPositionMid = false;
        if (sec > 14.5) step++;
        // if (sec > 14.1) step++;
        break;
    case 15:
        controllerData.sLYStick = 0.6;
        if (sec > 14.8) step++;
        break;
    case(16):
        controllerData.sLYStick = 0.0;
        controlData.saHomePosition = true;
        break;
    // case 14:
    //     controlData.saPositionMid = true;
    //     step++;
    //     break;
    // case 15:
    //     controlData.saPositionMid = false;
    //     if (sec > 14.4) step++;
    //     break;
    // case 16:
    //     controlData.saIntakeBackwards = true;
    //     if (sec > 14.8) step++;
    //     break;
    // case 17:
    //     controlData.saIntakeBackwards = false;
    //     break;
    }
}