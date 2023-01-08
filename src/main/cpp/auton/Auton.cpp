#include "auton/Auton.h"
#include "RobotData.h"

void Auton::RobotInit(AutonData &autonData) {
    sendAutonSelectionChooser();
}

// creates pathGroup vector (list of strings that are interpretted by drivebase)
void Auton::AutonomousInit(AutonData &autonData)
{
    autonData.autonStep = -1;   // starts at -1 so that the stepper function can advance it to index 0 the first time

    // directory to deploy folder on roborio
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

    autonData.autonRoutineName = autonChooser.GetSelected();
    fs::path autonDirectory = deployDirectory / "Autos" / autonData.autonRoutineName;
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
    // autonChooser.AddOption("potato", "potato");

    // autonChooser.AddOption("taxiShootA", "taxiShootA");
    // autonChooser.AddOption("taxiShootAHide", "taxiShootAHide");
    // autonChooser.AddOption("taxiShootB", "taxiShootB");
    // autonChooser.AddOption("taxiShootC", "taxiShootC");

    // autonChooser.AddOption("fourBallB", "fourBallB");
    // autonChooser.AddOption("fourBallC", "fourBallC");
    
    autonChooser.AddOption("5Ball", "5Ball");
    // autonChooser.AddOption("fiveBallCAlt", "fiveBallCAlt");

    // autonChooser.AddOption("citrus", "citrus");
    // autonChooser.AddOption("hideBallsA", "hideBallsA");

    // autonChooser.AddOption("nearFieldOne", "nearFieldOne");

    // autonChooser.AddOption("sixBallC", "sixBallC");

    frc::SmartDashboard::PutData("Select Auton:", &autonChooser);
}


void Auton::AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControlData &controlData, ControllerData &controllerData)
{
    // frc::smartDashboard::PutString("autonRoutineName", autonData.autonRoutineName);

    controlData.mode = mode_teleop_sa;

    // if (autonData.autonRoutineName == "potato")
    // {
    //     potato(robotData, controlData, controllerData);
    // }
    // else if (autonData.autonRoutineName == "citrus") {
    //     citrus(robotData, controlData, controllerData);
    // }
    // else if (autonData.autonRoutineName == "hideBallsA") {
    //     citrus(robotData, controlData, controllerData);
    // }
    // else if (autonData.autonRoutineName == "nearFieldOne") {
    //     nearFieldOne(robotData, controlData, controllerData);
    // }
    // else if (autonData.autonRoutineName == "taxiShootA") {
    //     taxiShootA(robotData, controlData, controllerData);
    // }
    // else if (autonData.autonRoutineName == "taxiShootAHide") {
    //     taxiShootA(robotData, controlData, controllerData);
    // }
    // else if (autonData.autonRoutineName == "taxiShootB") {
    //     taxiShoot(robotData, controlData, controllerData);
    // }
    // else if (autonData.autonRoutineName == "taxiShootC") {
    //     taxiShoot(robotData, controlData, controllerData);
    // }
    // else if (autonData.autonRoutineName == "fourBallB") {
    //     fourBallB(robotData, controlData, controllerData);
    // }
    // else if (autonData.autonRoutineName == "fourBallC") {
    //     fourBallC(robotData, controlData, controllerData);
    // }
    if (autonData.autonRoutineName == "5Ball") 
    {
        fiveBallC(robotData, controlData, controllerData);
    }
    // else if (autonData.autonRoutineName == "fiveBallCAlt") {
    //     fiveBallC(robotData, controlData, controllerData);
    // }
    // else if (autonData.autonRoutineName == "sixBallC") {
    //     sixBallC(robotData, controlData, controllerData);
    // }
}


// void Auton::potato(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
// {
//     controlData.saIntake = false;
// }


// void Auton::citrus(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData) {
//     double sec = robotData.timerData.secSinceEnabled;

//     // intake
//     if (sec > 0 && sec < 11) {
//         controlData.saIntake = true;
//         controlData.saEjectBalls = false;
//     } else if (sec > 12) {
//         controlData.saIntake = false;
//         controlData.saEjectBalls = true;
//     }

//     // shooting
//     controlData.shootMode = shootMode_vision;

//     if (sec > 1.3 && sec < 4) {
//         controlData.saFinalShoot = robotData.drivebaseData.dbStationaryForShot;
//     } else {
//         controlData.saFinalShoot = false;
//     }
// }


// void Auton::nearFieldOne(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData) {
//     //double sec = robotData.timerData.secSinceEnabled;

//     // intake
//     controlData.saIntake = false;

//     // shooting
//     controlData.shootMode = shootMode_vision;

//     controlData.saFinalShoot = robotData.drivebaseData.dbStationaryForShot;
// }


// void Auton::taxiShoot(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData) {
//     double sec = robotData.timerData.secSinceEnabled;

//     // intake
//     if (sec > 0 && sec < 4) {
//         controlData.saIntake= true;
//     } else {
//         controlData.saIntake = false;
//     }

//     // shooting
//     if (sec > 3 && sec < 7) {
//         controlData.shootMode = shootMode_vision;
//     } else {
//         controlData.shootMode = shootMode_none;
//     }

//     if (sec > 5 && sec < 7) {
//         controlData.saFinalShoot = true;
//     } else {
//         controlData.saFinalShoot = false;
//     }
// }

// void Auton::taxiShootA(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData) {
//     double sec = robotData.timerData.secSinceEnabled;

//     // intake
//     if (sec > 0 && sec < 11) {
//         controlData.saIntake = true;
//         controlData.saEjectBalls = false;
//     } else if (sec > 11) {
//         controlData.saIntake = false;
//         controlData.saEjectBalls = true;
//     }

//     // shooting
//     controlData.shootMode = shootMode_vision;

//     if (sec > 1.5 && sec < 6) {
//         controlData.saFinalShoot = true;
//     } else {
//         controlData.saFinalShoot = false;
//     }
// }

// void Auton::fourBallB(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData) {
//     double sec = robotData.timerData.secSinceEnabled;

//     // intake
//     if (sec < 11) {
//         controlData.saIntake = true;
//     } else {
//         controlData.saIntake = false;
//     }

//     // run flywheel and aim
//     controlData.shootMode = shootMode_vision;


//     // final shoot
//     if (sec > 3.2 && sec < 4.5) {
//         controlData.saFinalShoot = true;
//     } else if (sec > 13.5 && sec < 15) {
//         controlData.saFinalShoot = true;
//     } else {
//         controlData.saFinalShoot = false;
//     }
// }

// void Auton::fourBallC(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData) {
//     double sec = robotData.timerData.secSinceEnabled;

//     // intake
//     if (sec < 11) {
//         controlData.saIntake = true;
//     } else {
//         controlData.saIntake = false;
//     }

//     // run flywheel and aim
//     controlData.shootMode = shootMode_vision;


//     // final shoot
//     if (sec > 3.2 && sec < 4.5) {
//         controlData.saFinalShoot = true;
//     } else if (sec > 13.5 && sec < 15) {
//         controlData.saFinalShoot = true;
//     } else {
//         controlData.saFinalShoot = false;
//     }
// }

void Auton::fiveBallC(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData)
{
    double sec = robotData.timerData.secSinceEnabled;

    // intake
    if (sec > 1 && sec < 5.5)
    {
        controlData.saIntake = true;
    } else if (sec > 8 && sec < 12.5) {

        controlData.saIntake = true;
    } else 
    {
        controlData.saIntake = false;
    }
    controlData.saIntake = false;

    //aim-+/
    if (sec > 0 && sec < 14) 
    {
        controlData.shootMode = shootMode_vision;
    } else 
    {
        controlData.shootMode = shootMode_none;
    }
}

//     // final shoot
//     if (sec > 0 && sec < 1.5) {
//         controlData.saFinalShoot = true;
//     } else if (sec > 4 && sec < 8) {
//         controlData.saFinalShoot = robotData.drivebaseData.dbStationaryForShot;
//     } else if (sec > 12.5 && sec < 15) {
//         controlData.saFinalShoot = robotData.drivebaseData.dbStationaryForShot;
//     } else {
//         controlData.saFinalShoot = false;
//     }

// }

// void Auton::sixBallC(const RobotData &robotData, ControlData &controlData, ControllerData &controllerData) {}