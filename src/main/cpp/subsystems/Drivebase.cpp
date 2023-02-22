#include "subsystems/Drivebase.h"
#include "RobotData.h"


void Drivebase::RobotInit(const RobotData &robotData)
{
    dbL.RestoreFactoryDefaults();
    dbR.RestoreFactoryDefaults();
    dbLF.RestoreFactoryDefaults();
    dbRF.RestoreFactoryDefaults();

    
    dbRF.Follow(dbR);
    dbLF.Follow(dbL);

    dbL.SetInverted(robotData.configData.drivebaseConfigData.leftInverted);
    dbLF.SetInverted(robotData.configData.drivebaseConfigData.leftInverted);
    dbR.SetInverted(robotData.configData.drivebaseConfigData.rightInverted);
    dbRF.SetInverted(robotData.configData.drivebaseConfigData.rightInverted);

    dbL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    dbLF.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    dbR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    dbRF.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);


    // NEED TO SET CURRENT LIMIT
    /**
  * Configure the current limits that will be used
  * Stator Current is the current that passes through the motor stators.
  *  Use stator current limits to limit rotor acceleration/heat production
  * Supply Current is the current that passes into the controller from the supply
  *  Use supply current limits to prevent breakers from tripping
  *
  * enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)  */
    dbL.SetSmartCurrentLimit(robotData.configData.drivebaseConfigData.currentLimit);
    dbLF.SetSmartCurrentLimit(robotData.configData.drivebaseConfigData.currentLimit);
    dbR.SetSmartCurrentLimit(robotData.configData.drivebaseConfigData.currentLimit);
    dbRF.SetSmartCurrentLimit(robotData.configData.drivebaseConfigData.currentLimit);

    dbLPIDController.SetP(robotData.configData.drivebaseConfigData.leftP);
    dbLPIDController.SetFF(robotData.configData.drivebaseConfigData.leftFF);
    dbLPIDController.SetD(0);

    dbRPIDController.SetP(robotData.configData.drivebaseConfigData.rightP);
    dbRPIDController.SetFF(robotData.configData.drivebaseConfigData.rightFF);
    dbRPIDController.SetD(0);

    dbL.BurnFlash();
    dbLF.BurnFlash();
    dbR.BurnFlash();
    dbRF.BurnFlash();

    setPercentOutput(0, 0);

    zeroEncoders();

    odometryInitialized = false;
}

void Drivebase::TeleopInit(const RobotData &robotData) 
{
    if (!odometryInitialized) 
    {
    //     frc::Pose2d startPoint = startPointChooser.GetSelected();
    //     resetOdometry(startPoint, robotData.gyroData.rawYaw);
        resetOdometry(0, 0, 0, robotData);
        odometryInitialized = true;
    }
    
}

void Drivebase::AutonomousInit(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData) 
{    

    lastDegrees.clear();
    odometryInitialized = false;

    // get trajectory from auton's pointer
    getNextAutonStep(robotData, drivebaseData, autonData);
    zeroEncoders();
}

void Drivebase::RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData, GyroData &gyroData, ControlData &controlData)
{
    updateData(robotData, drivebaseData);

    if (frc::DriverStation::IsEnabled())
    {
        dbL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbLF.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbRF.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }

    if (frc::DriverStation::IsTeleop()) 
    {
        teleopControl(robotData, drivebaseData, gyroData, controlData);
    }
    else if (frc::DriverStation::IsAutonomous())
    {
        autonControl(robotData, drivebaseData, autonData, gyroData);
    }


}

void Drivebase::DisabledInit()
{
    
    setPercentOutput(0, 0);
    // dbL.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // dbLF.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // dbR.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // dbRF.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    odometryInitialized = false;
}

double Drivebase::GetTurnMax()
{
    double maxTurn = 0.5;
    double minTurn = 0.15;

    double velocityAverage = abs((dbLEncoder.GetVelocity() + dbREncoder.GetVelocity())/2.0);
    double slope = (minTurn - maxTurn) / (5500);
    double b = maxTurn - (slope * 0);
    return ((slope * velocityAverage) + b);
}

// updates encoder and gyro values
void Drivebase::updateData(const RobotData &robotData, DrivebaseData &drivebaseData)
{
    // //add back wheel encoders at some point
    drivebaseData.currentLDBPos = dbLEncoder.GetPosition();
    drivebaseData.currentRDBPos = dbREncoder.GetPosition();

    drivebaseData.lDriveVel = -dbLEncoder.GetVelocity() / mpsToRpm ;
    // frc::SmartDashboard::PutNumber("lDriveVel", drivebaseData.lDriveVel);
    drivebaseData.rDriveVel = -dbREncoder.GetVelocity() / mpsToRpm;
    // frc::SmartDashboard::PutNumber("rDriveVel", -drivebaseData.rDriveVel);

    // WARNING the average calcuation here subtracts for some reason. The values for left and right db velocity act as expected on their own...
    drivebaseData.avgDriveVel = (drivebaseData.lDriveVel - drivebaseData.rDriveVel) / 2.0;
    // frc::SmartDashboard::PutNumber("avgDriveVel", drivebaseData.avgDriveVel);

    // option 1, will alloy us to shoot while pivoting drivebase
    drivebaseData.dbStationaryForShot = (std::abs(drivebaseData.avgDriveVel) < 0.1);
    // option 2, both driverails must be stationary for us to fire
    // drivebaseData.dbStationaryForShot = ((std::abs(drivebaseData.lDriveVel) < 0.2) && (std::abs(drivebaseData.rDriveVel) < 0.1));
    // frc::SmartDashboard::PutBoolean("dbStationaryForShot", drivebaseData.dbStationaryForShot);

    // frc::SmartDashboard::PutNumber("driveMode", drivebaseData.driveMode);

    // frc::SmartDashboard::PutNumber("odometry x", drivebaseData.odometryX);
    // frc::SmartDashboard::PutNumber("odometry y", drivebaseData.odometryY);

    // call updateOdometry
    updateOdometry(robotData, drivebaseData);
}
// driving functions:

// adjusts for the deadzone and converts joystick input to velocity values for PID
void Drivebase::teleopControl(const RobotData &robotData, DrivebaseData &drivebaseData, GyroData &gyroData, ControlData &controlData)
{

    if (robotData.elevatorData.drivebaseSlowMode)
    {
        drivebaseMultiplier = 0.45;
    }
    else
    {
        drivebaseMultiplier = 1;
    }
    // frc::SmartDashboard::PutNumber("DRIVE MODE", robotData.drivebaseData.driveMode);
    // frc::SmartDashboard::PutNumber("SHOOT MODE", robotData.controlData.shootMode);
    // assign drive mode
    if ((!robotData.controlData.vectorDrive) && ((robotData.controlData.lDrive <= -0.08 || robotData.controlData.lDrive >= 0.08) || (robotData.controlData.rDrive <= -0.08 || robotData.controlData.rDrive >= 0.08))) {
        drivebaseData.driveMode = DRIVEMODE_JOYSTICK;
    }
    else if (robotData.controlData.vectorDrive) 
    {
        drivebaseData.driveMode = DRIVEMODE_VECTOR;
    }  
    else 
    {
        drivebaseData.driveMode = DRIVEMODE_JOYSTICK;
    }


    if (drivebaseData.driveMode == DRIVEMODE_JOYSTICK) 
    {
        controlData.maxTurn = GetTurnMax();
        // frc::SmartDashboard::PutNumber("TURNMAX", GetTurnMax());
        double tempLDrive = robotData.controlData.lDrive;
        double tempRDrive = robotData.controlData.rDrive;

        // converts from tank to arcade drive, limits the difference between left and right drive
        double frontBack = robotData.controlData.maxStraight * (tempLDrive + tempRDrive) / 2;
        double leftRight = robotData.controlData.maxTurn * (tempRDrive - tempLDrive) / 2;

        //deadzone NOT needed for drone controller
        if (tempLDrive <= -0.08 || tempLDrive >= 0.08)
        {
            tempLDrive = (frontBack - leftRight);
        }
        else
        {
            tempLDrive = 0; 
        }

        if (tempRDrive <= -0.08 || tempRDrive >= 0.08)
        {
            tempRDrive = (frontBack + leftRight);
        }
        else
        {
            tempRDrive = 0;
        }

        // if (robotData.controlData.mode == MODE_AUTO_BALANCE)
        // {
        //     if (gyroData.rawPitch > 2.5)
        //     {
        //         tempLDrive = std::max((gyroData.rawPitch - 2.5)*0.1, 0.3);
        //         tempRDrive = std::max((gyroData.rawPitch - 2.5)*0.1, 0.3);

        //     }
        //     else if (gyroData.rawPitch < -2.5)
        //     {
        //         tempLDrive = std::max((-(gyroData.rawPitch) + 2.5)*0.1, 0.3);
        //         tempRDrive = std::max((-(gyroData.rawPitch) + 2.5)*0.1, 0.3);
        //     }
        //     else
        //     {
        //         tempLDrive = 0;
        //         tempRDrive = 0;
        //     }
        // }

        //set as percent vbus
        setPercentOutput(tempLDrive * drivebaseMultiplier, tempRDrive * drivebaseMultiplier);
    }
    else if (drivebaseData.driveMode == DRIVEMODE_TURNINPLACE) 
    {
        //turnInPlaceTeleop(-robotData.limelightData.angleOffset, robotData);
    }
    else if (drivebaseData.driveMode == DRIVEMODE_VECTOR)
    {
        //setPercentOutput(robotData.jetsonData.leftSkew, robotData.jetsonData.rightSkew);
    }
   


}

void Drivebase::autonControl(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData, GyroData &gyroData) 
{
    // sample the desired pos based on time from trajectory object
    // get chassis speeds from ramsete controller, comparing current pos w/ desired pos
    // translate chassis speeds to wheel speeds w/ toWheelSPeeds from kinematics using rate of turn and linear speed
    // feed wheel speeds to PID
    // check if done with current path by either checking TotalTime() or checking in vicinity of final target point

    // frc::smartDashboard::PutNumber("secSinceEnabled", robotData.timerData.secSinceEnabled);
    // frc::SmartDashboard::PutNumber("MODE",(int) drivebaseData.driveMode);

    if (drivebaseData.driveMode == DRIVEMODE_BREAK)
    {
        // frc::SmartDashboard::PutNumber("breakEndSec", breakEndSec);
        if (robotData.timerData.secSinceEnabled > breakEndSec /* && !robotData.controlData.saFinalShoot */) 
        {
            // frc::SmartDashboard::PutNumber("secSinceEnabled", robotData.timerData.secSinceEnabled);
            // frc::SmartDashboard::PutNumber("breakEndSec", breakEndSec);
            getNextAutonStep(robotData, drivebaseData, autonData);
        }
        setPercentOutput(0,0);
    }
    else if (drivebaseData.driveMode == DRIVEMODE_TURNINPLACE)
    {
        turnInPlaceAuton(turnInPlaceDegrees - robotData.gyroData.rawYaw, robotData, drivebaseData, autonData);
    }
    else if (drivebaseData.driveMode == DRIVEMODE_TRAJECTORY)
    {
        // frc::SmartDashboard::PutNumber("secSinceEnabled", robotData.timerData.secSinceEnabled);

        units::second_t sampleSec{robotData.timerData.secSinceEnabled - trajectorySecOffset};

        // frc::SmartDashboard::PutNumber("sampleSec", sampleSec.to<double>());

        double totalTime = trajectory.TotalTime().to<double>();
        // frc::SmartDashboard::PutNumber("trajTotalTime", totalTime);

        if (sampleSec.to<double>() > totalTime) 
        {
            getNextAutonStep(robotData, drivebaseData, autonData);
        }

        // frc::SmartDashboard::PutNumber("CURRENT TIME", sampleSec.to<double>());
        // frc::SmartDashboard::PutNumber("TARGET TIEM", 6);
        
        frc::Trajectory::State trajectoryState = trajectory.Sample(sampleSec);
        frc::Pose2d desiredPose = trajectoryState.pose;

        double trajX = desiredPose.X().to<double>();
        double trajY = desiredPose.Y().to<double>();
        // frc::SmartDashboard::PutNumber("trajX", trajX);
        // frc::SmartDashboard::PutNumber("trajY", trajY);

        frc::ChassisSpeeds chassisSpeeds = ramseteController.Calculate(odometry.GetPose(), trajectoryState);

        frc::DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

        double leftWheelSpeed = wheelSpeeds.left.to<double>();
        double rightWheelSpeed = wheelSpeeds.right.to<double>();
        // frc::SmartDashboard::PutNumber("leftWheelSpeed", leftWheelSpeed);
        // frc::SmartDashboard::PutNumber("rightWheelSpeed", rightWheelSpeed);

        setVelocity(leftWheelSpeed, rightWheelSpeed);
    }
}

void Drivebase::updateOdometry(const RobotData &robotData, DrivebaseData &drivebaseData) 
{

    // library's odometry
    units::radian_t currentRadians{(robotData.gyroData.rawYaw / 180) * M_PI};
    frc::Rotation2d currentRotation{currentRadians};

    // NEGATIVE because left motor/encoder should be inverted
    units::meter_t leftDistance{getEncoderDistance(dbLEncoder.GetPosition())}; 
    units::meter_t rightDistance{getEncoderDistance(dbREncoder.GetPosition())}; 

    // frc::SmartDashboard::PutNumber("left distance", getEncoderDistance(dbLEncoder.GetPosition()));

    // frc::SmartDashboard::PutNumber("UPDATELEGTY", (double)leftDistance);
    // frc::SmartDashboard::PutNumber("UPDATERIGHT", (double)rightDistance);
    odometry.Update(currentRotation, leftDistance, rightDistance);

    field.SetRobotPose(odometry.GetPose());
    // frc::SmartDashboard::PutData("Field", &field);


    drivebaseData.currentPose = odometry.GetPose();
    drivebaseData.odometryX = drivebaseData.currentPose.X().to<double>();
    drivebaseData.odometryY = drivebaseData.currentPose.Y().to<double>();

    drivebaseData.odometryYaw = drivebaseData.currentPose.Rotation().Degrees().to<double>();
    // drivebaseData.odometryYaw = (drivebaseData.odometryYaw / M_PI * 180); // convert from radians [-pi, pi] to degrees [0, 360]
    if (drivebaseData.odometryYaw < 0) 
    {
        drivebaseData.odometryYaw = 360 + drivebaseData.odometryYaw;
    }
    // frc::SmartDashboard::PutNumber("odometryX", drivebaseData.odometryX);
    // frc::SmartDashboard::PutNumber("odometryY", drivebaseData.odometryY);
    // frc::SmartDashboard::PutNumber("odometryYaw", drivebaseData.odometryYaw);
}

/**
 * @param pose position to reset odometry to (Pose2d)
 * @param resetAngle angle to reset odometry to (degrees, double)
 */
void Drivebase::resetOdometry(const frc::Pose2d &pose, double gyroAngle) 
{
    const units::radian_t gyroRadians{gyroAngle};
    frc::Rotation2d gyroRotation{gyroRadians};

    odometry.ResetPosition(gyroRotation, units::meter_t{getEncoderDistance(dbLEncoder.GetPosition())}, units::meter_t{getEncoderDistance(dbREncoder.GetPosition())},  pose);
    //zeroEncoders();
}

// reset odometry to any double x, y, deg
void Drivebase::resetOdometry(double x, double y, double radians, const RobotData &robotData) 
{
    const units::meter_t meterX{x};
    const units::meter_t meterY{y};

    const units::radian_t radianYaw{radians};
    // frc::SmartDashboard::PutNumber("Pi", pi);
    // frc::SmartDashboard::PutNumber("radian yaw", robotData.gyroData.rawYaw / 180 * pi);

    const units::radian_t gyroRadians{robotData.gyroData.rawYaw / 180 * M_PI};
    // frc::SmartDashboard::PutNumber("RORaw Yaw", robotData.gyroData.rawYaw);

    const frc::Rotation2d gyroRotation{gyroRadians};
    const frc::Pose2d resetPose{meterX, meterY, radianYaw};
    odometry.ResetPosition(gyroRotation, units::meter_t{-getEncoderDistance(dbLEncoder.GetPosition())}, units::meter_t{getEncoderDistance(dbREncoder.GetPosition())},  resetPose);

    //zeroEncoders();
}

double Drivebase::getEncoderDistance(double encoderPosition)
{
    return encoderPosition*rotationsToMeters;
}


// sets the drive base velocity for auton
void Drivebase::setVelocity(double leftVel, double rightVel)
{
    double leftRPM = leftVel * mpsToRpm;
    double rightRPM = rightVel * mpsToRpm;

    // frc::SmartDashboard::PutNumber("left rpm", leftRPM);
    // frc::SmartDashboard::PutNumber("right rpm", rightRPM);

    dbLPIDController.SetReference(leftRPM, rev::CANSparkMax::ControlType::kVelocity); // dbL.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, leftTPDS);
    dbRPIDController.SetReference(rightRPM, rev::CANSparkMax::ControlType::kVelocity); // dbR.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, rightTPDS);
}

void Drivebase::zeroEncoders() 
{
    dbLEncoder.SetPosition(0);
    dbREncoder.SetPosition(0);
    // frc::SmartDashboard::PutString("zeroed encoders", "yes");
}

// get the frc::Pose2d of a set of x, y meters & yaw degrees
frc::Pose2d Drivebase::getPose(double x, double y, double deg) 
{
    units::meter_t meterX{x};
    units::meter_t meterY{y};

    const units::radian_t radianYaw{deg / 180 * M_PI};
    const frc::Rotation2d rotation{radianYaw};
    frc::Pose2d pose{meterX, meterY, rotation};
    return pose;
}

void Drivebase::getNextAutonStep(const RobotData &robotData, DrivebaseData &drivebaseData,  AutonData &autonData) 
{

    autonData.autonStep++;

    if (autonData.autonStep < autonData.pathGroup.size()) 
    {
        // frc::SmartDashboard::PutString("getNextAutonStep()", "b");
        // frc::SmartDashboard::PutNumber("autonStep", autonData.autonStep);
        // frc::SmartDashboard::PutString("robotData.autonData.pathGroup[robotData.autonData.autonStep", autonData.pathGroup[autonData.autonStep]);

        std::string trajectoryName = autonData.pathGroup.at(autonData.autonStep);
        // frc::SmartDashboard::PutString("K", trajectoryName);

        // frc::SmartDashboard::PutString("trajectoryName", trajectoryName);

        if (trajectoryName.substr(0, 11) == "turnInPlace")
        {
            drivebaseData.driveMode = DRIVEMODE_TURNINPLACE;
            turnInPlaceDegrees = robotData.gyroData.rawYaw + std::stod(trajectoryName.substr(12, trajectoryName.length()));

            return;
        }

        else if (trajectoryName.substr(0, 5) == "break")
        {
            drivebaseData.driveMode = DRIVEMODE_BREAK;
            breakEndSec = std::stod(trajectoryName.substr(6, trajectoryName.length())) + robotData.timerData.secSinceEnabled;
            return;
        }

        else 
        {
            drivebaseData.driveMode = DRIVEMODE_TRAJECTORY; 

            fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

            fs::path pathDirectory = deployDirectory / "output" / (trajectoryName + ".wpilib.json");

            // frc::SmartDashboard::PutString("pathDirectory", pathDirectory.string());

            trajectory = frc::TrajectoryUtil::FromPathweaverJson(pathDirectory.string());
            // frc::SmartDashboard::PutNumber("original seconds since enabled", robotData.timerData.secSinceEnabled);
            trajectorySecOffset = robotData.timerData.secSinceEnabled;

            
            if (!odometryInitialized) 
            {
                // automatically reset odometry to start pose of the first path
                units::second_t zeroSec{0};


                frc::Trajectory::State trajectoryState = trajectory.Sample(zeroSec);
                frc::Pose2d firstPose = trajectoryState.pose;

                double firstX = firstPose.X().to<double>();
                double firstY = firstPose.Y().to<double>();
                
                double firstRadians = firstPose.Rotation().Radians().to<double>();
                // frc::SmartDashboard::PutNumber("firstRadians", firstRadians);

                resetOdometry(firstX, firstY, firstRadians, robotData);
                // frc::smartDashboard::PutNumber("autonStep OdoInit", autonData.autonStep);

                odometryInitialized = true;
            }

            // frc::smartDashboard::PutBoolean("odometryInitialized", odometryInitialized);
        }

        // frc::SmartDashboard::PutNumber("autonStep", autonData.autonStep);
    }
    else 
    {
        drivebaseData.driveMode = DRIVEMODE_BREAK;
    }
}

void Drivebase::turnInPlaceAuton(double degrees, const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData) 
{

    // frc::SmartDashboard::PutNumber("degree diff", degrees);
    
    lastDegrees.push_back(degrees);
    if (lastDegrees.size() > 2) 
    {
        lastDegrees.pop_front();
    }

    double leftOutput = 0;
    double rightOutput = 0;

    int directionFactor = 1;
    if (degrees <= 0) 
    {
        directionFactor = -1;
    }

    // frc::smartDashboard::PutBoolean("allValuesWithin", allValuesWithin(lastDegrees, 1));
    if (allValuesWithin(lastDegrees, 5)) 
    {
        setPercentOutput(0, 0);
        // only advance auton step if it's not shooting
        if (drivebaseData.driveMode == DRIVEMODE_TURNINPLACE) 
        {
            getNextAutonStep(robotData, drivebaseData, autonData);
        }
        // frc::SmartDashboard::PutString("AUTON", "TURN IN PLACE");
    } else 
    {
        // profile that adjusts aggressiveness of turn based on the amount of degrees left to turn. has been tuned for speed & accuracy on both small and large turns
        leftOutput = std::pow(std::abs(degrees / 400), 1.5) + 0.13;
        rightOutput = std::pow(std::abs(degrees / 400), 1.5) + 0.13;
    }
    

    // frc::SmartDashboard::PutNumber("leftOutput", leftOutput);
    // frc::SmartDashboard::PutNumber("rightOutput", rightOutput);
    
    setPercentOutput(leftOutput * (-directionFactor), rightOutput * (directionFactor));
}

void Drivebase::turnInPlaceTeleop(double degrees, const RobotData &robotData) 
{
    // frc::SmartDashboard::PutNumber("degree diff", degrees);
    
    lastDegrees.push_back(degrees);
    if (lastDegrees.size() > 5) 
    {
        lastDegrees.pop_front();
    }

    double leftOutput = 0;
    double rightOutput = 0;

    int directionFactor = 1;
    if (degrees <= 0) 
    {
        directionFactor = -1;
    }

    if (allValuesWithin(lastDegrees, 4)) 
    {
        setPercentOutput(0, 0);
        // frc::SmartDashboard::PutString("TELEOP", "TURN IN PLACE");
    } else 
    {
        leftOutput = std::pow(std::abs(degrees / 400), 1.3) + 0.09;
        rightOutput = std::pow(std::abs(degrees / 400), 1.3) + 0.09;
    }
    

    // frc::SmartDashboard::PutNumber("leftOutput", leftOutput);
    // frc::SmartDashboard::PutNumber("rightOutput", rightOutput);
    
    setPercentOutput(leftOutput * (-directionFactor), rightOutput * (directionFactor));
}


void Drivebase::setPercentOutput(double leftVBus, double rightVBus) 
{
    dbL.Set(leftVBus);
    dbR.Set(rightVBus);
}

// checks deque contents to see if all values are within the given tolerance (true)
bool Drivebase::allValuesWithin(std::deque<double> deque, double tolerance) 
{
    bool hasOutlier = false;
    for (size_t i = 0; i < deque.size(); i++) 
    {
        if (std::abs(deque[i]) > tolerance) 
        {
            hasOutlier = true;
        }
    }
    return !hasOutlier;
}

void Drivebase::sendStartPointChooser() 
{
    startPointChooser.AddOption("(0, 0), 0 deg", getPose(0, 0, 0));
    startPointChooser.AddOption("(3, 1), 90 deg", getPose(3, 1, 90));
    // frc::SmartDashboard::PutData("Select Start Point:", &startPointChooser);
}


void Drivebase::DisabledPeriodic()
{
    // frc::SmartDashboard::PutNumber("LEFT DISTANCE", dbLEncoder.GetPosition());
}

