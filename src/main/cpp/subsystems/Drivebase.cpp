#include "subsystems/Drivebase.h"
#include "RobotData.h"


void Drivebase::RobotInit(const RobotData &robotData)
{
    if (
        dbL.GetInverted() != robotData.configData.drivebaseConfigData.leftInverted ||
        dbLPIDController.GetP() != 0.077396 / mpsToRpm ||
        dbLPIDController.GetFF() != 0.051094 / mpsToRpm ||
        dbL.GetIdleMode() != rev::CANSparkMax::IdleMode::kBrake
    )
    {
        dbL.RestoreFactoryDefaults();
        dbL.SetInverted(robotData.configData.drivebaseConfigData.leftInverted);
        dbL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbL.SetSmartCurrentLimit(robotData.configData.drivebaseConfigData.currentLimit);

        dbLPIDController.SetP(0.077396 / mpsToRpm);
        dbLPIDController.SetFF(0.051094 / mpsToRpm);
        dbLPIDController.SetD(0);

        dbL.BurnFlash();
    }

    if (
        dbR.GetInverted() != robotData.configData.drivebaseConfigData.rightInverted ||
        dbRPIDController.GetP() != 0.077396 / mpsToRpm ||
        dbRPIDController.GetFF() != 0.051094 / mpsToRpm ||
        dbR.GetIdleMode() != rev::CANSparkMax::IdleMode::kBrake
    )
    {
        dbR.RestoreFactoryDefaults();
        dbR.SetInverted(robotData.configData.drivebaseConfigData.rightInverted);
        dbR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbR.SetSmartCurrentLimit(robotData.configData.drivebaseConfigData.currentLimit);

        dbRPIDController.SetP(0.077396 / mpsToRpm);
        dbRPIDController.SetFF(0.051094 / mpsToRpm);
        dbRPIDController.SetD(0);

        dbR.BurnFlash();
    }

    if (
        dbRF.IsFollower() == false ||
        dbRF.GetIdleMode() != rev::CANSparkMax::IdleMode::kBrake
    )
    {
        dbRF.RestoreFactoryDefaults();
        dbRF.Follow(dbR);
        dbRF.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbRF.SetSmartCurrentLimit(robotData.configData.drivebaseConfigData.currentLimit);

        dbRF.BurnFlash();
    }

    if (
        dbLF.IsFollower() == false ||
        dbLF.GetIdleMode() != rev::CANSparkMax::IdleMode::kBrake
    )
    {
        dbLF.RestoreFactoryDefaults();
        dbLF.Follow(dbR);
        dbLF.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbLF.SetSmartCurrentLimit(robotData.configData.drivebaseConfigData.currentLimit);

        dbLF.BurnFlash();
    }

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
        zeroEncoders();
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
        autonControl(robotData, drivebaseData, autonData, gyroData, controlData);
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

    frc::SmartDashboard::PutNumber("odometry x", drivebaseData.odometryX);
    frc::SmartDashboard::PutNumber("odometry y", drivebaseData.odometryY);

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
        frc::SmartDashboard::PutNumber("TURNMAX", GetTurnMax());
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

        if (robotData.controlData.mode == MODE_AUTO_BALANCE)
        {
            if (gyroData.rawPitch > 2.5)
            {
                tempLDrive = std::max((gyroData.rawPitch - 2.5)*0.05, 0.3);
                tempRDrive = std::max((gyroData.rawPitch - 2.5)*0.05, 0.3);

            }
            else if (gyroData.rawPitch < -2.5)
            {
                tempLDrive = std::max((-(gyroData.rawPitch) + 2.5)*0.05, 0.3);
                tempRDrive = std::max((-(gyroData.rawPitch) + 2.5)*0.05, 0.3);
            }
            else
            {
                tempLDrive = 0;
                tempRDrive = 0;
            }
        }

        if ((robotData.controlData.mode == MODE_TELEOP_ADVANCED_SA) &&
            (tempLDrive < 0.08 && tempLDrive > -0.08) &&
            (tempRDrive < 0.08 && tempRDrive > -0.08) &&
            (robotData.controlData.saPositionHigh || robotData.controlData.saPositionMid))
        {

            //temp
            double angleOff;
            double distanceOff;

            if (!profileCreated)
            {
                startTime = robotData.timerData.secSinceEnabled;
                leftStartPosition = 0.0;
                rightEndPosition = 0.0;
                leftEndPosition = leftStartPosition + (angleOff * degreesToMeters);
                rightEndPosition = rightStartPosition - (angleOff * degreesToMeters);

                leftProfile = frc::TrapezoidProfile<units::meters>
                {
                    frc::TrapezoidProfile<units::meters>::Constraints{units::velocity::meters_per_second_t{1}, units::acceleration::meters_per_second_squared_t{1}},
                    frc::TrapezoidProfile<units::meters>::State{units::meter_t{leftEndPosition}, units::meters_per_second_t{0}},
                    frc::TrapezoidProfile<units::meters>::State{units::meter_t{leftStartPosition}, units::meters_per_second_t{0}}
                };

                rightProfile = frc::TrapezoidProfile<units::meters>
                {
                    frc::TrapezoidProfile<units::meters>::Constraints{units::velocity::meters_per_second_t{1}, units::acceleration::meters_per_second_squared_t{1}},
                    frc::TrapezoidProfile<units::meters>::State{units::meter_t{rightEndPosition}, units::velocity::meters_per_second_t{0}},
                    frc::TrapezoidProfile<units::meters>::State{units::meter_t{rightStartPosition, units::velocity::meters_per_second_t{0}}
                };
            }
            
            // turn
            // once turn, then drive
        }
        else
        {
            setPercentOutput(tempLDrive * drivebaseMultiplier, tempRDrive * drivebaseMultiplier);
        }
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

void Drivebase::autonControl(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData, GyroData &gyroData, ControlData &controlData) 
{
    

    double temporaryX;
    double temporaryY;
    // sample the desired pos based on time from trajectory object
    // get chassis speeds from ramsete controller, comparing current pos w/ desired pos
    // translate chassis speeds to wheel speeds w/ toWheelSPeeds from kinematics using rate of turn and linear speed
    // feed wheel speeds to PID
    // check if done with current path by either checking TotalTime() or checking in vicinity of final target point
double tempLDrive = 0;
    double tempRDrive = 0;
    // frc::smartDashboard::PutNumber("secSinceEnabled", robotData.timerData.secSinceEnabled);
    frc::SmartDashboard::PutNumber("MODE",(int) drivebaseData.driveMode);

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
        frc::SmartDashboard::PutNumber("secSinceEnabled", robotData.timerData.secSinceEnabled);

        units::second_t sampleSec{robotData.timerData.secSinceEnabled - trajectorySecOffset};

        frc::SmartDashboard::PutNumber("sampleSec", sampleSec.to<double>());

        double totalTime = trajectory.TotalTime().to<double>();
        // frc::SmartDashboard::PutNumber("trajTotalTime", totalTime);

        if ((sampleSec.to<double>() > totalTime)) 
        {
            getNextAutonStep(robotData, drivebaseData, autonData);
        }

        frc::SmartDashboard::PutNumber("CURRENT TIME", sampleSec.to<double>());
        frc::SmartDashboard::PutNumber("TARGET TIEM", 6);
        
        frc::Trajectory::State trajectoryState = trajectory.Sample(sampleSec);

        // trajectoryState.pose.Y() = units::meter_t(trajectoryState.pose.Y().to<double>() + 2);

        frc::Pose2d desiredPose = trajectoryState.pose;

        // trajectoryState.pose.Y() = units::meter(desiredPose.Y().to<double>() + 1);

        double trajX = desiredPose.X().to<double>();
        double trajY = desiredPose.Y().to<double>();
        frc::SmartDashboard::PutNumber("trajX", trajX);
        frc::SmartDashboard::PutNumber("trajY", trajY);

        frc::ChassisSpeeds chassisSpeeds = ramseteController.Calculate(odometry.GetEstimatedPosition(), trajectoryState);

        frc::DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

        double leftWheelSpeed = wheelSpeeds.left.to<double>();
        double rightWheelSpeed = wheelSpeeds.right.to<double>();
        frc::SmartDashboard::PutNumber("leftWheelSpeed", leftWheelSpeed);
        frc::SmartDashboard::PutNumber("rightWheelSpeed", rightWheelSpeed);

        temporaryX = trajX;
        temporaryY = trajX;

        setVelocity(leftWheelSpeed, rightWheelSpeed);
    }
    else if (drivebaseData.driveMode == DRIVEMODE_CHARGE_STATION_TRAVERSE)
    {
        if (!forward)
        {
            
            setVelocity(-5+(gyroData.rawYaw*0.05), -5-(gyroData.rawYaw*0.05));

            if (robotData.gyroData.rawRoll > 5)
            {
                ChargeStationTraverseStep = 1;
            }
            if (ChargeStationTraverseStep == 1 && robotData.gyroData.rawRoll < 3)
            {
                setVelocity(0,0);
                odometryInitialized = false;
                // controlData.saResetOdometry = true;
                getNextAutonStep(robotData, drivebaseData, autonData);
            }
            
        }
        else
        {
            setVelocity(5+(gyroData.rawYaw*0.05), 5-(gyroData.rawYaw*0.05));

            
            if (robotData.gyroData.rawRoll < -5)
            {
                ChargeStationTraverseStep = 1;
            }
            if (ChargeStationTraverseStep == 1 && robotData.gyroData.rawRoll > -3)
            {
               setVelocity(0,0);
                    //zeroEncoders();
                //resetOdometry(14.392, 2.717, gyroData.rawYaw * 3.14159/180.0, robotData);
                //odometryInitialized = false;
                getNextAutonStep(robotData, drivebaseData, autonData);
                
                
            }
        }
        
    }
    

    else if (drivebaseData.driveMode == DRIVEMODE_AUTO_BALANCE)
        {
            tempLDrive = gyroData.rawRoll*-0.008;
            tempRDrive = gyroData.rawRoll*-0.008;
            setPercentOutput(tempLDrive, tempRDrive);

        }

        //set as percent vbus
}

void Drivebase::updateOdometry(const RobotData &robotData, DrivebaseData &drivebaseData) 
{

    // library's odometry
    units::radian_t currentRadians{(robotData.gyroData.rawYaw / 180.0) * M_PI};
    frc::Rotation2d currentRotation{currentRadians};

    // NEGATIVE because left motor/encoder should be inverted
    units::meter_t leftDistance{getEncoderDistance(dbLEncoder.GetPosition())}; 
    units::meter_t rightDistance{getEncoderDistance(dbREncoder.GetPosition())}; 

    frc::SmartDashboard::PutNumber("left distance", getEncoderDistance(dbLEncoder.GetPosition()));

    frc::SmartDashboard::PutNumber("UPDATE LEFT", (double)leftDistance);
    frc::SmartDashboard::PutNumber("UPDATE RIGHT", (double)rightDistance);

    frc::SmartDashboard::PutBoolean("vision able to reset path", robotData.limelightData.limelightAllowedToReset);

    if (robotData.limelightData.limelightAllowedToReset && robotData.controlData.saResetOdometry)
    {
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            // resetOdometry(robotData.limelightData.limelightOdometryX, robotData.limelightData.limelightOdometryY, currentRadians.to<double>(), robotData);
            // odometry.AddVisionMeasurement(robotData.limelightData.Odometry, )

            odometry.AddVisionMeasurement(robotData.limelightData.Odometry, frc::Timer::GetFPGATimestamp() - units::time::second_t{robotData.limelightData.latency});
        }
        else
        {
            // zeroEncoders();
            // resetOdometry(robotData.limelightData.limelightOdometryX, robotData.limelightData.limelightOdometryY, currentRadians.to<double>() + M_PI, robotData);
            odometry.AddVisionMeasurement(robotData.limelightData.Odometry, frc::Timer::GetFPGATimestamp() - units::time::second_t{robotData.limelightData.latency});
        }
    }
    // else
    // {
    //     odometry.Update(currentRotation, leftDistance, rightDistance);
    // }

    odometry.UpdateWithTime(frc::Timer::GetFPGATimestamp(), currentRotation, leftDistance, rightDistance);
    
    field.SetRobotPose(odometry.GetEstimatedPosition());
    frc::SmartDashboard::PutData("Field", &field);

    drivebaseData.currentPose = odometry.GetEstimatedPosition();

    drivebaseData.odometryX = drivebaseData.currentPose.X().to<double>();
    drivebaseData.odometryY = drivebaseData.currentPose.Y().to<double>();
    

    // drivebaseData.odometryX = robotData.limelightData.limelightOdometryX;
    // drivebaseData.odometryY = robotData.limelightData.limelightOdometryY;

        // drivebaseData.odometryX = drivebaseData.currentPose.X().to<double>();
        // drivebaseData.odometryY = drivebaseData.currentPose.Y().to<double>();

    drivebaseData.odometryYaw = drivebaseData.currentPose.Rotation().Degrees().to<double>();
    // drivebaseData.odometryYaw = (drivebaseData.odometryYaw / M_PI * 180); // convert from radians [-pi, pi] to degrees [0, 360]
    if (drivebaseData.odometryYaw < 0) 
    {
        drivebaseData.odometryYaw = 360 + drivebaseData.odometryYaw;
    }
    frc::SmartDashboard::PutNumber("odometryX", drivebaseData.odometryX);
    frc::SmartDashboard::PutNumber("odometryY", drivebaseData.odometryY);
    frc::SmartDashboard::PutNumber("odometryYaw", drivebaseData.odometryYaw);
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
    // zeroEncoders();
    odometry.ResetPosition(gyroRotation, units::meter_t{getEncoderDistance(dbLEncoder.GetPosition())}, units::meter_t{getEncoderDistance(dbREncoder.GetPosition())},  resetPose);

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

    frc::SmartDashboard::PutNumber("left rpm", leftRPM);
    frc::SmartDashboard::PutNumber("right rpm", rightRPM);

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
        frc::SmartDashboard::PutString("K", trajectoryName);

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

        else if (trajectoryName.substr(0,28) == "chargeStationTraverseForward")
        {
            ChargeStationTraverseStep = 0;
            forward = true;
            drivebaseData.driveMode = DRIVEMODE_CHARGE_STATION_TRAVERSE;
            return;
        }
        else if (trajectoryName.substr(0,29) == "chargeStationTraverseBackward")
        {
            ChargeStationTraverseStep = 0;
            forward = false;
            drivebaseData.driveMode = DRIVEMODE_CHARGE_STATION_TRAVERSE;
            return;
        }
        else if (trajectoryName.substr(0,7) == "balance")
        {
            drivebaseData.driveMode = DRIVEMODE_AUTO_BALANCE;
        }

        else 
        {
            drivebaseData.driveMode = DRIVEMODE_TRAJECTORY; 

            fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

            fs::path pathDirectory = deployDirectory / "output" / (trajectoryName + ".wpilib.json");

            frc::SmartDashboard::PutString("pathDirectory", pathDirectory.string());

            trajectory = frc::TrajectoryUtil::FromPathweaverJson(pathDirectory.string());
            frc::SmartDashboard::PutNumber("original seconds since enabled", robotData.timerData.secSinceEnabled);
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
                // zeroEncoders();
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
        leftOutput = (std::pow(std::abs(degrees / 400), 1.5) + 0.13) * 4.0;
        rightOutput = (std::pow(std::abs(degrees / 400), 1.5) + 0.13) * 4.0;
    }
    

    frc::SmartDashboard::PutNumber("leftOutput", leftOutput);
    frc::SmartDashboard::PutNumber("rightOutput", rightOutput);
    
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


void Drivebase::DisabledPeriodic(const RobotData &robotData)
{
    frc::SmartDashboard::PutNumber("LEFT DISTANCE", dbLEncoder.GetPosition());
    // if (robotData.controllerData.sABtn)
    // {
    //         dbL.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // dbLF.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // dbR.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // dbRF.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // }
}

