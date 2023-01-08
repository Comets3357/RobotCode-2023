#pragma once

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <cmath>

struct RobotData;

enum Mode {
    mode_teleop_sa,
    mode_teleop_manual,
    mode_climb_sa,
    mode_climb_manual
};

enum ShootMode {
    shootMode_none,
    shootMode_vision,
    shootMode_fender,
    shootMode_sideWall,
    shootMode_wallLaunchPad,
    shootMode_cornerLaunchPad
};

struct ControlData
{
    // states:
    Mode mode{mode_teleop_sa};
    ShootMode shootMode = shootMode_none;
    
    bool shift = false;

    // drivebase:
    double lDrive;
    double rDrive;
    bool turnResponsive;
    bool dbInverted;
    double maxStraight = 1;
    double maxTurn = 0.4;
    bool vectorDrive;

    //intake:
    bool mIntakeDown; 
    //brings the intake down MANUAL (hold)
    bool mIntakeUp; // (hold)
    bool mIntakeRollersIn; //runs intake forward MANUAL (axis)
    bool mIntakeRollersOut; //runs intake backward MANUAL (axis)
    bool mZeroIntakePivot;
    bool mZeroHood; //set the hood encoder to zero MANUAL
    bool mZeroTurret; //set the turret encoder to zero MANUAL
    bool saIntake; //runs the intake rollers and brings intake down and the indexer to intake balls SEMIAUTO
    bool saIntakeBackward; //runs the intake backwards SEMIAUTO
    bool saIntakeIdle; // turns intakeIdle true while held

    //indexer:
    bool mSideWheelForward;
    bool mSideWheelBackward;
    bool mCenterWheelForward;
    bool mCenterWheelBackward;
    bool mIndexerDown; //runs indexer backwards MANUAL
    bool mIndexerUp; //runs indexer foward MANUAL
    bool saEjectBalls; //runs intake and indexer backwards to eject balls SEMIAUTO
    bool mDecrementCargo; // manually decrements the amount of cargo in the indexer from the front of the deque
    bool mIncrementCargo; // manually increments the amount of cargo in the indexer from the front of the deque

    //shooter:
    bool saShooting; //gets hood at right angle, shooter wheel up to speed SEMIAUTO
    bool saFinalShoot; //makes belts run to actually fire balls SEMIAUTO
    bool saIndexerShooting;   // semi auto command
    bool staticTurret;

    bool upperHubShot = true;
    bool cornerLaunchPadShot; //fixed long shot from the launch pad to upper hub SEMIAUTO
    bool wallLaunchPadShot;
    bool sideWallShot;
    bool fenderShot;
    bool hubShot;//fixed close shot to lower hub from infront of hub SEMIAUTO
    bool autoRejectOpponentCargo = true;
    bool wrongBall; //if the ball isn't our alliance color eject ball out shooter SEMIAUTO  DEPRECATED (using ControlData.autoRejectOpponentCargo)
    bool mShooterWheelForward; //get flywheel running MANUAL
    bool mShooterWheelBackward; //get flywheel running backward MANUAL
    double mHood; //moves hood up or down MANUAL (axis)
    double mTurret; // moves turret left or right MANUAL (axis)
    float saTurretDirectionController; //move the turret in the direction of the joystick
    bool usingTurretDirection;
    float saDistanceOffset = 0; //everytime pressed, add or subtract six inches to distance

    //climb:
    bool saClimbTraversalSequence;
    bool saClimbHeightSequence;
    bool saCancelSequence;
    bool saClimbInit;
    bool climbZeroing;
    bool saPauseSequence;
    bool mClimbZeroElevatorRev;
    bool mClimbZeroPivotArmsRev;
    
    double mElevatorExtension;
    double mArmPivot;

    //benchTest:

    bool manualBenchTest = false;
    bool autoBenchTest = false;
    bool incrementMotor = false;
    bool incrementSpeed = false;
    bool incrementSubsystem = false;
    bool PIDModeToggle = false;
};

struct ControllerData
{
    // btn data:
    // L = left, R = right, p = primary, s = secondary, Btn = button

    // primary:

    double pLXStick = 0;
    double pLYStick = 0;
    double pRXStick = 0;
    double pRYStick = 0;

    bool pLShoulderSwitch = false;
    bool pRShoulderSwitch = false;
    bool pLPalmSwitch = false;
    bool pRPalmSwitch = false;

    // secondary:

    double sLXStick = 0;
    double sLYStick = 0;
    double sRXStick = 0;
    double sRYStick = 0;

    bool sLStickBtn = false;
    bool sRStickBtn = false;

    bool sLStickBtnToggled = false;
    bool sRStickBtnToggled = false;

    double sLTrigger = 0;
    double sRTrigger = 0;
    bool sLBumper = false;
    bool sLBumperToggled = false;
    bool sRBumper = false;
    bool sRBumperToggled = false;

    bool sXBtn = false;
    bool sYBtn = false;
    bool sABtn = false;
    bool sBBtn = false;

    bool sABtnToggled = false;
    bool sBBtnToggled = false;
    bool sXBtnToggled = false;
    bool sYBtnToggled = false;

    bool sLCenterBtn = false;
    bool sRCenterBtn = false;

    bool sLCenterBtnToggled = false;
    bool sRCenterBtnToggled = false;

    bool sLTriggerToggled = false;
    bool sRTriggerToggled = false;

    int sDPad = -1;

    bool testAButton = false;
    bool testBButton = false;
    bool testXButton = false;
    bool testYButton = false;
    bool testRBumper = false;
    bool testLBumper = false;
};

class Controller
{

    public:
        void TeleopPeriodic(const RobotData &robotData, ControllerData &controllerData, ControlData &controlData);
        void TestPeriodic(const RobotData &robotData, ControllerData &controllerData, ControlData &controlData);
    private:
        /**
         * Don't touch "Controller.cpp" that is for the direct access to joystick buttons
         * when writing code and assigning it to specific button or axis, 
         * write to it through "ControlData.cpp"
         * */

        void updateBtnData(ControllerData &controllerData);
        void updateControlData(const RobotData &robotData, const ControllerData &controllerData, ControlData &controlData);
        void updateShootMode(const RobotData &robotData, ControlData &controlData);

        // basic btn getters:
        bool getBtn(int js, int index);
        bool getBtnToggled(int js, int index);
        int getPOV(int js, int index);
        double getAxis(int js, int index);

        frc::Joystick primary{0};
        frc::Joystick secondary{1};
        frc::Joystick testControl{2};
};

