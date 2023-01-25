#pragma once

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <cmath>

struct RobotData;

enum Mode {
    MODE_TELEOP_SA,
    MODE_TELEOP_MANUAL,
    MODE_AUTO_BALANCE
    
};

struct ControlData
{
    // states:
    Mode mode{MODE_TELEOP_SA};
    
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

    bool mIntakeDown; // TAKE OUT.
    bool mIntakeUp; // TAKE OUT.
    bool mIntakeRollersIn;
    bool mIntakeRollersOut;
    bool mForceZeroIntake; // TAKE OUT.

    bool saIntaking;
    bool saIntakeBackwards;
    bool saCubeIntake;

    //arm:

    bool saMoveArm;
    bool saArmIntakePosition;
    bool saArmPositionTwo;
    bool saArmPositionThree;

    bool mMovePivot;
    bool mMoveWrist;

    bool mForceZeroWrist;
    bool mForceZeroPivot;

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

