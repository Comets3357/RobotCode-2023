#include "controller/Controller.h"
#include "RobotData.h"

// for updating states of control variables (to be accessed by other subsystems)
void Controller::updateControlData(const RobotData &robotData, const ControllerData &controllerData, ControlData &controlData)
{
    frc::SmartDashboard::PutBoolean("Semi-Auto", controlData.mode == MODE_TELEOP_SA);
    // states:
    controlData.shift = controllerData.sLBumper;
    
    switch (controllerData.sDPad) {
        case -1:
            break;
        case 0: // up
            controlData.mode = MODE_TELEOP_MANUAL;
            break;
        case 90:    // right
            controlData.mode = MODE_TELEOP_SA;
            break;
        case 180:   // down
            controlData.mode = MODE_TELEOP_DISABLE_BEAMS;
            break;
        case 270:   // left
            //controlData.mode = MODE_TELEOP_ADVANCED_SA;
            break;
        default:
            controlData.mode = MODE_TELEOP_SA;
            break;
            
    }
//-----------------------------------------------------------------------------------------------------------------------------------
//        DRIVEBASE / PRIMARY DRIVER BUTTONS:
//-----------------------------------------------------------------------------------------------------------------------------------

    // note: when pRShoulderSwitch is held, driving is sensitive to turning, while not held (default driving mode) driving is less sensitive to turning and good for quick staright movements and steady arcs (won't turn super easily)
    // controlData.turnResponsive = controllerData.pRShoulderSwitch;
    controlData.substationLineUp = controllerData.pRShoulderSwitch;
    if (controlData.turnResponsive)
    {
        controlData.maxStraight = 1;
        controlData.maxTurn = 1;
    }
    else
    {
        controlData.maxStraight = 1;
        controlData.maxTurn = 0.3;
    }

    controlData.dbInverted = controllerData.pLPalmSwitch;
    // if you're inverted then you swtich sides for driving so it's intuitive
    if (controlData.dbInverted)
    {
        controlData.lDrive = -controllerData.pRYStick;
        controlData.rDrive = -controllerData.pLYStick;
    }
    else
    {
        controlData.lDrive = controllerData.pLYStick;
        controlData.rDrive = controllerData.pRYStick;
    }

//-----------------------------------------------------------------------------------------------------------------------------------
//        MANIPULATOR / SECONDARY DRIVER BUTTONS:
//-----------------------------------------------------------------------------------------------------------------------------------

    // SEMI AUTO:

    if ((robotData.endEffectorData.gamePieceType == CONE || robotData.endEffectorData.gamePieceType == CUBE) && (robotData.controlData.mode != MODE_TELEOP_DISABLE_BEAMS))
    {
        controlData.saConeIntake = false;
        controlData.saCubeIntake = false;
        controlData.saUprightConeIntake = false;
    }
    else 
    {
        controlData.saUprightConeIntake = (controllerData.sXBtn) && controlData.shift;
        controlData.saCubeIntake = (controllerData.sLTrigger > 0.5);
        controlData.saConeIntake = (controllerData.sRTrigger > 0.5) && !controlData.shift;
    }

    // if (robotData.endEffectorData.armRetractRequest)
    // {
    //     controlData.saHomePosition = true;
    // }
    // else
    // {
    //     controlData.saHomePosition = (controllerData.sABtn) && !controlData.shift;
    // }

    if ((robotData.endEffectorData.pastReadOfGamePiece != NONE) && (robotData.endEffectorData.gamePieceType == NONE) && (robotData.controlData.mode != MODE_TELEOP_DISABLE_BEAMS))
    {
        controlData.saHomePosition = true;
    }
    else
    {
        controlData.saHomePosition = (controllerData.sABtn) && !controlData.shift;
    }

    controlData.pastStickRead = controlData.currentStickRead;
    controlData.currentStickRead = controllerData.sLYStick;

    // if (((controlData.saIntakeBackwards != controlData.toggleOutake) && !controlData.saIntakeBackwards) || 
    // ((robotData.controlData.mode == MODE_TELEOP_SA) && (controlData.pastStickRead != controlData.currentStickRead) && (std::abs(controlData.currentStickRead) < 0.5)))
    // {
    //     controlData.saHomePosition = true;
    // }
    // else
    // {
    //     controlData.saHomePosition = (controllerData.sABtn) && !controlData.shift;
    // }

    controlData.toggleOutake = controlData.saIntakeBackwards;
    controlData.saIntakeBackwards = (controllerData.sRBumper) && controlData.shift;
    // MANUAL:
    controlData.mEndEffectorRollersIn = controllerData.sBBtn && !controlData.shift;
    controlData.mEndEffectorRollersOut = controllerData.sYBtn && !controlData.shift;

// ARM:
    // SEMI AUTO:
    
    controlData.saResetOdometry;
    controlData.saForceRunBullBar;
    controlData.saPositionHumanPlayer = (controllerData.sABtn) && controlData.shift;
    controlData.saPositionLow = (controllerData.sXBtn) && !controlData.shift;
    controlData.saPositionMid = (controllerData.sBBtn) && !controlData.shift;
    controlData.saPositionHigh = (controllerData.sYBtn) && !controlData.shift;
    controlData.saSetUpPosition = (controllerData.sBBtn) && controlData.shift;
    controlData.saConeFlipPosition = (controllerData.sYBtn) && controlData.shift;

    controlData.saDoubleSubCone = (controllerData.sRTrigger > 0.5) && controlData.shift;

    // MANUAL:
    controlData.mMovePivot = (controllerData.sRYStick > 0.08 || controllerData.sRYStick < -0.08) && controlData.shift;
    controlData.mMoveWrist = (controllerData.sLYStick > 0.08 || controllerData.sLYStick < -0.08) && controlData.shift;
    controlData.mForceZeroWrist = (controllerData.sLStickBtn) && controlData.shift;
    controlData.mBullBarExtension = (controllerData.sLYStick > 0.08 || robotData.controllerData.sLYStick < -0.08) && !controlData.shift;
    controlData.mBullBarRollerForward = (controllerData.sRTrigger > 0.5) && !controlData.shift;
    controlData.mBullBarRollerBackward = (controllerData.sRTrigger > 0.5) && controlData.shift;
    controlData.mForceZeroBullBar = (controllerData.sRStickBtn) && !controlData.shift;
    controlData.mForceZeroPivot = (controllerData.sRStickBtn) && controlData.shift;

    controlData.saConeCall = (controllerData.sRCenterBtn) && !controlData.shift;
    controlData.saCubeCall = (controllerData.sLCenterBtn) && !controlData.shift;
    controlData.saFastConeCall = (controllerData.sRCenterBtn) && controlData.shift;
    controlData.saFastCubeCall = (controllerData.sLCenterBtn) && controlData.shift;

    // controlData.saDoubleSubCone

    controlData.saChangeAutoAllign = controllerData.sRXStick;
// ELEVATOR:
    //MANUAL:
    controlData.mMoveElevator = (controllerData.sRYStick > 0.08 || controllerData.sRYStick < -0.08) && controlData.shift;
    controlData.mForceZeroElevator = (controllerData.sLStickBtn) && !controlData.shift;

    controlData.saMoveBullBar = (controllerData.sRYStick > 0.08 || controllerData.sRYStick < -0.08) && !controlData.shift;
}



