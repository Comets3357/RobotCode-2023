#include "controller/Controller.h"
#include "RobotData.h"

// for updating states of control variables (to be accessed by other subsystems)
void Controller::updateControlData(const RobotData &robotData, const ControllerData &controllerData, ControlData &controlData)
{
    // states:
    controlData.shift = controllerData.sLBumper;
    
    switch (controllerData.sDPad) {
        case -1:
            break;
        case 0: // up
            controlData.mode = mode_teleop_manual;
            break;
        case 90:    // right
            controlData.mode = mode_teleop_sa;
            break;
        case 180:   // down
            controlData.mode = mode_climb_manual;
            break;
        case 270:   // left
            controlData.mode = mode_climb_sa;
            break;
        default:
            controlData.mode = mode_teleop_sa;
            break;
            
    }
//-----------------------------------------------------------------------------------------------------------------------------------
//        DRIVEBASE / PRIMARY DRIVER BUTTONS:
//-----------------------------------------------------------------------------------------------------------------------------------

    // note: when pRShoulderSwitch is held, driving is sensitive to turning, while not held (default driving mode) driving is less sensitive to turning and good for quick staright movements and steady arcs (won't turn super easily)
    controlData.turnResponsive = controllerData.pRShoulderSwitch;
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

    controlData.dbInverted = controllerData.pLShoulderSwitch;
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

// INTAKE:
    // SEMI AUTO:

    controlData.saIntaking = (controllerData.sRTrigger > 0.5) && !controlData.shift;
    controlData.saIntakeBackwards = (controllerData.sLTrigger > 0.5) && !controlData.shift;

    // MANUAL:
    
    controlData.mIntakeDown = controllerData.sRBumper && !controlData.shift;
    controlData.mIntakeUp = controllerData.sRBumper && controlData.shift;
    controlData.mIntakeRollersIn = (controllerData.sRTrigger > 0.5) && !controlData.shift;
    controlData.mIntakeRollersOut = (controllerData.sRTrigger > 0.5) && controlData.shift;
    controlData.mForceZeroIntake = controllerData.sRCenterBtn;
    
    

}

