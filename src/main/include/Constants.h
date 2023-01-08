#pragma once

// drivebase:
static const int
    leftLeadDeviceID = 1,
    leftFollowDeviceID = 2,
    rightLeadDeviceID = 4,
    rightFollowDeviceID = 5;

//intake:
static const int
    intakeRollerID = 12,
    intakePivotID = 11,
    intakeSingulatorID = 13;

static const double
    //practice bot
    absOut = 0.481,
    absIn = 0.616,
    revOut = 6.4,
    revIn = 0;

    //comp bot intake pivot positions
    // absOut = 0.287,
    // absIn = 0.428,
    // revOut = 6.09,
    // revIn = 0;

//indexer
static const int
    indexerBeltsID = 22,
    indexerWheelID = 21;

// shooter IDs
static const int
    shooterWheelID = 31,
    shooterHoodID = 33, 
    shooterTurretID = 34, 
    hoodRollerID = 32; 

static const float
    //comp bot shooter hood positions
    // hoodabsOut = 0.058,
    // hoodabsIn = 0.956,
    // //SPARE BACKUP #1
    // //hoodabsIn 0.931,
    // //hoodabsOut 0.028,
    // hoodrevOut = -38,
    // hoodrevIn = 0,
    // hoodAngleOut = 43,
    // hoodAngleIn = 21,

    //PRATLAS shooter hood positions
    hoodabsOut = 0.103f,
    hoodabsIn = 0.962f,
    hoodrevOut = -38.0f,
    hoodrevIn = 0.0f,
    hoodAngleOut = 43.0f,
    hoodAngleIn = 21.0f,

    // //ATLAS turret positions
    turretZeroDegrees = 0.0f, 
    turretFullRotationDegrees = 427.0f,
    turretMiddleDegrees = 213.77f,
    turretBackwardsDegrees_C = 33.192627f,
    turretBackwardsDegrees_CCW = turretMiddleDegrees + 180.0f,
    turretFullRotationRev_CCW = 139.85f,
    turretFullRotationRev_C = 0.0f,
    turretMiddleRev = turretFullRotationRev_CCW/2,
    turretFullRotationAbs_CCW = 0.830242f,
    turretFullRotationAbs_C = 0.143406f,

    forwardVelOffset = 175.0f,
    // forwardVelOffsetFar = 220,
    forwardHoodOffsetFar = 0.5f,
    forwardHoodOffsetClose = 4.5f,
    change = 7.5f;

    //FOR PRATLAS
    // turretZeroDegrees = 5, //15
    // turretFullRotationDegrees = 535, //540
    // turretMiddleDegrees = 273,
    // turretZeroRev = 89, //87.5
    // turretFullRotationRev_CCW = 174.85, //174.85
    // turretFullRotationRev_C = 0, //0
    // turretZeroAbs = 0.486, //0.479
    // turretFullRotationAbs_CCW = 0.906, //0.91
    // turretFullRotationAbs_C = 0.05; //0.05

//turret gyro offset
static const int
    //at the the corresponding rate of rotation, offset the turret by an extra 20 degrees to account for the extra strength needed (centipetal acceleration?)
    turretGyroOffset2 = -50,
    turretGyroOffset1 = 0;
    
static const int 
    //at certain rotational rate points add the speed offset above
    rotationalRate2 = 400,
    rotationalRate1 = 0;

//Set shooting hood positions and velocity 
static const float
    outerLaunchHood = -35.5f,
    outerLaunchVel = 650.0f,
    innerLaunchHood = -29.0f,
    innerLaunchVel = 1710.0f,
    wallHood = -21.5f,
    wallVel = 1510.0f,
    fenderHood = -10.0f,
    fenderVel = 1375.0f;

// climb
static const int
    climbElevatorID = 41,
    climbArmsID = 43;
    
const float 
    climbArmsZero = 0.8105f;

//limelight:
static const double
    pi = 3.141592653589793238463,
    hubHeight = 104,
    limelightMount = 36.5, //height of mount
    limelightAngle = 36.2, //angle of limelight on mount
    xcameraDistanceFromBot = 4.2,//3.8,//3.35, //3.2, //3.0625, //offset from center of shooter X
    ycameraDistanceFromBot = 9.5, //offset from center of shooter Y
    shooterDistanceFromCenterOfBot = 2, 
    crosshairOffset = 0;

// DIO
static const int
    intakeAbsoluteEncoderPort = 8,
    climbArmsAbsID = 4,
    HoodAbsoluteEncoderPort = 1,
    TurretAbsoluteEncoderPort = 0,
    bottomBeamBreakPort = 3,
    midBeamBreakPort = 9,
    topBeamBreakPort = 5;




