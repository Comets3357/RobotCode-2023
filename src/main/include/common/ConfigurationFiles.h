#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <frc/FileSystem.h>
#include <rev/CANSparkMax.h>

struct RobotData;

struct BullBarConfigData
{
    double voltageComp;

    double absoluteConversion;
    double absoluteOffset;
    double relativeConversion;

    double pValue;
    double currentLimit;

    bool invertRollers;
    bool invertSliderRelative;
    bool invertSliderAbsolute;
};

struct ArmConfigData
{
    double voltageComp;

    double wristP;
    double wristCurrentLimit;
    double wristAbsoluteConversion;
    double wristAbsoluteOffset;
    double wristRelativeConversion;

    double pivotP;
    double pivotCurrentLimit;
    double pivotAbsoluteConversion;
    double pivotAbsoluteOffset;
    double pivotRelativeConversion;

    bool wristRelativeInverted;
    bool wristAbsoluteInverted;

    bool pivotRelativeInverted;
    bool pivotAbsoluteInverted;
};

struct ElevatorConfigData
{
    double voltageComp;
    double pValue;
    double currentLimit;
    double relativeConversionFactor;
    double absoluteConversionFactor;
    double absoluteZeroOffset;

    bool invertAbosolute;
    bool invertRelative;
};

struct EndEffectorConfigData
{
    double voltageComp;
    double currentLimit;

    bool invertRollers;
};

struct DrivebaseConfigData
{
    double voltageComp;
    double currentLimit;
    double leftP;
    double leftFF;
    double rightP;
    double rightFF;

    bool rightInverted;
    bool leftInverted;
};

struct ConfigData 
{
    BullBarConfigData bullBarConfigData;
    ArmConfigData armConfigData;
    ElevatorConfigData elevatorConfigData;
    EndEffectorConfigData endEffectorConfigData;
    DrivebaseConfigData drivebaseConfigData;
};

class ConfigurationFiles 
{
public:
    void RobotInit(const RobotData &robotData, ConfigData &configData, const std::string &fileName);
private:
    void ParseLine(ConfigData &configData, const std::string &line);

    std::map<std::string, int> configMap = 
    {
        {"VoltageCompensation", 0},

        {"BullBarRollerInverted", 1},
        {"BullBarAbsoluteConversionFactor", 2},
        {"BullBarAbsoluteZeroOffset", 3},
        {"BullBarRelativeConversionFactor", 4},
        {"BullBarPValue", 5},
        {"BullBarSmartCurrentLimit", 6},
        {"BullBarSliderInvertedRelative", 7},
        {"BullBarSliderInvertedAbsolute", 8},

        {"EndEffectorInvertRollers", 20},
        {"EndEffectorCurrentLimit", 21},

        {"ElevatorInvertedAbsolute", 30},
        {"ElevatorInvertedRelative", 31},
        {"ElevatorCurrentLimit", 32},
        {"ElevatorPValue", 33},
        {"ElevatorRelativeConversionFactor", 34},
        {"ElevatorAbsoluteConversionFactor", 35},
        {"ElevatorAbsoluteZeroOffset", 36},

        {"DrivebaseRightInverted", 50}, 
        {"DrivebaseLeftInverted", 51},
        {"DriveBaseCurrentLimit", 52},
        {"DrivebaseLeftPValue", 53},
        {"DrivebaseLeftFFValue", 54},
        {"DrivebaseRightPValue", 55},
        {"DrivebaseRightFFValue", 56},

        {"WristPValue", 70},
        {"WristCurrentLimit", 71},
        {"WristRelativeInverted", 72},
        {"WristAbsoluteInverted", 73},
        {"WristAbsoluteConversion", 74},
        {"WristAbsoluteZeroOffset", 75},
        {"WristRelativeConversion", 76},

        {"PivotPValue", 90},
        {"PivotCurrentLimit", 91},
        {"PivotRelativeInverted", 92},
        {"PivotAbsoluteInverted", 93},
        {"PivotAbsoluteConversion", 94},
        {"PivotAbsoluteZeroOffset", 95},
        {"PivotRelativeConversion", 96}
    };


};