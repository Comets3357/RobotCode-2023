#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
// #include <unor>
#include <unordered_map>
#include <frc/FileSystem.h>

struct RobotData;



struct ConfigData 
{
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

        double bullBarCubePosition;
        double bullBarConePosition;
        double bullBarFlipConePosition;
        double bullBarMinPosition;
        double bullBarMaxPosition;
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

        double wristHomePosition;
        double wristConeMidPosition;
        double wristConeHighPosition;
        double wristCubeMidPosition;
        double wristCubeHighPosition;
        double wristConeIntakePosition;
        double wristUprightConePosition;
        double wristCubeIntakePosition;
        double wristLowPosition;

        double pivotHomePosition;
        double pivotConeMidPosition;
        double pivotConeHighPosition;
        double pivotCubeMidPosition;
        double pivotCubeHighPosition;
        double pivotConeIntakePosition;
        double pivotUprightConePosition;
        double pivotCubeIntakePosition;
        double pivotLowPosition;
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

        double elevatorHomePosition;
        double elevatorConeMidPosition;
        double elevatorConeHighPosition;
        double elevatorCubeMidPosition;
        double elevatorCubeHighPosition;
        double elevatorSetupPosition;
        double elevatorLowPosition;
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

    BullBarConfigData bullBarConfigData;
    ArmConfigData armConfigData;
    ElevatorConfigData elevatorConfigData;
    EndEffectorConfigData endEffectorConfigData;
    DrivebaseConfigData drivebaseConfigData;
};

class ConfigurationFiles 
{
public:
    void ReadFile(const RobotData &robotData, ConfigData &configData, const std::string &fileName);
private:
    void ParseLine(ConfigData &configData, const std::string &line);

    std::unordered_map<std::string, int> configMap = 
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
        {"PivotRelativeConversion", 96},

        {"ElevatorHomePosition", 100},
        {"ElevatorConeMidPosition", 101},
        {"ElevatorConeHighPosition", 102},
        {"ElevatorCubeMidPosition", 103},
        {"ElevatorCubeHighPosition", 104},
        {"ElevatorSetupPosition", 105},
        {"ElevatorLowPosition", 106},

        {"WristHomePosition" ,110},
        {"WristConeMidPosition", 111},
        {"WristConeHighPosition", 112},
        {"WristCubeMidPosition" ,113},
        {"WristCubeHighPosition", 114},
        {"WristUprightConePosition", 115},
        {"WristCubeIntakePosition", 116},
        {"WristLowPosition", 118},
        {"WristConeIntakePosition", 117},

        {"PivotHomePosition" ,120},
        {"PivotConeMidPosition", 121},
        {"PivotConeHighPosition", 122},
        {"PivotCubeMidPosition" ,123},
        {"PivotCubeHighPosition", 124},
        {"PivotUprightConePosition", 125},
        {"PivotCubeIntakePosition", 126},
        {"PivotConeIntakePosition", 127},
        {"PivotLowPosition", 128},

        {"BullBarCubePosition", 130},
        {"BullBarConePosition", 131},
        {"BullBarFlipConePosition", 132},
        {"BullBarMinPosition", 133},
        {"BullBarMaxPosition", 134}
    
    };

    int failedReadAttempts = 0;


};