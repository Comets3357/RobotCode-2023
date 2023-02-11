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






    };


};