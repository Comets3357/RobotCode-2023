#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <frc/FileSystem.h>

struct RobotData;

struct ConfigData 
{
    double bullBarAbsoluteOffset;
    double bullBarRelativeOffset;
};

class ConfigurationFiles 
{
public:
    void RobotInit(const RobotData &robotData, ConfigData &configData, const std::string &fileName);
private:
    void ParseLine(ConfigData &configData, const std::string &line);

    std::map<std::string, int> configMap = 
    {
        {"BullBarAbsoluteZeroOffset", 0},

    };


};