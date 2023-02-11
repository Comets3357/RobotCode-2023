#include "common/ConfigurationFiles.h"
#include "RobotData.h"

void ConfigurationFiles::RobotInit(const RobotData &robotData, ConfigData &configData, const std::string &fileName)
{
    frc::SmartDashboard::PutBoolean("Has config been read?", false);

    // Accessing the deploy dir
    fs::path deployDir = frc::filesystem::GetDeployDirectory();

    // Path to the config file
    fs::path robotConfigFile = deployDir / "ConfigFiles" / fileName;

    // Accessing the config file
    std::ifstream inFile;
    inFile.open(robotConfigFile.string());

    // Goes through each line of the config file
    if(inFile.is_open()) 
    {
        std::string line; 

        while (getline(inFile, line))
        {
            ParseLine(configData, line);
        }

        inFile.close();

        frc::SmartDashboard::PutBoolean("Has config been read?", true);
    }
    else
    {
        frc::SmartDashboard::PutBoolean("Has config been read?", false);
    }

    
}

/*
* @note Parses through each line to get the key and values
*/
void ConfigurationFiles::ParseLine(ConfigData &configData, const std::string &line)
{
    // If we have empty lines, return nothing
    if (line.empty() || line[0] == '#')
    {
        return;
    }

    // Creating the key and value strings
    std::string key;
    std::string value;

    // Splitting up the line into two strings
    std::size_t pos = line.find('=');

    // Grabs the key and value pairs
    if (pos != std::string::npos)
    {
        key = line.substr(0, pos);
        value = line.substr(1, pos + 1);
    }
    else 
    {
        key = line;
        value = "";
    }

    // This trims all of the uneeded space in the keys
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_first_of(" \t") + 1);

    // This trims all of the uneeded space in the values
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_first_of(" \t") + 1);

    // This is where we check all of our keys
    try
    {
        switch (configMap[key])
        {
// -------------------------------------------------------------------------------------------------------------------------------------
//                      BULL BAR CONFIG DATA
// -------------------------------------------------------------------------------------------------------------------------------------
            case 0: // VoltageCompensation
                configData.bullBarConfigData.voltageComp = std::stod(value);
                configData.armConfigData.voltageComp = std::stod(value);
                configData.endEffectorConfigData.voltageComp = std::stod(value);
                configData.elevatorConfigData.voltageComp = std::stod(value);
                configData.drivebaseConfigData.voltageComp = std::stod(value);
                break;
            case 1: // BullBarRollerInverted
                if (value == "true")
                {
                    configData.bullBarConfigData.invertRollers = true;
                }
                else
                {
                    configData.bullBarConfigData.invertRollers = false;
                }
                break;
            case 2: // BullBarAbsoluteConversionFactor
                configData.bullBarConfigData.absoluteConversion = std::stod(value);
                break;
            case 3: // BullBarAbsoluteZeroOffset
                configData.bullBarConfigData.absoluteOffset = std::stod(value);
                break;
            case 4: // BullBarRelativeConversionFactor
                configData.bullBarConfigData.relativeConversion = std::stod(value);
                break;
            case 5: // BullBarPValue
                configData.bullBarConfigData.pValue = std::stod(value);
                break;
            case 6: // BullBarSmartCurrentLimit
                configData.bullBarConfigData.currentLimit = std::stod(value);
                break;
            case 7: // BullBarSliderInvertedRelative
                if (value == "true")
                {
                    configData.bullBarConfigData.invertSliderRelative = true;
                }
                else
                {
                    configData.bullBarConfigData.invertSliderRelative = false;
                }
                break;
            case 8: // BullBarSliderInvertedAbsolute
                if (value == "true")
                {
                    configData.bullBarConfigData.invertSliderAbsolute = true;
                }
                else
                {
                    configData.bullBarConfigData.invertSliderAbsolute = false;
                }
                break;
// -------------------------------------------------------------------------------------------------------------------------------------
//                      END EFFECTOR CONFIG DATA
// -------------------------------------------------------------------------------------------------------------------------------------
            case 20: // EndEffectorInvertRollers
                if (value == "true")
                {
                    configData.endEffectorConfigData.invertRollers = true;
                }
                else
                {
                    configData.endEffectorConfigData.invertRollers = false;
                }
                break;
            case 21: // EndEffectorCurrentLimit
                configData.endEffectorConfigData.currentLimit = std::stod(value);
                break;
// -------------------------------------------------------------------------------------------------------------------------------------
//                      ELEVATOR CONFIG DATA
// -------------------------------------------------------------------------------------------------------------------------------------
            case 30: // ElevatorInvertedAbsolute
                if (value == "true")
                {
                    configData.elevatorConfigData.invertAbosolute = true;
                }
                else
                {
                    configData.elevatorConfigData.invertAbosolute = false;
                }
                break;
            case 31: // ElevatorInvertedRelative
                if (value == "true")
                {
                    configData.elevatorConfigData.invertRelative = true;
                }
                else
                {
                    configData.elevatorConfigData.invertRelative = false;
                }
                break;
            case 32: // ElevatorCurrentLimit
                configData.elevatorConfigData.currentLimit = std::stod(value);
                break;
            case 33: // ElevatorPValue
                configData.elevatorConfigData.pValue = std::stod(value);
                break;
            case 34: // ElevatorRelativeConversionFactor
                configData.elevatorConfigData.relativeConversionFactor = std::stod(value);
                break;
            case 35: // ElevatorAbsoluteConversionFactor
                configData.elevatorConfigData.absoluteConversionFactor = std::stod(value);
                break;
            case 36:
                configData.elevatorConfigData.absoluteZeroOffset = std::stod(value);
                break;
        }
    }
    catch(...)
    {

    }

}