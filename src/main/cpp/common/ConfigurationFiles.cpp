#include "common/ConfigurationFiles.h"
#include "RobotData.h"

void ConfigurationFiles::ReadFile(const RobotData &robotData, ConfigData &configData, const std::string &fileName)
{
    // frc::SmartDashboard::PutNumber("I AM HERE FOR VOLTAGE COMP", false);
    // frc::SmartDashboard::PutBoolean("Has config been read?", false);

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

        // frc::SmartDashboard::PutBoolean("Has config been read?", true);
    }
    else
    {
        // frc::SmartDashboard::PutBoolean("Has config been read?", false);
    }

    // frc::SmartDashboard::PutNumber("test parsed line", configData.bullBarConfigData.absoluteConversion);
    
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
        value = line.substr(pos + 1);
    }
    else 
    {
        key = line.substr(1);
        value = "";
    }

    // This trims all of the uneeded space for the keys and values
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_last_not_of(" \t") + 1);


    // This is where we check all of our keys to populate the data struct
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
                if (value == "true\r")
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
                if (value == "true\r")
                {
                    configData.bullBarConfigData.invertSliderRelative = true;
                }
                else
                {
                    configData.bullBarConfigData.invertSliderRelative = false;
                }
                break;
            case 8: // BullBarSliderInvertedAbsolute
                if (value == "true\r")
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
                if (value == "true\r")
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
                if (value == "true\r")
                {
                    configData.elevatorConfigData.invertAbosolute = true;
                }
                else
                {
                    configData.elevatorConfigData.invertAbosolute = false;
                }
                break;
            case 31: // ElevatorInvertedRelative
                if (value == "true\r")
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
// -------------------------------------------------------------------------------------------------------------------------------------
//                      DRIVEBASE CONFIG DATA
// -------------------------------------------------------------------------------------------------------------------------------------
            case 50: // DrivebaseRightInverted
                if (value == "true\r")
                {
                    configData.drivebaseConfigData.rightInverted = true;
                }
                else
                {
                    configData.drivebaseConfigData.rightInverted = false;
                }
                break;
            case 51: // DrivebaseLeftInverted
                if (value == "true\r")
                {
                    configData.drivebaseConfigData.leftInverted = true;
                }
                else
                {
                    configData.drivebaseConfigData.leftInverted = false;
                }
                break;
            case 52: // DriveBaseCurrentLimit
                configData.drivebaseConfigData.currentLimit = std::stod(value);
                break;
            case 53: // DrivebaseLeftPValue
                configData.drivebaseConfigData.leftP = std::stod(value);
                break;
            case 54: // DrivebaseLeftFFValue
                configData.drivebaseConfigData.leftFF = std::stod(value);
                break;
            case 55:// DrivebaseRightPValue
                configData.drivebaseConfigData.rightP = std::stod(value);
                break;
            case 56: // DrivebaseRightFFValue
                configData.drivebaseConfigData.rightFF = std::stod(value);
                break;
// -------------------------------------------------------------------------------------------------------------------------------------
//                      ARM CONFIG DATA
// -------------------------------------------------------------------------------------------------------------------------------------
            case 70: // WristPValue
                configData.armConfigData.wristP = std::stod(value);
                break;
            case 71: // WristCurrentLimit
                configData.armConfigData.wristCurrentLimit = std::stod(value);
                break;
            case 72: // WristRelativeInverted
                if (value == "true\r")
                {
                    configData.armConfigData.wristRelativeInverted = true;
                }
                else 
                {
                    configData.armConfigData.wristRelativeInverted = false;
                }
                break;
            case 73: // WristAbsoluteInverted
                if (value == "true\r")
                {
                    configData.armConfigData.wristAbsoluteInverted = true;
                }
                else
                {
                    configData.armConfigData.wristAbsoluteInverted = false;
                }
                break;
            case 74: // WristAbsoluteConversion
                configData.armConfigData.wristAbsoluteConversion = std::stod(value);
                break;
            case 75: // WristAbsoluteZeroOffset
                configData.armConfigData.wristAbsoluteOffset = std::stod(value);
                break;
            case 76: // WristRelativeConversion
                configData.armConfigData.wristRelativeConversion = std::stod(value);
                break;

            case 90: // PivotPValue
                configData.armConfigData.pivotP = std::stod(value);
                break;
            case 91: // PivotCurrentLimit
                configData.armConfigData.pivotCurrentLimit = std::stod(value);
                break;
            case 92: // PivotRelativeInverted
                if (value == "true\r")
                {
                    configData.armConfigData.pivotRelativeInverted = true;
                }
                else
                {
                    configData.armConfigData.pivotRelativeInverted = false;
                }   
                break;
            case 93: // PivotAbsoluteInverted
                if (value == "true\r")
                {
                    configData.armConfigData.pivotAbsoluteInverted = true;
                }
                else 
                {
                    configData.armConfigData.pivotAbsoluteInverted = false;
                }
                break;
            case 94: // PivotAbsoluteConversion
                configData.armConfigData.pivotAbsoluteConversion = std::stod(value);
                break;
            case 95: // PivotAbsoluteZeroOffset
                configData.armConfigData.pivotAbsoluteOffset = std::stod(value);
                break;
            case 96: // PivotRelativeConversion
                configData.armConfigData.pivotRelativeConversion = std::stod(value);
                break;
            default:
                break;
        }
    }
    catch(...)
    {

    }
}