// #pragma once


#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycle.h>
#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <math.h>

struct ElevatorData
{
    bool elevatorRunning = false;
    bool elevatorAbsoluteEncoderInitialized = false;
};

enum ElevatorRunMode
{
    ELEVATOR_ABSOLUTE_RUN,
    ELEVATOR_RELATIVE_RUN,
    ELEVATOR_NONE
};

class Elevator
{
public:

    void RobotInit(const RobotData &robotData, ElevatorData &elevatorData);
    void RobotPeriodic(const RobotData &robotData, ElevatorData &elevatorData);
    void DisabledInit();
    void DisabledPeriodic(const RobotData &robotData, ElevatorData elevatorData);
    void UpdateData(const RobotData &robotData, ElevatorData elevatorData);

private:

    ElevatorRunMode runMode = ELEVATOR_ABSOLUTE_RUN;

    //void SetElevatorPosition(double elevatorAbsolutePosition);
    void SemiAuto(const RobotData &robotData, ElevatorData &ElevatorData);
    void Manual(const RobotData &robotData, ElevatorData &ElevatorData);
    void DisableSoftLimits();
    void EnableSoftLimits();
    void ZeroRelativePosition(ElevatorData &elevatorData);
    void ForceZeroElevator();

    bool IsAbsoluteEncoderInitialized(ElevatorData &bullbarData);
    void MoveElevator(double targetDegree, const RobotData& robotData, double timeOffset);

    bool softLimitsEnabled = false;
    bool elevatorForceZeroed = false;

    bool absoluteEncoderFeedBackDevice = true;

    rev::CANSparkMax elevatorMotor{21, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder elevatorRelativeEncoder = elevatorMotor.GetEncoder();
    rev::SparkMaxPIDController elevatorPIDController = elevatorMotor.GetPIDController();
    rev::SparkMaxAbsoluteEncoder elevatorAbsoluteEncoder = elevatorMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    // Encoder Min and Max Values 
    double elevatorMaxPosition = elevatorMinPosition + 75; // TODO: fix this value when we get subsystem
    double elevatorMinPosition = 0; // TODO: fix this value when we get subsystem

    double elevatorUpwardSpeed = 0.4;
    double elevatorDownwardSpeed = -0.4;



    bool elevatorProfileActive = false;
    double elevatorProfileStartPos = 0;
    double elevatorProfileEndPos = 0;
    double elevatorProfileStartTime = 0;    
    double elevatorTimeOffset = 0;
    frc::TrapezoidProfile<units::degree> elevatorProfile
    {
        frc::TrapezoidProfile<units::degrees>::Constraints{0_deg_per_s, 0_deg/(1_s * 1_s)},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{0}, units::angular_velocity::degrees_per_second_t{0}},
        frc::TrapezoidProfile<units::degrees>::State{units::angle::degree_t{0}, units::angular_velocity::degrees_per_second_t{0}}
    };

    //elevator heights todo
    double intakeElevatorAbsolutePos = 0.0;
    double midElevatorAbsolutePos = 0.0;
    double highElevatorAbsolutePos = 0.0;
    double humanPlayerElevatorAbsolutePos = 0.0;

    
};





