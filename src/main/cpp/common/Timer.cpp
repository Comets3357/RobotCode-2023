#include "common/Timer.h"

#include <frc/DriverStation.h>

void Timer::RobotInit(TimerData &timerData)
{
    initTimer.Reset();
    initTimer.Start();
    timerData.secSinceInit = 0;
}

void Timer::EnabledInit(TimerData &timerData) {
    timer.Reset();
    timer.Start();
    timerData.secSinceEnabled = 0;
}

void Timer::EnabledPeriodic(TimerData &timerData)
{
    timerData.secSinceEnabled = timer.Get().to<double>();
    timerData.secSinceInit = initTimer.Get().to<double>();
    frc::SmartDashboard::PutNumber("timerData.secSinceEnabled", timerData.secSinceEnabled);
    frc::SmartDashboard::PutNumber("Match Timer", frc::DriverStation::GetMatchTime());
}

void Timer::DisabledInit(TimerData &timerData)
{
    // initTimer.Reset();
    // initTimer.Start();
    // timerData.secSinceInit = 0;
}

void Timer::DisabledPeriodic(TimerData &timerData) {
    timerData.secSinceInit = initTimer.Get().to<double>();
    timerData.secSinceEnabled = timer.Get().to<double>();

    frc::SmartDashboard::PutNumber("TIUNASDIJFKSJDF", timerData.secSinceInit);
}