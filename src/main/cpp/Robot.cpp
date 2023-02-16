// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


  driveBase.RobotInit();
  gyro.RobotInit();
  timer.RobotInit(robotData.timerData);
  auton.RobotInit(robotData.controlData, robotData.autonData);




}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

  gyro.RobotPeriodic(robotData.gyroData);
  timer.EnabledPeriodic(robotData.timerData);
  driveBase.RobotPeriodic(robotData, robotData.drivebaseData, robotData.autonData);
  controller.TeleopPeriodic(robotData, robotData.controllerData, robotData.controlData);
  limelight.RobotPeriodic(robotData, robotData.limelightData);
  
  frc::SmartDashboard::PutNumber("i am also here", 1000);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);


  gyro.AutonomousInit(robotData.gyroData);
  auton.AutonomousInit(robotData.autonData);
  driveBase.AutonomousInit(robotData, robotData.drivebaseData, robotData.autonData);
  timer.RobotInit(robotData.timerData);
  timer.EnabledInit(robotData.timerData);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  timer.EnabledPeriodic(robotData.timerData);
  gyro.RobotPeriodic(robotData.gyroData);
  auton.AutonomousPeriodic(robotData, robotData.autonData, robotData.controlData, robotData.controllerData);
  driveBase.RobotPeriodic(robotData, robotData.drivebaseData, robotData.autonData);
  
}

void Robot::TeleopInit() {
  gyro.TeleopInit(robotData.gyroData);

}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  driveBase.DisabledInit();
}

void Robot::DisabledPeriodic() {
  driveBase.DisabledPeriodic();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
