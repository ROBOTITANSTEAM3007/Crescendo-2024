// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <Defines.h>
#include <Shooter.h>
#include <Swerve.h>


//https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  frc::Spark leftClimbMotor{0};
  frc::Spark rightClimbMotor{1};

  frc::Joystick controlStick{1};

  swerveDrive swerve;

  frc::Solenoid rightLock{frc::PneumaticsModuleType::REVPH, 4};
  frc::Solenoid leftLock{frc::PneumaticsModuleType::REVPH, 5};

  frc::Compressor m_compressor{15, frc::PneumaticsModuleType::REVPH};

  shooter shootObj{&controlStick};

  

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
