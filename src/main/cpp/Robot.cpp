// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.AddOption(kAmpAuto, kAmpAuto);
  m_chooser.AddOption(kShootAuto, kShootAuto);
  m_chooser.AddOption(kDriveNinetyAuto, kDriveNinetyAuto);
  m_chooser.SetDefaultOption(kDriveZeroAuto, kDriveZeroAuto);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
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
  swerve.readEncoders();
  swerve.refreshPID();

  frc::SmartDashboard::PutNumber("Shooter Angle", shootObj.angle());
}

void Robot::ampAuto() {
  if(this->autoStepOne) {
    if(shootObj.setAngleRequest(armEncoderAmpPos)) {
      this->autoStepOne = false;
    }
  } else {
    shootObj.setAngleRequest(shootingAngle);
    swerve.driveFor(25); //25 inches
  }
  
}
void Robot::driveNinetyAuto() {
  swerve.driveFor(25);
}
void Robot::driveZeroAuto() {
  swerve.driveFor(12);
}



void Robot::shootAuto() {
  if(this->autoStepOne) {
    if(shootObj.setAngleRequest(shootingAngle)) {
      this->autoStepOne = false;
    }
  } else {
    swerve.driveFor(25); //25 inches
  }
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

  if (m_autoSelected == kAmpAuto || m_autoSelected == kDriveNinetyAuto) {
    swerve.calibGyro(90);
  } else {
    // Default Auto goes here
    swerve.calibGyro(0);
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAmpAuto) {
    ampAuto();
  } else if(m_autoSelected == kShootAuto) {
    shootAuto();
    
  }else if(m_autoSelected == kDriveNinetyAuto) {
    driveNinetyAuto();
  } else {
    driveZeroAuto();
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

        if(driveStick.GetRawButton(2)) {
          rightLock.Set(1);
          rightClimbMotor.Set(0.5);
          leftClimbMotor.Set(-0.5);
          leftLock.Set(1);
        } 
        else {
          leftLock.Set(0);
          rightLock.Set(0);
          leftClimbMotor.Set(0);
          rightClimbMotor.Set(0);
        }
        if(driveStick.GetRawButton(3)) {
          leftClimbMotor.Set(0.5);
          leftLock.Set(0);
        } if(driveStick.GetRawButton(4)) {
          rightClimbMotor.Set(-0.5);
          rightLock.Set(0);
        } 
      if(driveStick.GetRawButton(11)) {
        swerve.calibGyro(0);
      }
      if(driveStick.GetRawButton(12)) {
        swerve.calibGyro(90);
      }
      



      swerve.fieldCentricDrive();

    

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {
  swerve.calibPID();
  
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
