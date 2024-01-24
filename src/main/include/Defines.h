#pragma once
#include <frc/controller/PIDController.h>
#include <string>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <iostream>
#include <cmath>
#include <units/angle.h>
#include <math.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <wpimath/MathShared.h> 

#define encoderTolerance 0.01

#define frontRightOffset 107.8417
#define frontLeftOffset 114.4335
#define backRightOffset -110.7422
#define backLeftOffset 173.6718