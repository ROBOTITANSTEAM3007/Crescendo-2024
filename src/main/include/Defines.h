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
#include <thread>

#include <frc/ADIS16470_IMU.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <wpimath/MathShared.h> 
#include <frc/PneumaticHub.h>
#include <frc/Solenoid.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Compressor.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/DigitalInput.h>
#include <frc/filter/SlewRateLimiter.h>
#include <rev/CANSparkMax.h>
#include <frc/MathUtil.h>

#define encoderTolerance 2.5

#define frontRightOffset 24.9609375
#define frontLeftOffset 153.193359375
#define backRightOffset 17.31445
#define backLeftOffset 158.7304875

#define frontLeftAngleToCenter 51.232442405
#define frontRightAngleToCenter 308.767557595
#define backLeftAngleToCenter 128.767557595
#define backRightAngleToCenter 231.232442405

#define armEncoderChannel 10
#define armEncoderLowerBound 0.0615
#define armEncoderUpperBound 0.2009

#define armEncoderTolerance 0.05