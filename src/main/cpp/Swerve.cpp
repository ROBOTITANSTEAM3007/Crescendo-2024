#include "Swerve.h"




swerveDrive::swerveDrive(frc::Joystick *joy){

    m_motorChoice.SetDefaultOption(k_frontLeftChoice, k_frontLeftChoice);
    m_motorChoice.AddOption(k_frontRightChoice, k_frontRightChoice);
    m_motorChoice.AddOption(k_backLeftChoice, k_backLeftChoice);
    m_motorChoice.AddOption(k_backRightChoice, k_backRightChoice);

    frc::SmartDashboard::PutData("Motor Choice", &m_motorChoice);

    frontRight.m_wheelName = "Front Right";
    frontLeft.m_wheelName = "Front Left";
    backRight.m_wheelName = "Back Right";
    backLeft.m_wheelName = "Back Left";
    frontRight.m_angleOffset = frontRightOffset;
    frontLeft.m_angleOffset = frontLeftOffset;
    backRight.m_angleOffset = backRightOffset;
    backLeft.m_angleOffset = backLeftOffset;

    driveStick = joy;





    frc::SmartDashboard::PutNumber("P", m_p);
    frc::SmartDashboard::PutNumber("I", m_i);
    frc::SmartDashboard::PutNumber("D", m_d);

    frc::SmartDashboard::PutNumber("Calib Angle", m_calibAngle);


}

void swerveDrive::readEncoders() {
    frc::SmartDashboard::PutNumber("FrontLeftRot", frontLeft.encoder->GetAbsolutePosition().GetValueAsDouble()*360 - frontLeftOffset);
    frc::SmartDashboard::PutNumber("FrontRightRot", frontRight.encoder->GetAbsolutePosition().GetValueAsDouble()*360 - frontRightOffset);
    frc::SmartDashboard::PutNumber("backLeftRot", backLeft.encoder->GetAbsolutePosition().GetValueAsDouble()*360 - backLeftOffset);
    frc::SmartDashboard::PutNumber("backRightRot", backRight.encoder->GetAbsolutePosition().GetValueAsDouble()*360 - backRightOffset);


    frc::SmartDashboard::PutBoolean("Front Left Motor Polarity", frontLeft.m_polarity);
    frc::SmartDashboard::PutBoolean("Front Right Motor Polarity", frontRight.m_polarity);
    frc::SmartDashboard::PutBoolean("Back Left Motor Polarity", backLeft.m_polarity);
    frc::SmartDashboard::PutBoolean("Back Right Motor Polarity", backRight.m_polarity);



};

void swerveDrive::calibPID(){

    m_calibAngle = frc::SmartDashboard::GetNumber("Calib Angle", m_calibAngle);




    m_selectedMotor = m_motorChoice.GetSelected();

    if(m_selectedMotor == k_frontLeftChoice){
        m_frontLeftPOffset = frc::SmartDashboard::GetNumber("Front Left P Offset", m_frontLeftPOffset);
        frontLeft.setRotation(m_calibAngle + m_frontLeftPOffset);
        frontLeft.setDrive(0.2);
        frontRight.chill();
        backLeft.chill();
        backRight.chill();
        
    } else if(m_selectedMotor == k_frontRightChoice) {
        m_frontRightPOffset = frc::SmartDashboard::GetNumber("Front Right P Offset", m_frontRightPOffset);

        frontRight.setRotation(m_calibAngle);
        frontRight.setDrive(0.2);
        frontLeft.chill();
        backLeft.chill();
        backRight.chill();

    } else if(m_selectedMotor == k_backLeftChoice) {
        m_backLeftPOffset = frc::SmartDashboard::GetNumber("Back Left P Offset", m_backLeftPOffset);

        backLeft.setRotation(m_calibAngle);
        backLeft.setDrive(0.2);

        frontLeft.chill();
        frontRight.chill();
        backRight.chill();

    } else if(m_selectedMotor == k_backRightChoice) {
        m_backRightPOffset = frc::SmartDashboard::GetNumber("Back Right P Offset", m_backRightPOffset);
        backRight.setRotation(m_calibAngle);
        backRight.setDrive(0.2);
        frontLeft.chill();
        backLeft.chill();
        frontRight.chill();

    } else {

    }
};


void swerveWheel::refreshPID(double P, double I, double D){
    this->m_p = P;
    this->m_i = I;
    this->m_d = D;
    
    m_directionController.Reset();
    m_directionController.SetPID(m_p, m_i, m_d);
}


void swerveDrive::refreshPID(){
    m_p = frc::SmartDashboard::GetNumber("P", m_p);
    m_i = frc::SmartDashboard::GetNumber("I", m_i);
    m_d = frc::SmartDashboard::GetNumber("D", 0);


    frontLeft.refreshPID(m_p + m_frontLeftPOffset, m_i, m_d);
    frontRight.refreshPID(m_p + m_frontRightPOffset, m_i, m_d);
    backLeft.refreshPID(m_p + m_backLeftPOffset, m_i, m_d);
    backRight.refreshPID(m_p + m_backRightPOffset, m_i, m_d);
}

//Basic drive system pseudocode

//Vectors have a magnitude and direction.  Magnitude is motor speed and angle is direction.
//for a non field oriented drive we can just take the y and x of the joystick, find the angle with some trig and set the wheels to that.
/*
rotation function: atan*/

void swerveDrive::fieldCentricDrive() {

    m_stickY = -driveStick->GetY();
    m_stickX = -driveStick->GetX();
    m_stickTwist = -driveStick->GetTwist();

    
	if(fabs(m_stickX) > 0.1 || fabs(m_stickY) > 0.1 || fabs(m_twistAngle) > 0.1) {
		//Control for an undefined slope
        m_angleD = (double)imu.GetAngle(imu.kYaw);
		

		frontLeft.F = m_stickTwist * cos((frontLeftAngleToCenter+90)/RtoD) + m_stickY;
        frontLeft.S = m_stickTwist * sin((frontLeftAngleToCenter + 90)/RtoD) + m_stickX;
        frontLeft.m_magnitude = std::sqrt(std::pow(frontLeft.F, 2) + std::pow(frontLeft.S, 2));
        if(frontLeft.F != 0) {
            
            frontLeft.m_angle = (atan(frontLeft.S/frontLeft.F) * 57.2958) - m_angleD;

		} else if(frontLeft.F > 0) {
			frontLeft.m_angle = 90 - m_angleD;
		} else {
			frontLeft.m_angle = -90 - m_angleD;
		}
		if(m_stickY < 0) {
    		frontLeft.m_angle = 180 + frontLeft.m_angle;
		} else if(m_stickX < 0){
		    frontLeft.m_angle = 360 + frontLeft.m_angle;
		}

        frontRight.F = m_stickTwist * cos((frontRightAngleToCenter+90)/RtoD) + m_stickY;
        frontRight.S = m_stickTwist * sin((frontRightAngleToCenter + 90)/RtoD) + m_stickX;
        frontRight.m_magnitude = std::sqrt(std::pow(frontRight.F, 2) + std::pow(frontRight.S, 2));
        if(frontRight.F != 0) {
            
            frontRight.m_angle = (atan(frontRight.S/frontRight.F) * 57.2958) - m_angleD;

		} else if(frontRight.F > 0) {
			frontRight.m_angle = 90 - m_angleD;
		} else {
			frontRight.m_angle = -90 - m_angleD;
		}
		if(m_stickY < 0) {
    		frontRight.m_angle = 180 + frontRight.m_angle;
		} else if(m_stickX < 0){
		    frontRight.m_angle = 360 + frontRight.m_angle;
		}

        backLeft.F = m_stickTwist * cos((backLeftAngleToCenter+90)/RtoD) + m_stickY;
        backLeft.S = m_stickTwist * sin((backLeftAngleToCenter + 90)/RtoD) + m_stickX;
        backLeft.m_magnitude = std::sqrt(std::pow(backLeft.F, 2) + std::pow(backLeft.S, 2));
        if(backLeft.F != 0) {
            
            backLeft.m_angle = (atan(backLeft.S/backLeft.F) * 57.2958) - m_angleD;

		} else if(backLeft.F > 0) {
			backLeft.m_angle = 90 - m_angleD;
		} else {
			backLeft.m_angle = -90 - m_angleD;
		}
		if(m_stickY < 0) {
    		backLeft.m_angle = 180 + backLeft.m_angle;
		} else if(m_stickX < 0){
		    backLeft.m_angle = 360 + backLeft.m_angle;
		}

        backRight.F = m_stickTwist * cos((backRightAngleToCenter+90)/RtoD) + m_stickY;
        backRight.S = m_stickTwist * sin((backRightAngleToCenter + 90)/RtoD) + m_stickX;
        backRight.m_magnitude = std::sqrt(std::pow(backRight.F, 2) + std::pow(backRight.S, 2));
        if(backRight.F != 0) {
            
            backRight.m_angle = (atan(backRight.S/backRight.F) * 57.2958) - m_angleD;

		} else if(backRight.F > 0) {
			backRight.m_angle = 90 - m_angleD;
		} else {
			backRight.m_angle = -90 - m_angleD;
		}
		if(m_stickY < 0) {
    		backRight.m_angle = 180 + backRight.m_angle;
		} else if(m_stickX < 0){
		    backRight.m_angle = 360 + backRight.m_angle;
		}

        magMax=fmax(fabs(frontLeft.m_magnitude),fmax(fabs(frontRight.m_magnitude), fmax(fabs(backLeft.m_magnitude),fabs(backRight.m_magnitude))));
        if(magMax > 1) {
            frontLeft.m_magnitude /= magMax;
            frontRight.m_magnitude /= magMax;
            backLeft.m_magnitude /= magMax;
            backRight.m_magnitude /= magMax;
        }

        frontLeft.setRotation(frontLeft.m_angle);
        frontLeft.setDrive(-frontLeft.m_magnitude);

        frontRight.setRotation(frontRight.m_angle);
        frontRight.setDrive(frontRight.m_magnitude);

        backLeft.setRotation(backLeft.m_angle);
        backLeft.setDrive(-backLeft.m_magnitude);

        backRight.setRotation(backRight.m_angle);
        backRight.setDrive(-backRight.m_magnitude);



        


    } else if(fabs(m_stickTwist) >= 0.6) {
        inPlaceTurn();
    } else {
        frontLeft.chill();
        frontRight.chill();
        backLeft.chill();
        backRight.chill();

    }
    

}

void swerveDrive::calibGyro(double setAngle) {
    imu.SetGyroAngle(imu.kYaw, (units::degree_t)setAngle);
}

void swerveDrive::inPlaceTurn() {
    m_magnitude = m_stickTwist * 0.3;

    frontRight.setRotation(225);
    frontLeft.setRotation(-45);
    backRight.setRotation(135);
    backLeft.setRotation(45);

    frontRight.setDrive(m_magnitude);
    frontLeft.setDrive(-m_magnitude);
    backRight.setDrive(-m_magnitude);
    backLeft.setDrive(-m_magnitude);
}

double swerveDrive::turnClosest(double a, double b) {
    double targetAngle = fmod(a, 360) - fmod(b, 360);
    if(targetAngle > 180) {
        targetAngle -= 360;
    } else if(targetAngle < 180) {
        targetAngle += 360;
    }

    return(targetAngle);
}



double swerveWheel::getEncoder() {
    return(this->rel->GetPosition());
}

void swerveDrive::driveFor(double distance) {
    if(this->rotations == 0 && this->lastEncPos == 0) {
        this->rotations = (distance/wheelCircumfrence) * driveGearRatioMultiplier;
    }
    frontLeft.setRotation(0);
    frontRight.setRotation(0);
    backLeft.setRotation(0);
    backRight.setRotation(0);
    frc::SmartDashboard::PutNumber("Rotations", rotations);

    if(this->rotations > 0.2){
        double difference = fabs(fabs(frontLeft.getEncoder()) - fabs(this->lastEncPos));
        this->rotations -= difference;
        frontLeft.setDrive(-0.25);
        frontRight.setDrive(0.25);
        backLeft.setDrive(-0.25);
        backRight.setDrive(-0.25);
        this->lastEncPos = frontLeft.getEncoder();
    } else {
        frontLeft.setDrive(0);
        frontRight.setDrive(0);
        backLeft.setDrive(0);
        backRight.setDrive(0);
    }

    
}

void swerveDrive::robotRelativeDrive(){

    
    m_stickY = -driveStick->GetY();
    m_stickX = -driveStick->GetX();
    m_stickTwist = -driveStick->GetTwist();

    
	if(fabs(m_stickX) > 0.1 || fabs(m_stickY) > 0.1 || fabs(m_twistAngle) > 0.1) {
		//Control for an undefined slope
        m_angleD = 0;
		

		frontLeft.F = m_stickTwist * cos((frontLeftAngleToCenter+90)/RtoD) + m_stickY;
        frontLeft.S = m_stickTwist * sin((frontLeftAngleToCenter + 90)/RtoD) + m_stickX;
        frontLeft.m_magnitude = std::sqrt(std::pow(frontLeft.F, 2) + std::pow(frontLeft.S, 2));
        if(frontLeft.F != 0) {
            
            frontLeft.m_angle = (atan(frontLeft.S/frontLeft.F) * 57.2958) - m_angleD;

		} else if(frontLeft.F > 0) {
			frontLeft.m_angle = 90 - m_angleD;
		} else {
			frontLeft.m_angle = -90 - m_angleD;
		}
		if(m_stickY < 0) {
    		frontLeft.m_angle = 180 + frontLeft.m_angle;
		} else if(m_stickX < 0){
		    frontLeft.m_angle = 360 + frontLeft.m_angle;
		}

        frontRight.F = m_stickTwist * cos((frontRightAngleToCenter+90)/RtoD) + m_stickY;
        frontRight.S = m_stickTwist * sin((frontRightAngleToCenter + 90)/RtoD) + m_stickX;
        frontRight.m_magnitude = std::sqrt(std::pow(frontRight.F, 2) + std::pow(frontRight.S, 2));
        if(frontRight.F != 0) {
            
            frontRight.m_angle = (atan(frontRight.S/frontRight.F) * 57.2958) - m_angleD;

		} else if(frontRight.F > 0) {
			frontRight.m_angle = 90 - m_angleD;
		} else {
			frontRight.m_angle = -90 - m_angleD;
		}
		if(m_stickY < 0) {
    		frontRight.m_angle = 180 + frontRight.m_angle;
		} else if(m_stickX < 0){
		    frontRight.m_angle = 360 + frontRight.m_angle;
		}

        backLeft.F = m_stickTwist * cos((backLeftAngleToCenter+90)/RtoD) + m_stickY;
        backLeft.S = m_stickTwist * sin((backLeftAngleToCenter + 90)/RtoD) + m_stickX;
        backLeft.m_magnitude = std::sqrt(std::pow(backLeft.F, 2) + std::pow(backLeft.S, 2));
        if(backLeft.F != 0) {
            
            backLeft.m_angle = (atan(backLeft.S/backLeft.F) * 57.2958) - m_angleD;

		} else if(backLeft.F > 0) {
			backLeft.m_angle = 90 - m_angleD;
		} else {
			backLeft.m_angle = -90 - m_angleD;
		}
		if(m_stickY < 0) {
    		backLeft.m_angle = 180 + backLeft.m_angle;
		} else if(m_stickX < 0){
		    backLeft.m_angle = 360 + backLeft.m_angle;
		}

        backRight.F = m_stickTwist * cos((backRightAngleToCenter+90)/RtoD) + m_stickY;
        backRight.S = m_stickTwist * sin((backRightAngleToCenter + 90)/RtoD) + m_stickX;
        backRight.m_magnitude = std::sqrt(std::pow(backRight.F, 2) + std::pow(backRight.S, 2));
        if(backRight.F != 0) {
            
            backRight.m_angle = (atan(backRight.S/backRight.F) * 57.2958) - m_angleD;

		} else if(backRight.F > 0) {
			backRight.m_angle = 90 - m_angleD;
		} else {
			backRight.m_angle = -90 - m_angleD;
		}
		if(m_stickY < 0) {
    		backRight.m_angle = 180 + backRight.m_angle;
		} else if(m_stickX < 0){
		    backRight.m_angle = 360 + backRight.m_angle;
		}

        magMax=fmax(fabs(frontLeft.m_magnitude),fmax(fabs(frontRight.m_magnitude), fmax(fabs(backLeft.m_magnitude),fabs(backRight.m_magnitude))));
        if(magMax > 1) {
            frontLeft.m_magnitude /= magMax;
            frontRight.m_magnitude /= magMax;
            backLeft.m_magnitude /= magMax;
            backRight.m_magnitude /= magMax;
        }

        frontLeft.setRotation(frontLeft.m_angle);
        frontLeft.setDrive(-frontLeft.m_magnitude);

        frontRight.setRotation(frontRight.m_angle);
        frontRight.setDrive(frontRight.m_magnitude);

        backLeft.setRotation(backLeft.m_angle);
        backLeft.setDrive(-backLeft.m_magnitude);

        backRight.setRotation(backRight.m_angle);
        backRight.setDrive(-backRight.m_magnitude);

    } else if(fabs(m_stickTwist) >= 0.6) {
        inPlaceTurn();
    } else {
        frontLeft.chill();
        frontRight.chill();
        backLeft.chill();
        backRight.chill();

    }
    

}



void swerveWheel::chill() {
    this->m_driveMotor->Set(0);
    this->m_rotationMotor->Set(0);
}

void swerveDrive::findZeros() {

}


void swerveWheel::setDrive(double driveMag){

    frc::SmartDashboard::PutBoolean("Polarity", m_polarity);
    if(m_polarity) {
        m_driveMotor->Set(speedLimiter.Calculate(std::clamp(-driveMag, -1.0, 1.0)));
    } else {
        m_driveMotor->Set(speedLimiter.Calculate(std::clamp(driveMag, -1.0, 1.0)) );
    }
}


double swerveWheel::closestAngle(double a, double b){


    //Get Direction
    double m_bestTargetAngle = std::fmod(a, 360) - std::fmod(b, 360);

	

    if (fabs(m_bestTargetAngle) > 180) {
        
		if(m_bestTargetAngle > 0) {
        	m_bestTargetAngle = -(360) + m_bestTargetAngle;
		} else {
			m_bestTargetAngle = 360 + m_bestTargetAngle;
		}
    } 
    
    
    
    if(fabs(m_bestTargetAngle) > 90) {
        //std::cout << "Inverting angle\n";
        m_polarity = 0;
        if(m_bestTargetAngle > 0) {
            m_bestTargetAngle -= 180;
        } else {
            m_bestTargetAngle += 180;
        }
     }else {
		//std::cout << "keeping original angle";
        m_polarity = 1;
    }

    return(m_bestTargetAngle);
}


void swerveWheel::setRotation(double targetAngle) {
    currentAngle = this->encoder->GetAbsolutePosition().GetValueAsDouble() * 360; //Appears that this should be in degrees

    targetAngle += m_angleOffset;


    //account for offset pusing for a degree over/under 360
    if (fabs(targetAngle) > 360) {
        targetAngle -= 360;
    }

    //if(fabs(currentAngle - targetAngle) > encoderTolerance) {

        
        //Get Closest angle to setPoint
        m_setPointAngle = closestAngle(currentAngle, targetAngle);
        //Get Closest angle to setPoint + 180
        //m_setPointAngleFlipped = closestAngle(currentAngle, targetAngle + 180);

        //if(m_setPointAngle < m_setPointAngleFlipped){
            
            m_rotationMotor->Set(std::clamp(m_directionController.Calculate(currentAngle, currentAngle + m_setPointAngle),-0.5, 0.5));
            frc::SmartDashboard::PutNumber("Target Angle For Wheel: " + m_wheelName, m_setPointAngle + currentAngle);
        /*} else {
            m_rotationMotor->Set(std::clamp(m_directionController.Calculate(currentAngle, currentAngle + m_setPointAngleFlipped),-0.2, 0.2));
            frc::SmartDashboard::PutNumber("Target Angle For Wheel " + m_wheelName, m_setPointAngleFlipped + currentAngle);
        }*/


        
        
    //} else {
      //  m_rotationMotor->Set(0);
    //}
    
   
}



swerveWheel::swerveWheel(short driveCAN, short rotationCAN, short encoderCAN, double p, double i, double d) {
    this->driveCAN = driveCAN;
    this->rotationCAN = rotationCAN;
    this->encoderCAN = encoderCAN;

    
    m_driveMotor = new rev::CANSparkMax{driveCAN, rev::CANSparkMax::MotorType::kBrushless};
    m_rotationMotor = new rev::CANSparkMax{rotationCAN, rev::CANSparkMax::MotorType::kBrushless};
    encoder = new ctre::phoenix6::hardware::CANcoder{encoderCAN};
    this->rel = new rev::SparkRelativeEncoder{m_driveMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)};

    this->refreshPID(p,i,d);

}