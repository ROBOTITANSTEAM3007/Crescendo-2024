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

    frc::SmartDashboard::PutBoolean("FUCKING WORKS", (m_p == 0.05));

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
    m_stickTwist = driveStick->GetTwist();

    double m_magnitude = std::sqrt(std::pow(fabs(m_stickY), 2) + std::pow(fabs(m_stickX), 2)); //Pythagorean theorem
	if(m_magnitude > 0.2) {
        m_magnitude = (m_magnitude - 0.2) * 1.25;
		//Control for an undefined slope
		if(m_stickY != 0) {

			m_angleR = atan(m_stickX/m_stickY);  //Should be between 90 and -90 degrees
			m_angleD = (m_angleR * 57.2958);
		} else if(m_stickX > 0) {
			m_angleD = 90;
		} else {
			m_angleD = -90;
		}
		if(m_stickY < 0) {
    		m_angleD = 180 + m_angleD;
		} else if(m_stickX < 0){
		    m_angleD = m_angleD + 360;
		}
        frc::SmartDashboard::PutNumber("Joystick Angle",m_angleD);

        m_angleD -= (double)imu.GetAngle(imu.kYaw);
        //Target angles for all four wheels
        calculateTurn();

        //Tell motors which way to drive
        frontLeft.setDrive(-m_magnitude);
        frontRight.setDrive(m_magnitude);
        backLeft.setDrive(-m_magnitude);
        backRight.setDrive(-m_magnitude);
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

void swerveDrive::calculateTurn(){
    m_twistAngle = m_stickTwist * 45;

    //If the front left wheel is in the Back
    if(fabs(turnClosest(m_angleD, frontLeftAngleToCenter)) <= 90) {
        frontLeft.setRotation(m_angleD + m_twistAngle);
    }
    //If it's in the Front 
    else {
        frontLeft.setRotation(m_angleD - m_twistAngle);
    }
    //If the front right wheel is in the back
    if(fabs(turnClosest(m_angleD, frontRightAngleToCenter)) > 90) {
        frontRight.setRotation(m_angleD + m_twistAngle);
    }
    //If it's in the front
    else {
        frontRight.setRotation(m_angleD - m_twistAngle);
    }
    //If the left back wheel is in the front
    if(fabs(turnClosest(m_angleD, backLeftAngleToCenter)) < 90) {
        backLeft.setRotation(m_angleD - m_twistAngle);
    }
    //If it's in the back 
    else {
        backLeft.setRotation(m_angleD + m_twistAngle);
    }
    //If the back left wheel is in the front
    if(fabs(turnClosest(m_angleD, backRightAngleToCenter)) <= 90) {
        backRight.setRotation(m_angleD - m_twistAngle);
    }
    //If it's in the back 
    else {
        backRight.setRotation(m_angleD + m_twistAngle);
    }
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
    m_stickTwist = driveStick->GetTwist();

    double m_magnitude = std::sqrt(std::pow(fabs(m_stickY), 2) + std::pow(fabs(m_stickX), 2)); //Pythagorean theorem
	if(m_magnitude > 0.2) {
        m_magnitude = (m_magnitude - 0.2) * 1.25;

		//Control for an undefined slope
		if(m_stickY != 0) {

			m_angleR = atan(m_stickX/m_stickY);  //Should be between 90 and -90 degrees
			m_angleD = (m_angleR * 57.2958);
		} else if(m_stickX > 0) {
			m_angleD = 90;
		} else {
			m_angleD = -90;
		}
		if(m_stickY < 0) {
    		m_angleD = 180 + m_angleD;
		} else if(m_stickX < 0){
		    m_angleD = m_angleD + 360;
		}
        frc::SmartDashboard::PutNumber("Joystick Angle",m_angleD);
        //Target angles for all four wheels
        calculateTurn();

        //Tell motors which way to drive
        frontLeft.setDrive(-m_magnitude);
        frontRight.setDrive(m_magnitude);
        backLeft.setDrive(-m_magnitude);
        backRight.setDrive(-m_magnitude);
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