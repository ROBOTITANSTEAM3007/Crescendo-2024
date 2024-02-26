/*Defines a NEO motor based solution for swerve drive.*/

#pragma once
#include "Defines.h"







class swerveWheel{

    public:

        //Polarity of 0 means that 0 degrees = 0, polarity of 1 means that 180 degrees = 0
        void setRotation(double targetAngle);
    
        void setDrive(double driveMag);

        //Takes 3 arguments, CAN ID for drive wheel, rotation wheel, and cancoder
        swerveWheel(short driveCAN, short rotationCAN, short encoderCAN, double p, double i, double d);

        //Made seperate for ease of use input PID
        void refreshPID(double P, double I, double D);

        //Tells motors to not turn
        void chill();

        //Get closest angle between two angles a is current angle and b is target angle
        double closestAngle(double a, double b);

        std::string m_wheelName;

        double currentAngle;

        double m_angleOffset;

        bool inverted = 0;

        //Encoder pointer
        ctre::phoenix6::hardware::CANcoder* encoder;

        double m_turnAngle;



    private:

        short driveCAN;
        short rotationCAN;
        short encoderCAN;

        //PID initializing
        frc::PIDController m_directionController{0,0,0};

        //Instantiate the motors for each wheel
        rev::CANSparkMax* m_driveMotor;
        rev::CANSparkMax* m_rotationMotor;




        double m_bestTargetAngle;
        double m_setPointAngle;
        double m_setPointAngleFlipped;

        frc::SlewRateLimiter<units::scalar> speedLimiter{3 / 1_s};




        double m_p;
        double m_i;
        double m_d;




        bool m_polarity = 0;

        double angleOffset;


    



};

class swerveDrive  {

    public:
        //Read Encoders to smartdashboard
        void readEncoders();

        void robotRelativeDrive();

        void refreshPID();

        swerveDrive();

        //Point wheels at zero
        void findZeros();

        // Calibrate PID on each wheel
        void calibPID();

        void calculateTurn();

        void inPlaceTurn();



    private:
        //Can IDs for CANSparkMax must be variable not definitions due to a compiler issue
        const short frontLeftDriveMotorID = 8;
        const short frontRightDriveMotorID = 1;
        const short backLeftDriveMotorID = 6;
        const short backRightDriveMotorID = 3;

        //Rotation motor CAN IDs
        const short frontLeftRotationMotorID = 7;
        const short frontRightRotationMotorID = 2;
        const short backLeftRotationMotorID = 5;
        const short backRightRotationMotorID = 4;

        //CANCoder CAN IDs
        const short frontLeftRotationEncoderID = 9;
        const short frontRightRotationEncoderID = 12;
        const short backLeftRotationEncoderID = 10;
        const short backRightRotationEncoderID = 11;

        frc::Joystick driveStick{0};

        frc::SendableChooser<std::string> m_motorChoice;
        const std::string k_frontLeftChoice = "Front Left Motor";
        const std::string k_frontRightChoice = "Front Right Motor";
        const std::string k_backLeftChoice = "Back Left Motor";
        const std::string k_backRightChoice = "Back Right Motor";
        std::string m_selectedMotor;


        double m_stickY;
        double m_stickX;
        double m_stickTwist;

        //Angle in Radians
        double m_angleR;
        //Angle in Degrees
        double m_angleD;

        //How fast we are to move
        double m_magnitude;

        //How much to turn, between -45 and 45 degrees
        double m_twistAngle;



        double m_calibAngle = 0;

        double m_p = 0.005;
        double m_i = 0;
        double m_d = 0;


        double m_frontRightPOffset = 0;
        double m_frontLeftPOffset = 0;
        double m_backLeftPOffset = 0;
        double m_backRightPOffset = 0;



        void testZeros();


    protected:

        swerveWheel frontLeft{frontLeftDriveMotorID, frontLeftRotationMotorID, frontLeftRotationEncoderID, m_p, m_i, m_d};
        swerveWheel frontRight{frontRightDriveMotorID, frontRightRotationMotorID, frontRightRotationEncoderID, m_p, m_i, m_d};
        swerveWheel backLeft{backLeftDriveMotorID, backLeftRotationMotorID, backLeftRotationEncoderID, m_p, m_i, m_d};
        swerveWheel backRight{backRightDriveMotorID, backRightRotationMotorID, backRightRotationEncoderID, m_p, m_i, m_d};
        

        

        
    



};
