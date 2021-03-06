package frc.robot;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.util.Mathz;

public final class Constants {

    public final static double mountAngle = 40; // in degrees

    public final static double realDistance = 31.25; // 18 inches, 28 inches
    public final static double limelightHeight = Units.inchesToMeters(24); //Mid:8.5 Bottom:7 Top: 9.25
    public final static double tapeHeight = 8.2917;// Bottom:13 Mid:14 Top:15    
    
//-----------------------------------------------------
//CAN Bus information
//------------------------------------------------------

    /* CAN Bus ID's
    ID# 0-3: Drive Base
    ID# 4-5: Shooter 
   */

    public static final int CANLeftTalon = 2;
    public static final int CANRightTalon = 1;
    public static final int CANLeftVictor = 3;
    public static final int CANRightVictor = 0;
    public static final int CANLeftShooter = 14;
    public static final int CANRightShooter = 15;
    public static final int CANBallSysIntake = 11;
    public static final int CANBallSysHopper = 13;
    //public static final int CANBallSysConveyor = 8;


    // CAN bus timings
    public static final int CAN_Update_Rate = 10; // 5 miliseconds
    public static final int kTimeoutMs = 10;

    // 'direction' of sensors
    public static final boolean leftSensorPhase = true;
    public static final boolean rightSensorPhase = false;

       
//-----------------------------------------------------
//Robot Ports
//------------------------------------------------------
    public static final int PWMDeployer = 2;
    public static final int PWMClimbMtr1 = 0;
    public static final int PWMClimbMtr2 = 2;
    public static final int PWMConveyorBelt= 4;

    public static final int DIOphotoElectric = 0;

    public static final int joystick_left = 0;
    public static final int joystick_right = 1;
    public static final int Xbox = 2;
   
//-----------------------------------------------------
//Robot details & Coefficents
//------------------------------------------------------

    // robot details
    public static final double wheelDiameter = 6; 
     // distance between left wheels and right wheels
    public static final double trackWidth = 9 + 11.0/16.0; // inches 

    
    // Drive Train PFD control loop coefficents
    public static final double kPLeftAuto = 0;
    public static final double kDLeftAuto = 0;
    public static final double kFLeftAuto = 0; // encoder ticks / Volt in 
    public static final double kPRightAuto = 0;
    public static final double kDRightAuto = 0;
    public static final double kFRightAuto = 0;

    public static final double kPLeftTeleop = 0.0;
    public static final double kDLeftTeleop = 0;
    public static final double kFLeftTeleop = 0;
    public static final double kPRightTeleop = 0.0;
    public static final double kDRightTeleop = 0;
    public static final double kFRightTeleop = 0;

     // Shooter PFD control loop coefficents
     public static final double kPLeftShooter = 0;
     public static final double kDLeftShooter = 0;
     public static final double kPRightShooter = 0;
     public static final double kDRightShooter = 0;
     public static final double kFShooters = Mathz.RPMtoTPS((Constants.ShooterGearing / Constants.MotorConstant_775)) * 1023.0 / 12.0; // calculated ideal feedfoward gain (full output is @ 1023)

     // Ramsete Controller constants
     public static final double kRamseteB = 2.0;
     public static final double kRamseteZeta = 0.7;

     // Feedforward constants
     public static final double driveTrainKV = 0;
     public static final double driveTrainKA = 0;
     public static final double driveTrainKSLeft = 0;
     public static final double driveTrainKSRight = 0;

     // Shooter Constants
     public static final double ShooterGearing = 4; 
     public static final double MotorConstant_775 = 1561; // RPM/Volt
     public static final boolean boolLeftSensorPhase = false;
     public static final boolean boolRightSensorPhase = false;

     // 
     public static final double PathTolerance = 0.05;

     // 
     public static final double conveyorVoltage = 0.7;
}
