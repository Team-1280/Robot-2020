package frc.robot;

public final class Constants {
    
//-----------------------------------------------------
//CAN Bus information
//------------------------------------------------------

    /* CAN Bus ID's
    ID# 0-3: Drive Base
    ID# 4-5: Shooter 
   */ 
    public static final int CANLeftTalon = 0;
    public static final int CANRightTalon = 1;
    public static final int CANLeftVictor = 2;
    public static final int CANRightVictor = 3;

    // CAN bus timings
    public static final int CAN_Update_Rate = 5; // 5 miliseconds
    public static final int kTimeoutMs = 10;

       
//-----------------------------------------------------
//Robot Ports
//------------------------------------------------------
public static final int PWMIntake1 = 0;
public static final int PWMIntake2 = 0;
public static final int PWMConveyorBelt = 0;
public static final int PWMHopper = 0;

public static final int joystick_left = 0;
public static final int joystick_right = 1;
   
//-----------------------------------------------------
//Robot details & Coefficents
//------------------------------------------------------

    // robot details
    public static final double wheelDiameter = 6;
    public static final double trackWidth = 0; // distance between left wheels and right wheels

    
    // Drive Train PFD control loop coefficents
    public static final double kPLeftAuto = 0;
    public static final double kDLeftAuto = 0;
    public static final double kFLeftAuto = 0;
    public static final double kPRightAuto = 0;
    public static final double kDRightAuto = 0;
    public static final double kFRightAuto = 0;

    public static final double kPLeftTeleop = 0.0;
    public static final double kDLeftTeleop = 0;
    public static final double kFLeftTeleop= 0;
    public static final double kPRightTeleop = 0.0;
    public static final double kDRightTeleop = 0;
    public static final double kFRightTeleop = 0;

     // Shooter PFD control loop coefficents
     public static final double kPLeftShooter = 0;
     public static final double kDLeftShooter = 0;
     public static final double kFLeftShooter = 0;
     public static final double kPRightShooter = 0;
     public static final double kDRightShooter = 0;
     public static final double kFRightShooter = 0;
}
