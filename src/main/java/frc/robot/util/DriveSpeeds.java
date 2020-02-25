package frc.robot.util;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class DriveSpeeds {
    public DifferentialDriveWheelSpeeds wheelSpeeds;
    public double accelLeft = 0;
    public double accelRight = 0;

    public DriveSpeeds(DifferentialDriveWheelSpeeds speeds, double acLeft,double acRight){
        accelLeft = acLeft;
        accelRight = acRight;
        wheelSpeeds =  speeds;
    }
}