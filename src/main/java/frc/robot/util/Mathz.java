package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;

public class Mathz {
    public static double getAngle(Joystick joy){
        return Math.atan(joy.getY()/joy.getX());
    }
    
    // TPS = Tick Per Second
	public static double RPMtoTPS(double RPM){
		return RPM * 4096.0 / 60.0;
    }
    
    public static double Deadband(double value, double threshold) {
        if (value >= Math.abs(threshold)) 
            return value;
            
        if (value <= -1 * Math.abs(threshold))
            return value;
            
        /* Outside deadband */
        return 0;
        }

    public static double AbsoluteError(double x1, double x2){
        return Math.abs(x1) - Math.abs(x2);
    }

    public static double VoltageCompensation(double percentOutput, double SystemVoltage, double max){
        return percentOutput * max / SystemVoltage;
    }
}