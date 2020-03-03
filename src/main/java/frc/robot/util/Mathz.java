package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;

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

    public static double getDistance(Pose2d p1, Pose2d p2){
        double deltaX2 = Math.pow(p1.getTranslation().getX() - p2.getTranslation().getX(),2);
        double deltaY2 = Math.pow(p1.getTranslation().getY() - p2.getTranslation().getY(),2);
        return Math.sqrt(deltaX2 + deltaY2);
    }

    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }
}