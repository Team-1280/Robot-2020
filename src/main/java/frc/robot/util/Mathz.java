package frc.robot.util;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

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

    public static double getTrajectoryLength(Trajectory trajectory){
       List<State> traj = trajectory.getStates();
       int arraySize = traj.size();
       double pathLength = 0.0;

       for(int i = 0; i < arraySize; i++){
        traj.get(i);
       }
       return 0;
    }
}