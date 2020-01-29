package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

public class Limelight extends SubsystemBase{

    private NetworkTable table;
    private NetworkTableEntry tx,ty,ta; 

    public final double realDistance = 31.25; // 18 inches, 28 inches
    public final double limelightHeight = 0.917; //Mid:8.5 Bottom:7 Top:9.25
    public final double tapeHeight = 8.2917;// Bottom:13 Mid:14 Top:15
    public final double limelightScalar = 0.486011021619;

    public Limelight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }

    public double getx(){
        return tx.getDouble(0.0);
    }

    public double gety(){
        return ty.getDouble(0.0);
    }

    public double geta(){
        return ta.getDouble(0.0);
    }

    public double getLimelightDistance(){
        
        double y = gety();
        double distanceFeet = ((tapeHeight - limelightHeight) / Math.tan(Math.toRadians(y+Math.toDegrees(Math.atan(1.39583/1)))));
        double fitFeet = distanceFeet * (limelightScalar * (distanceFeet + 3.073) - .05);
        double fitFeet2 = 0;
        if(fitFeet > 4 && fitFeet <= 10){//Piecewise for distances between 4 feet and 6 feet
            fitFeet2 = fitFeet - .67*(fitFeet-4);
        }
        else if(fitFeet > 10){//Piecewise for distances between 6 feet and ~10.5 feet
            fitFeet2 = fitFeet - 4;
            if(fitFeet2 > 6){
            fitFeet2 = fitFeet2 - .75*(fitFeet2-6);
            }
        }
        fitFeet2 -= .2;
        
        return fitFeet2;
    }
}