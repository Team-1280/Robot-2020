package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

public class Limelight extends SubsystemBase{

    private NetworkTable table;
    private NetworkTableEntry tx,ty,ta; 


    public Limelight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }

    @Override
    public void periodic() {

    }

    public double getx(){
        return tx.getDouble(999);
    }

    public double gety(){
        return ty.getDouble(999);
    }

    public double geta(){
        return ta.getDouble(0);
    }

    public boolean isConnected(){
        return gety() != 999;
    }

    /*
    @param int state: ledMode state
    0: use the LED Mode set in the current pipeline
    1: force off
    2: force blink
    3: force on
    */
    public void setLED(int state){
        table.getEntry("ledMode").setNumber(state);
    }

    public double getLimelightDistance(){
        double deltaH = Constants.tapeHeight - Constants.limelightHeight;
        double distanceFeet = ((deltaH) / Math.tan(Math.toRadians(gety()+Constants.mountAngle)));

        return distanceFeet;
    }
}