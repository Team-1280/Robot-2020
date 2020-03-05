package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LazyVictorSPX;
import frc.robot.util.Mathz;

public class Intake extends SubsystemBase{
   private LazyVictorSPX victorIntake = new LazyVictorSPX(Constants.PWMIntake1);
   private LazyVictorSPX victorConveyor = new LazyVictorSPX(Constants.PWMConveyorBelt);
   private LazyVictorSPX victorHopper = new LazyVictorSPX(Constants.PWMHopper);
   private DigitalInput photoElectric1 = new DigitalInput(Constants.DIOphotoElectric);
   private PowerDistributionPanel PDP;

   public Intake(PowerDistributionPanel PDP){
    this.PDP = PDP;
    victorIntake.clearStickyFaults();
    victorHopper.clearStickyFaults();
    victorConveyor.clearStickyFaults();
   }
   
    @Override
    public void periodic() {
      if(getConveyorSensor()){
        setConveyor(Constants.conveyorVoltage);
      }
      else{
        setConveyor(0);
      }
    }
    
  public void setIntakeSpeed(double iSpeed){
    victorIntake.set(ControlMode.PercentOutput, Mathz.VoltageCompensation(iSpeed, PDP.getVoltage(), 12.0));
  }
  public void setConveyor(double cSpeed){
    victorConveyor.set(ControlMode.PercentOutput, Mathz.VoltageCompensation(cSpeed, PDP.getVoltage(), 12.0));
  } 
  public void setHopper(double hSpeed){
    victorHopper.set(ControlMode.PercentOutput, Mathz.VoltageCompensation(hSpeed, PDP.getVoltage(), 12.0));
  }

  public boolean getConveyorSensor(){
    return photoElectric1.get();
  }
}