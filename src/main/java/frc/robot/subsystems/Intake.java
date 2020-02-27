package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Mathz;
public class Intake extends SubsystemBase{
   private VictorSP victorIntake = new VictorSP(Constants.PWMIntake1);
   private VictorSP victorConveyor = new VictorSP(Constants.PWMConveyorBelt);
   private VictorSP victorHopper = new VictorSP(Constants.PWMHopper);
   private DigitalInput photoElectric1 = new DigitalInput(Constants.DIOphotoElectric);
   private PowerDistributionPanel PDP;

   public Intake(PowerDistributionPanel PDP){
    this.PDP = PDP;
   }
   
    @Override
    public void periodic() {
      if(getConveyorSensor()){
        setConveyor(0.3);
      }
      else{
        setConveyor(0);
      }
    }
    
  public void setIntakeSpeed(double iSpeed){
    victorIntake.set(Mathz.VoltageCompensation(iSpeed, PDP.getVoltage(), 12.0));
  }
  public void setConveyor(double cSpeed){
    victorConveyor.set(Mathz.VoltageCompensation(cSpeed, PDP.getVoltage(), 12.0));
  } 
  public void setHopper(double hSpeed){
    victorHopper.set(Mathz.VoltageCompensation(hSpeed, PDP.getVoltage(), 12.0));
  }

  public boolean getConveyorSensor(){
    return photoElectric1.get();
  }
}