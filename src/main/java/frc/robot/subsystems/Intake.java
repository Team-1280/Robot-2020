package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase{
   VictorSP victor1=new VictorSP(0);
   VictorSP victor3=new VictorSP(2);
   VictorSP victor4=new VictorSP(3);
   DigitalInput photoElectric1 = new DigitalInput(0);
   DigitalInput photoElectric2 = new DigitalInput(0);
   
    @Override
    public void periodic() {
      
    }
    
  public void setIntakeSpeed(double iSpeed){
    victor1.set(iSpeed);
  }
  public void setConveyor(double cSpeed){
    victor3.set(cSpeed);
  } 
  public void setHopper(double hSpeed){
    victor4.set(hSpeed);
  }
}