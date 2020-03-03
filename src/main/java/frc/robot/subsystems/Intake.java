package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Mathz;
public class Intake extends SubsystemBase{
   private VictorSPX victorIntake = new VictorSPX(Constants.CANBallSysIntake);
   private VictorSPX victorConveyor = new VictorSPX(Constants.CANBallSysConveyor);
   private VictorSPX victorHopper = new VictorSPX(Constants.CANBallSysHopper);
   private DigitalInput photoElectric1 = new DigitalInput(Constants.DIOphotoElectric);
   private PowerDistributionPanel PDP;

   public Intake(PowerDistributionPanel PDP){
    this.PDP = PDP;
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

  public void configMotors(){
    victorIntake.configFactoryDefault(10);
		victorConveyor.configFactoryDefault(10);
    victorHopper.configFactoryDefault(10);
    
		victorIntake.configVoltageCompSaturation(12);
    victorConveyor.configVoltageCompSaturation(12);
    victorHopper.configVoltageCompSaturation(12);

    victorIntake.enableVoltageCompensation(true);
		victorHopper.enableVoltageCompensation(true);
		victorConveyor.enableVoltageCompensation(true);

		victorIntake.clearStickyFaults(19);
    victorConveyor.clearStickyFaults(10);
    victorHopper.clearStickyFaults(10);
  }
    
  public void setIntakeSpeed(double iSpeed){
    victorIntake.set(ControlMode.PercentOutput, iSpeed);
  }

  public void setConveyor(double cSpeed){
    victorConveyor.set(ControlMode.PercentOutput, cSpeed);
  } 

  public void setHopper(double hSpeed){
    victorHopper.set(ControlMode.PercentOutput, hSpeed);
  }

  public boolean getConveyorSensor(){
    return photoElectric1.get();
  }
}