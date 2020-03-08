package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.Mathz;

public class Shooter extends SubsystemBase{
    private LazyTalonSRX talon_left = new LazyTalonSRX(Constants.CANLeftShooter);
    private LazyTalonSRX talon_right = new LazyTalonSRX(Constants.CANRightShooter);
	private double RPM = 0;

    public Shooter(){
		configMotors();
		/* Disable all motor controllers */
		talon_right.set(ControlMode.PercentOutput, 0);
		talon_left.set(ControlMode.PercentOutput, 0);
		/* Set Neutral Mode */
		talon_left.setNeutralMode(NeutralMode.Brake);
		talon_right.setNeutralMode(NeutralMode.Brake);

		zeroSensors();
	}

	private void configMotors(){
		talon_left.configFactoryDefault(10);
		talon_right.configFactoryDefault(10);
		
		talon_left.configVoltageCompSaturation(12);
		talon_right.configVoltageCompSaturation(12);
		talon_left.enableVoltageCompensation(true);
		talon_right.enableVoltageCompensation(true);

		talon_left.clearStickyFaults(19);
		talon_right.clearStickyFaults(10);

		talon_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		talon_right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	
		talon_right.setInverted(true);

		talon_left.setSensorPhase(Constants.boolLeftSensorPhase);
		talon_right.setSensorPhase(Constants.boolRightSensorPhase);
	}

	public void configTeleop(){
		talon_left.config_kP(0, Constants.kPRightTeleop, 10);
		talon_left.config_kD(0, Constants.kDRightTeleop, 10);
		talon_left.config_kF(0, Constants.kFShooters, 10);
		talon_right.config_kP(0, Constants.kPLeftTeleop, 10);
		talon_right.config_kD(0, Constants.kDRightTeleop, 10);
		talon_right.config_kF(0, Constants.kFShooters, 10);
		talon_left.configClosedloopRamp(12d / 200d, 10);
		talon_right.configClosedloopRamp(12d / 200d, 10);
    }

	public void configTest(){
		talon_left.config_kP(0, 0, 10);
		talon_left.config_kD(0, 0, 10);
		talon_left.config_kF(0, 0, 10);
		talon_right.config_kP(0, 0, 10);
		talon_right.config_kD(0, 0, 10);
		talon_right.config_kF(0, 0, 10);
	}

    @Override
    public void periodic() {

	}

	public synchronized void setRPM(double speed){
		RPM = speed;
	}

	public void updateWheelVelocity() {
		talon_left.set(ControlMode.Velocity, Mathz.RPMtoTPS(RPM));
		talon_right.set(ControlMode.Velocity, Mathz.RPMtoTPS(RPM));
	}

	public double getLeftVelocity(){
		return Mathz.RPMtoTPS(talon_left.getSensorCollection().getQuadratureVelocity());
	}

	public double getRightVelocity(){
		return Mathz.RPMtoTPS(talon_right.getSensorCollection().getQuadratureVelocity());
	}


	public void setPercent(double percent){
		talon_left.set(ControlMode.PercentOutput, percent);
		talon_right.set(ControlMode.PercentOutput, percent);
	}

	public double getVoltage(){
		return (talon_left.getMotorOutputVoltage() + talon_right.getMotorOutputVoltage())/2.0;
	}

	public void zeroSensors(){
		talon_left.setSelectedSensorPosition(0);
		talon_right.setSelectedSensorPosition(0);
	}
}