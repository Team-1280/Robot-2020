package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DriveSpeeds;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.LazyVictorSPX;

public class Drive extends SubsystemBase{
    private LazyTalonSRX talon_left = new LazyTalonSRX(Constants.CANLeftTalon);
    private LazyTalonSRX talon_right = new LazyTalonSRX(Constants.CANRightTalon);
    private LazyVictorSPX victor_left = new LazyVictorSPX(Constants.CANLeftVictor);
    private LazyVictorSPX victor_right = new LazyVictorSPX(Constants.CANRightVictor);
	private ADIS16448_IMU gyro = new ADIS16448_IMU();

	private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.trackWidth));
	private DifferentialDriveOdometry currentOdometry;
	RamseteController controller;

	private double Drivemultiplier = 1;

	private double sensitivityScaler = 100; 
	// smaller the value, higher the sensitivity adjustment

    public Drive(){
		configMotors();
		/* Disable all motor controllers */
		talon_right.set(ControlMode.PercentOutput, 0);
		talon_left.set(ControlMode.PercentOutput, 0);
		
		/* Set Neutral Mode */
		talon_left.setNeutralMode(NeutralMode.Brake);
		talon_right.setNeutralMode(NeutralMode.Brake);

		calibrateGyro();
		zeroSensors();
	}

	private void configMotors(){
		talon_left.configFactoryDefault(10);
		talon_right.configFactoryDefault(10);
		victor_left.configFactoryDefault(10);
		victor_right.configFactoryDefault(10);

		talon_left.configVoltageCompSaturation(12);
		talon_right.configVoltageCompSaturation(12);
		victor_left.configVoltageCompSaturation(12);
		victor_right.configVoltageCompSaturation(12);
		talon_left.enableVoltageCompensation(true);
		talon_right.enableVoltageCompensation(true);
		victor_left.enableVoltageCompensation(true);
		victor_right.enableVoltageCompensation(true);

		talon_left.clearStickyFaults(19);
		talon_right.clearStickyFaults(10);
		victor_left.clearStickyFaults(10);
		victor_right.clearStickyFaults(10);

		talon_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		talon_right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	
		talon_right.setInverted(true);
		victor_right.setInverted(true);

		talon_left.setSensorPhase(Constants.leftSensorPhase);
		talon_right.setSensorPhase(Constants.rightSensorPhase);

		victor_left.follow(talon_left);
		victor_right.follow(talon_right);	
	}
	
	public void configAuto(String autoPath){
		/*
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(autoPath);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		  } catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + autoPath, ex.getStackTrace());
		  }

		currentOdometry = new DifferentialDriveOdometry(getAngle(), trajectory.getInitialPose());
		*/
		zeroSensors();
		talon_left.config_kP(0, Constants.kPLeftAuto, 10);
		talon_left.config_kD(0, Constants.kDLeftAuto, 10);
		talon_right.config_kF(0, Constants.kFLeftAuto, 10);
		talon_left.config_kP(0, Constants.kPRightAuto, 10);
		talon_left.config_kD(0, Constants.kDRightAuto, 10);
		talon_left.config_kF(0, Constants.kFRightAuto, 10);
		talon_right.configClosedloopRamp(12d / 200d, 10);
		talon_left.configClosedloopRamp(12d / 200d, 10);
	}

	public void configTeleop(){
		talon_left.config_kP(0, Constants.kPRightTeleop, 10);
		talon_left.config_kD(0, Constants.kDRightTeleop, 10);
		talon_left.config_kF(0, Constants.kFRightTeleop, 10);
		talon_right.config_kP(0, Constants.kPLeftTeleop, 10);
		talon_right.config_kD(0, Constants.kDRightTeleop, 10);
		talon_right.config_kF(0, Constants.kFLeftTeleop, 10);
		talon_left.configClosedloopRamp(12d / 200d, 10);
		talon_right.configClosedloopRamp(12d / 200d, 10);
	}

	public void configCalibrate(){
		// get k's from Network Tables
		talon_left.config_kP(0, 0, 10);
		talon_left.config_kD(0, 0, 10);
		talon_left.config_kF(0, 0, 10);
		talon_right.config_kP(0, 0, 10);
		talon_right.config_kD(0, 0, 10);
		talon_right.config_kF(0, 0, 10);
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
	
	public Rotation2d getAngle(){
		return new Rotation2d(Math.toRadians(gyro.getAngle()));
	}


	public enum axis{
		x,y,z
	}

	public double getAcceleration(axis AXIS){
		double accel = 0;
		switch(AXIS){
			case x:
				accel = gyro.getAccelInstantX();
				break; 

			case y:
				accel = gyro.getAccelInstantY();
				break; 

			case z: 
				accel= gyro.getAccelInstantZ();
				break;
		}
		return accel;
	}

	public double getLeftDistance(){
		return 0;
	}

	public double getRightDistance(){
		return 0;
	}

	public void calibrateGyro(){
		gyro.calibrate();
	}

	public void setWheelPow(double left, double right){
		talon_right.set(ControlMode.PercentOutput, right);
		talon_left.set(ControlMode.PercentOutput, left);
	}

	public void setWheelVelocity(DriveSpeeds speeds) {
		double leftSetpoint = (speeds.wheelSpeeds.leftMetersPerSecond) * 4096 / (Units.inchesToMeters(Constants.wheelDiameter)* Math.PI);
		double rightSetpoint = (speeds.wheelSpeeds.rightMetersPerSecond) * 4096 / (Units.inchesToMeters(Constants.wheelDiameter) * Math.PI);
		double feedForwardLeft = Constants.driveTrainKV * leftSetpoint + Constants.driveTrainKA * speeds.accelLeft + Constants.driveTrainKS;
		double feedForwardRight = Constants.driveTrainKV * leftSetpoint + Constants.driveTrainKA * speeds.accelLeft + Constants.driveTrainKS;

		talon_left.set(ControlMode.Velocity, leftSetpoint, DemandType.ArbitraryFeedForward, feedForwardLeft);
		talon_right.set(ControlMode.Velocity, rightSetpoint, DemandType.ArbitraryFeedForward, feedForwardRight);
	}

	public void updateOdometry(){
		// Based off of Code Orange FRC 3476
		double leftDistance = TicksToMeters(talon_left.getSelectedSensorPosition(0));
		double rightDistance = TicksToMeters(talon_right.getSelectedSensorPosition(0));

		currentOdometry.update(getAngle(), leftDistance, rightDistance);
	}

	public double TicksToMeters(double ticks){
		// 1 rotation = 4096 ticks
		double wheelCircumference = Math.PI * Units.inchesToMeters(Constants.wheelDiameter);
		return wheelCircumference * ticks / 4096.0;
	}

	public void bangDrive(double mag, double turn, double z){

		double radius = 1/Math.sin(turn); // change 24 to value appropriate for our robot
		//double deltaV = (Constants.trackWidth * Math.PI) * (mag * Drivemultiplier / radius);
		//double sensitivity = Math.pow(Math.abs(mag) + 1, -1 / sensitivityScaler);
		//deltaV *= sensitivity;

		double deltaV = Drivemultiplier*(Math.sin(turn)); // (Ziggy Tuner)

		double vel_left = mag + deltaV;
		double vel_right = mag - deltaV;

		if(vel_left > 1.0){
			vel_right -= (vel_left-1.0);
			vel_left = 1.0;
		}
		else if(vel_right > 1.0){
			vel_left -= (vel_right-1.0);
			vel_right = 1.0;
		} else if(vel_left < -1.0){
			vel_right += (-vel_left -1.0);
			vel_left = -1.0;
		} else if(vel_right < -1.0){
			vel_left += (-vel_right -1.0);
			vel_right = -1.0;
		}
		setWheelPow(vel_left, vel_right);
	}

	public double getVoltage(){
		return (talon_left.getMotorOutputVoltage() + talon_right.getMotorOutputVoltage() + victor_left.getMotorOutputVoltage()+ victor_right.getMotorOutputVoltage())/4.0;
	}

	public Pose2d getOdometry(){
		return currentOdometry.getPoseMeters();
	}

	public DifferentialDriveKinematics getKinematics(){
		return kinematics;
	}

	public Rotation2d getInverse(Rotation2d rotation){
		return new Rotation2d(rotation.getCos(), -rotation.getSin());
	}

	public void zeroSensors(){
		talon_left.setSelectedSensorPosition(0);
		talon_right.setSelectedSensorPosition(0);
	}
}