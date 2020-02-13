package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.LazyVictorSPX;

public class Drive extends SubsystemBase{
    private LazyTalonSRX talon_left = new LazyTalonSRX(Constants.CANLeftTalon);
    private LazyTalonSRX talon_right = new LazyTalonSRX(Constants.CANRightTalon);
    private LazyVictorSPX victor_left = new LazyVictorSPX(Constants.CANLeftVictor);
    private LazyVictorSPX victor_right = new LazyVictorSPX(Constants.CANRightVictor);
	private ADIS16448_IMU gyro = new ADIS16448_IMU();

	private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.trackWidth));
	private DifferentialDriveWheelSpeeds wheelSpeeds;

	private Pose2d currentOdometry;

	// Update the pose
	//DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getGyroHeading(), new Pose2d(5.0, 13.5, new Rotation2d());
	// SensorCollection leftEncoder = new SensorCollection(talon1);
	// SensorCollection rightEncoder = new SensorCollection(talon2);  
	// = new ChassisSpeeds(0, 0, Math.PI);

	private double oldDistance = 0; 

    public Drive(){
		configMotors();
		/* Disable all motor controllers */
		talon_right.set(ControlMode.PercentOutput, 0);
		talon_left.set(ControlMode.PercentOutput, 0);
		
		/* Set Neutral Mode */
		talon_left.setNeutralMode(NeutralMode.Brake);
		talon_right.setNeutralMode(NeutralMode.Brake);

		calibrateGyro();

		// assumes starting point of 0-0. 
		currentOdometry	= new Pose2d(new Translation2d(), getAngle());
	}

	private void configMotors(){
		talon_left.configFactoryDefault();
		talon_right.configFactoryDefault();
		victor_left.configFactoryDefault();
		victor_right.configFactoryDefault();

		talon_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		talon_right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	
		talon_right.setInverted(true);
		victor_right.setInverted(true);
		  
		victor_left.follow(talon_left);
		victor_right.follow(talon_right);	
	}
	
	public void configAuto(){
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
		updateOdometry();
	}
	
	public Rotation2d getAngle(){
		return new Rotation2d(Math.toRadians(gyro.getAngle()));
	}


	public enum axis{
		x,y,z
	}

	public double getAcceleration(axis AXIS){
		double accel = 0;;
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

	public void setWheelPow(double right, double left){
		talon_left.set(ControlMode.PercentOutput, right);
		talon_right.set(ControlMode.PercentOutput, left);
	}

	public void setWheelVelocity(ChassisSpeeds chassisSpeed) {
		wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeed);
		double leftSetpoint = (wheelSpeeds.leftMetersPerSecond) * 4096 / (Units.inchesToMeters(Constants.wheelDiameter)* Math.PI);
		double rightSetpoint = (wheelSpeeds.rightMetersPerSecond) * 4096 / (Units.inchesToMeters(Constants.wheelDiameter) * Math.PI);

		talon_left.set(ControlMode.Velocity, leftSetpoint);
		talon_right.set(ControlMode.Velocity, rightSetpoint);
	}

	public void updateOdometry(){
		double leftDistance = talon_left.getSelectedSensorPosition(0);
		double rightDistance = talon_right.getSelectedSensorPosition(0);
		double currentDistance = (leftDistance + rightDistance) / 2;

		double deltaDistance = currentDistance - oldDistance;
		Translation2d deltaPosition = new Translation2d(deltaDistance, 0);
		Rotation2d deltaRotation = getInverse(getAngle());
		Rotation2d halfRotation = new Rotation2d(deltaRotation.getRadians()/2);

		Transform2d transform = new Transform2d(deltaPosition.rotateBy(halfRotation),deltaRotation);
		currentOdometry = currentOdometry.transformBy(transform);

		oldDistance = currentDistance;
	}

	public double TicksToMeters(double ticks){
		// 1 rotation = 4096 ticks
		double wheelCircumference = Math.PI * Units.inchesToMeters(Constants.wheelDiameter);
		return wheelCircumference * ticks / 4096.0;
	}

	public Rotation2d getInverse(Rotation2d rotation){
		return new Rotation2d(rotation.getCos(), -rotation.getSin());
	}

	

	/*
    public void drive(double turn, double magnitude_turn, double power, boolean quickTurn){
      double steeringAssist = kP*(Math.sin(turn)) *Math.pow(magnitude_turn+0.7,2); // sensitivity scaling. 0.7 = kz (Ziggy Tuner)
      if(!quickTurn){
		talon_left.set(ControlMode.PercentOutput, power - steeringAssist);
		talon_right.set(ControlMode.PercentOutput, power + steeringAssist);
      }
      else{
        talon_left.set(ControlMode.PercentOutput, -3 * steeringAssist);
        talon_right.set(ControlMode.PercentOutput,  3 * steeringAssist);
      }
	}
	*/


}