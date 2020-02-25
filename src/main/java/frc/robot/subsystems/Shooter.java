package frc.robot.subsystems;


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

public class Shooter extends SubsystemBase{
    private LazyTalonSRX talon_left = new LazyTalonSRX(Constants.CANLeftTalon);
    private LazyTalonSRX talon_right = new LazyTalonSRX(Constants.CANRightTalon);


	private double Drivemultiplier = 1;

	private double sensitivityScaler = 100; 
	// smaller the value, higher the sensitivity adjustment

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
		talon_left.config_kF(0, Constants.kFRightTeleop, 10);
		talon_right.config_kP(0, Constants.kPLeftTeleop, 10);
		talon_right.config_kD(0, Constants.kDRightTeleop, 10);
		talon_right.config_kF(0, Constants.kFLeftTeleop, 10);
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

	public void setWheelVelocity(double RPM) {
        double velocity = RPM / 60.0 * 4096; 
		double leftSetpoint = (velocity);
		double rightSetpoint = (velocity);
		double feedForward = velocity / (Constants.ShooterGearing / Constants.MotorConstant_775 ) + Constants.driveTrainKS;

		talon_left.set(ControlMode.Velocity, leftSetpoint, DemandType.ArbitraryFeedForward, feedForward);
		talon_right.set(ControlMode.Velocity, rightSetpoint, DemandType.ArbitraryFeedForward, feedForward);
	}

	public double getVoltage(){
		return (talon_left.getMotorOutputVoltage() + talon_right.getMotorOutputVoltage())/2.0;
	}



	public void zeroSensors(){
		talon_left.setSelectedSensorPosition(0);
		talon_right.setSelectedSensorPosition(0);
	}
}