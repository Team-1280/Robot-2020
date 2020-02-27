package frc.robot.subsystems;
import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
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
	
	private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getAngle());
	private RamseteController ramseteController = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
	private Trajectory currentTrajectory;
	private int trajectoryIndex = 0;
	private boolean isTrajectoryFinished = false;

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.trackWidth));

	public Drive(){
        configMotors();
        /* Disable all motor controllers */
        talon_right.set(ControlMode.PercentOutput, 0);
        talon_left.set(ControlMode.PercentOutput, 0);
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
        
        talon_left.setSelectedSensorPosition(0);
		talon_right.setSelectedSensorPosition(0);
    }

    public void setGains(){
        talon_left.config_kP(0, Constants.kPRightTeleop, 10);
        talon_left.config_kD(0, Constants.kDRightTeleop, 10);
        talon_left.config_kF(0, Constants.kFRightTeleop, 10);
        talon_right.config_kP(0, Constants.kPLeftTeleop, 10);
        talon_right.config_kD(0, Constants.kDRightTeleop, 10);
        talon_right.config_kF(0, Constants.kFLeftTeleop, 10);
        talon_left.configClosedloopRamp(12d / 200d, 10);
        talon_right.configClosedloopRamp(12d / 200d, 10);
    }
    
    public void calibratePeriodic(double kP1, double kD1, double kP2, double kD2, double vel){
        talon_left.config_kP(0, kP1, 10);
        talon_left.config_kD(0, kD1, 10);
        talon_right.config_kP(0, kP2, 10);
        talon_right.config_kD(0, kD2, 10);
        talon_left.configClosedloopRamp(12d / 200d, 10);
        talon_right.configClosedloopRamp(12d / 200d, 10);
        setWheelVelocity(new DifferentialDriveWheelSpeeds(vel, vel));
	}
	
	@Override
    public void periodic() {

    }
	
    public void calibrateGyro(){
        gyro.calibrate();
	}
	
    public void setWheelPow(double left, double right){
        talon_right.set(ControlMode.PercentOutput, right, DemandType.ArbitraryFeedForward, Constants.driveTrainKSLeft / 12.0);
        talon_left.set(ControlMode.PercentOutput, left, DemandType.ArbitraryFeedForward, Constants.driveTrainKSRight / 12.0);
    }

    public void setWheelVelocity(DifferentialDriveWheelSpeeds  speeds) {
        double leftSpeed = speeds.leftMetersPerSecond;
        double rightSpeed = speeds.rightMetersPerSecond;
        talon_left.set(ControlMode.Velocity, leftSpeed, DemandType.ArbitraryFeedForward, (Constants.driveTrainKSLeft) / 12);
        talon_right.set(ControlMode.Velocity, rightSpeed, DemandType.ArbitraryFeedForward, (Constants.driveTrainKSRight) / 12.0);
	}
	
    public void updateOdometry(){
        double leftDistance = TicksToMeters(talon_left.getSelectedSensorPosition(0));
        double rightDistance = TicksToMeters(talon_right.getSelectedSensorPosition(0));
        odometry.update(getAngle(), leftDistance, rightDistance);
        System.out.println("X: " + Math.round(odometry.getPoseMeters().getTranslation().getX()*1000.0)/1000.0);
        System.out.println("Y: " + Math.round(odometry.getPoseMeters().getTranslation().getY()*1000.0)/1000.0);
	}

	public boolean isEndSection(Pose2d robotPose){
		boolean finished = false;
		Translation2d poseError = robotPose.getTranslation();
		poseError.plus(getOdometry().getTranslation().times(-1));
		if(Math.abs(poseError.getX()) < Constants.PathTolerance && Math.abs(poseError.getY()) < Constants.PathTolerance){
			finished = true;
		}
		return finished;
	}

	public void updateAuto(){
		updateOdometry();
		if(currentTrajectory != null && isTrajectoryFinished != true){
			State desiredState = currentTrajectory.getStates().get(trajectoryIndex);
			if(isEndSection(desiredState.poseMeters)){
				trajectoryIndex ++;
				if(trajectoryIndex < currentTrajectory.getStates().size()){
					desiredState = currentTrajectory.getStates().get(trajectoryIndex);
				}
				else{
					isTrajectoryFinished = true;
				}
            }
			setWheelVelocity(kinematics.toWheelSpeeds(ramseteController.calculate(getOdometry(), desiredState)));
		}
	}

	public boolean isTrajectoryDone(){
		return isTrajectoryFinished;
	}
	
	public void setTrajectory(Trajectory trajectory){
		currentTrajectory = trajectory;
		trajectoryIndex = 0;
	}
	
    public double TicksToMeters(double ticks){
        // 1 rotation = 4096 ticks
        double wheelCircumference = Math.PI * Units.inchesToMeters(Constants.wheelDiameter);
        return wheelCircumference * ticks / 4096.0;
	}

	public double metersToEncoderTicks(double meters){
		// 1 rotation = 4096 ticks
		double wheelCircumference = Math.PI * Units.inchesToMeters(Constants.wheelDiameter);
		return meters * 4096 / wheelCircumference;
	}
	
    public void teleopDrive(double mag, double turn, double z){
        
	}
	
    public double getVoltage(){
        return (talon_left.getMotorOutputVoltage() + talon_right.getMotorOutputVoltage() + victor_left.getMotorOutputVoltage()+ victor_right.getMotorOutputVoltage())/4.0;
	}
	
    public Pose2d getOdometry(){
        return odometry.getPoseMeters();
	}
	
    public DifferentialDriveKinematics getKinematics(){
        return kinematics;
	}
	
	public Rotation2d getAngle(){
        return new Rotation2d(Math.toRadians(gyro.getGyroAngleX()));
	}
    
    public double getSpeedLeft(){
        return TicksToMeters(talon_left.getSelectedSensorVelocity());
    }

    public double getSpeedRight(){
        return TicksToMeters(talon_right.getSelectedSensorVelocity());
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

    public void zeroSensors(){
        talon_left.setSelectedSensorPosition(0);
        talon_right.setSelectedSensorPosition(0);
	}
}