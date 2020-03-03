package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class FollowPath extends CommandBase{
    
    private final Timer m_timer = new Timer();
    private final Trajectory trajectory;
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
    private RamseteController ramseteController = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
    private Pose2d poseTolerance = Constants.poseTolerance;

    private Drive drive; 
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;
    private Pose2d poseError = new Pose2d();
    private Pose2d rotateError = new Pose2d();
    private double pathLength = 0;
    private double pathDistTraveled = 0;
    

  public FollowPath(Drive m_drive, Trajectory traj) {
    trajectory = traj;
    drive = m_drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_prevTime = 0;
    var initialState = trajectory.sample(0);
    m_prevSpeeds = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(initialState.velocityMetersPerSecond, 0, initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(ramseteController.calculate(drive.getOdometry(), trajectory.sample(curTime)));

    double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftFeedforward = Constants.driveTrainKSLeft*Math.signum(leftSpeedSetpoint)+  leftSpeedSetpoint*Constants.driveTrainKV +
            Constants.driveTrainKA * (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt;

    double rightFeedforward = Constants.driveTrainKSLeft*Math.signum(rightSpeedSetpoint)+  rightSpeedSetpoint*Constants.driveTrainKV +
            Constants.driveTrainKA * (rightSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt;
    
    drive.setWheelVelocity(targetWheelSpeeds, leftFeedforward, rightFeedforward);

    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;
  }

      /**
     * Returns true if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        final var eTranslate = poseError.getTranslation();
        final var eRotate = rotateError.getRotation();
        final var tolTranslate = poseTolerance.getTranslation();
        final var tolRotate = poseTolerance.getRotation();
        return Math.abs(eTranslate.getX()) < tolTranslate.getX()
                && Math.abs(eTranslate.getY()) < tolTranslate.getY()
                && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
        }
        
  public double getAutoPercentage(){
    return (pathDistTraveled)/ pathLength;
}

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(trajectory.getTotalTimeSeconds());
  }

}