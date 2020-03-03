package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.commands.BackwardsHopper;
import frc.robot.commands.ForwardsHopper;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.autoCommands.*;

import frc.robot.subsystems.*;
import frc.robot.util.Mathz;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Global Stuff 
  private PowerDistributionPanel PDP = new PowerDistributionPanel();

  // The robot's subsystems and commands are defined here...
  private final Drive drive = new Drive();
  private final Shooter shoot = new Shooter();
  private final Intake intake = new Intake(PDP);

  private final Command intakeBalls = new IntakeBalls(intake); 
  private final Command backHopper = new BackwardsHopper(intake); 
  private final Command forwardHopper = new ForwardsHopper(intake); 


  private ShuffleboardTab window = Shuffleboard.getTab("Drive");
  private NetworkTableEntry example = window.add("Example Entry", 0).getEntry();

     // smaller the value, higher the sensitivity adjustment
     private ShuffleboardTab debugWindow = Shuffleboard.getTab("Autonomous Selection");
     private NetworkTableEntry kP1 = debugWindow.add("kP1", 0).getEntry();
     private NetworkTableEntry kD1 = debugWindow.add("kD1", 0).getEntry();
     private NetworkTableEntry kP2 = debugWindow.add("kP2", 0).getEntry();
     private NetworkTableEntry kD2 = debugWindow.add("kD2", 0).getEntry();
     private NetworkTableEntry velocity = debugWindow.add("Velocity", 0).getEntry();
     private NetworkTableEntry velocityLeftError = debugWindow.add("Left Error", 0).getEntry();
     private NetworkTableEntry velocityRightError = debugWindow.add("Right Error", 0).getEntry();
     private NetworkTableEntry velocity_left = debugWindow.add("velocity right", 0).withWidget("Velocity Left").getEntry();
     private NetworkTableEntry velocity_right = debugWindow.add("velocity left", 0).withWidget("Velocity Right").getEntry();
     

  private Joystick joy_left = new Joystick(Constants.joystick_left);
  private Joystick joy_right = new Joystick(Constants.joystick_right);
  private XboxController xbox = new XboxController(Constants.Xbox);

  private JoystickButton intakeButton = new JoystickButton(xbox, 1);

  private JoystickButton hopperButton = new JoystickButton(xbox, 2);

  private Auto1 auto;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    PDP.clearStickyFaults();
    SolenoidBase.clearAllPCMStickyFaults(0);
  }

  private void configureButtonBindings() {
    intakeButton.whenPressed(intakeBalls);
    hopperButton.whenPressed(forwardHopper);
    hopperButton.whenInactive(backHopper);
  }

  public void TeleopDrive(){
    drive.rainbowDrive(joy_left.getY(), Math.atan(joy_right.getY()/joy_right.getX()) + Math.PI/2, joy_right.getMagnitude(), joy_right.getZ(), joy_right.getX() > 0);
    //drive.bangDrive(joy_left.getY(), Math.sin(getAngle(joy_right)), joy_left.getZ()>0.5); // create buffer time so it will take time for it switch
  }

  public void calibrateTester(){
    double vel = velocity.getDouble(0);
    double vel_right = drive.getSpeedRight();
    double vel_left = drive.getSpeedLeft();
    velocity_right.setDouble(vel_right);
    velocity_left.setDouble(vel_left);
    velocityLeftError.setDouble(Mathz.AbsoluteError(vel, vel_right));
    velocityRightError.setDouble(Mathz.AbsoluteError(vel, vel_left));

    drive.calibratePeriodic(kP1.getDouble(0), kD1.getDouble(0), kP2.getDouble(0), kD2.getDouble(0), vel);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return auto;
  }

/*
  public void loadAuto(){
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      trajectoryList.set(0, trajectory);
    } catch (IOException ex) {
      //DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON1, ex.getStackTrace());
      // output to ShuffleBoard
      trajectoryList.set(0,null);
    }
    int automode 
    switch(){

    }
  }
  */

  public double getSystemVoltage(){
    return PDP.getVoltage();
  }

  public double getTotalCurrent(){
      return PDP.getTotalCurrent();
  }
}
