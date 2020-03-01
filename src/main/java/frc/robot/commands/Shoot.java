package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Shoot extends CommandBase {
  private final Shooter shooter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(Shooter shoot) {
    shooter = shoot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    // if limelight not connected -> Shoot balls @ Midline speeds
    // if target in sights -> Shoot 
    // 
    // 
      
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
