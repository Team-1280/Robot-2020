package frc.robot.commands;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Shoot extends CommandBase {
  private final Shooter shooter;
  private final Limelight vision;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(Shooter shoot, Limelight m_vision) {
    shooter = shoot;
    vision = m_vision;
    // Use ddRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, vision);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    // if limelight not connected -> Shoot balls @ Midline speeds
    // if target in sights -> Shoot 
    
  }

  public void setRPM(){

  }

  //public 

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
