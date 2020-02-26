/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Auto3Ball extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Shooter shooter;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Auto3Ball(Shooter shoot) {
    shooter = shoot;
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // IS limelight Connected? 
        // Yes? 
    // check position of trarget. Is target in view? -> if not, servo until it is.

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // update odometry
    // update trajectory until @ end of path
      // @ certain points along path, call other commands
    // set new path or end
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // once end of path of last trajectory in array is @ end
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isPathComplete(){
    // if final position ~ 
    return false;
  }

}
