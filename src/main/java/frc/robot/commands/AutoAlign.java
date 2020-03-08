/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class AutoAlign extends CommandBase {
  private final Limelight vision;
  private final Drive drive;
  private double lastOffset;
  private double lastTime;
  private Timer timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlign(Drive m_drive, Limelight m_vision) {
    vision = m_vision; 
    drive = m_drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
    addRequirements(drive);
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastOffset = vision.getx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if target not seen, seek.
    double offset = vision.getx();
    double currentTime = timer.get();
    double dt = lastTime - currentTime;
    double correction = Constants.AlignKP * offset + Constants.AlignKD * (offset - lastOffset)/dt;
    drive.setWheelPow(correction, -correction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
