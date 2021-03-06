/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import frc.robot.subsystems.Drive;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Auto1 extends CommandBase {

  private Drive drive;
  private ArrayList<Trajectory> trajectoryList = new ArrayList<Trajectory>(1);
  private int trajectoryIndex = 0;
  private boolean isDone = false;

  private final String trajectoryJSON1 = "paths/YourPath.wpilib.json";
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public Auto1(Drive m_drive, ArrayList<Trajectory> m_trajectoryList) {
    drive = m_drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!drive.isTrajectoryDone()){
    drive.updateAuto();
    }
    else{
      isDone = true;
    }
    /*
    else{
      //trajectoryIndex ++;
      //drive.setTrajectory(trajectoryList.get(trajectoryIndex))
    }
    */
    // update trajectory until @ end of path
      // @ certain points along path, call other commands
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // once end of path of last trajectory in array is @ end
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
