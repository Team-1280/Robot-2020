/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.PrintWriter;
import java.io.StringWriter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Frame;
import frc.robot.util.PixyCam;
import frc.robot.util.SpiLogger;

public class Pixy extends SubsystemBase {
 
  private PixyCam _p = new PixyCam(); 
  private SpiLogger _spiLogger = new SpiLogger();
  String autoSelected;


  public Pixy() {
    //chooser.addDefault("Default Auto", defaultAuto);
    //		chooser.addObject("My Auto", customAuto);
    //		SmartDashboard.putData("Auto choices", chooser);

	//	SmartDashboard.putString("cp1", "checkpoint #1 what");
		try
		{
			while (true) {
				Frame f = _p.getFrame();
				System.out.println("frame:: - "+f.toString());
			//	SmartDashboard.putString("frame", f.toString());
			}
		}
		catch (Exception e)
		{
			StringWriter writer = new StringWriter();
			e.printStackTrace(new PrintWriter(writer));
  }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
