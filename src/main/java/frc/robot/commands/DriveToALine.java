// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;
//import jdk.tools.jlink.internal.plugins.AddOptionsPlugin;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToALine extends SequentialCommandGroup {
  /** Creates a new DriveToALine. */
  DriveTrain dt = new DriveTrain();
  public DriveToALine() {
    
    // Add your commands in the addCommands() call, e.g.
    //addCommands(new FooCommand(), new BarCommand());
    //addCommands(new AutoDrive2M(RobotContainer.getDriveTrain(), 2, 0.7));
    //addCommands(new DriveTrain().tankDrive(.5,.5) );
  }
}