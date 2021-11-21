// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.TurnGyroPID;
import frc.robot.commands.DriveForward; // Needs to be changed when merge
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedPath extends SequentialCommandGroup {
  DriveTrain dt = new DriveTrain();

  // Drive square
  public RedPath() {
    addCommands(
      new DriveForward(dt, 1.0),
      new TurnGyroPID(dt, 90.0),
      new DriveForward(dt, 1.0),
      new TurnGyroPID(dt, 90.0),
      new DriveForward(dt, 1.0),
      new TurnGyroPID(dt, 90.0),
      new DriveForward(dt, 1.0)
    );
  }
}
