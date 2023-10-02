// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveBase.DriveForDistance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestDriveInACircle extends SequentialCommandGroup {
  /** Creates a new TestDriveInACircle. */
  final double twoPi = Math.PI * 2;
  final int numOfSteps = 8;

  public TestDriveInACircle() {
    for (int i = 0; i < numOfSteps; i++) {
      double step = (double) i / (double) numOfSteps;
      addCommands(
          new PrintCommand("Step: " + i),
          new DriveForDistance(0.2 * Math.cos(twoPi * step), 0.2 * Math.sin(twoPi * step), 0.0, 0.1));
    }
  }
}
