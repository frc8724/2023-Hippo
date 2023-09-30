// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveBase.DriveForDistance;

public class TestDriveFigureEight extends SequentialCommandGroup {
  /** Creates a new TestDriveFigureEight. */
  public TestDriveFigureEight() {
    addCommands(
        new PrintCommand("Start of Test"),

        new DriveForDistance(0.2, 0.0, 0.0, 1.0), // fwd
        new PrintCommand("Middle of Test"),

        new DriveForDistance(0.0, 0.2, 0.0, 1.0), // right
        new DriveForDistance(-0.2, 0.0, 0.0, 1.0), // back
        new DriveForDistance(0.0, -0.2, 0.0, 1.0), // left
        new DriveForDistance(-0.2, 0.0, 0.0, 1.0), // back
        new DriveForDistance(0.0, 0.2, 0.0, 1.0), // right
        new DriveForDistance(0.2, 0.0, 0.0, 1.0), // fwd
        new DriveForDistance(0.0, -0.2, 0.0, 1.0), // left
        new PrintCommand("End of Test")

    );

  }
}
