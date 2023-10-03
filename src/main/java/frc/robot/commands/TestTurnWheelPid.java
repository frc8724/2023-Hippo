// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SimpleFalconSubsystem.SwerveTurnWheelTo;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestTurnWheelPid extends SequentialCommandGroup {
  /** Creates a new TestTurnWheelPid. */
  public TestTurnWheelPid(double rad) {
    addCommands(
        new SwerveTurnWheelTo(rad),
        new WaitCommand(5.0));
  }
}
