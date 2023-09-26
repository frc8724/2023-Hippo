// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveTurnWheelAt extends InstantCommand {
  double rad;
  SwerveTurningFalcon subsystem;

  public SwerveTurnWheelAt(double r) {
    rad = r;

    addRequirements(RobotContainer.m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.m_robotDrive.setWheelsAt(rad);
    RobotContainer.m_robotDrive.zeroWheels();
  }
}
