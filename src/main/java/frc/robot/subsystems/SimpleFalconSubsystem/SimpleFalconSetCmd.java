// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SimpleFalconSetCmd extends CommandBase {
  DoubleSupplier m_ds;
  SimpleFalconSubsystem m_subsystem;

  /** Creates a new SimpleFalconSetCmd. */
  public SimpleFalconSetCmd(SimpleFalconSubsystem subsystem, DoubleSupplier ds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_ds = ds;  
    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_subsystem.set(m_ds.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}