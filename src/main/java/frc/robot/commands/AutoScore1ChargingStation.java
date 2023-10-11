// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.DriveBase.DriveZeroGyro;
import frc.robot.subsystems.DriveBase.DriveZeroWheels;
import frc.robot.subsystems.DriveBase.DrivebaseResetEncoders;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore1ChargingStation extends SequentialCommandGroup {
  /** Creates a new AutoScore1ChargingStation. */
  public AutoScore1ChargingStation() {
    addCommands(
        new DriveForDistance(0.0, 0.0, 0.0, 0.0),
        new WaitCommand(1.0),

        new DriveZeroWheels(),
        new DriveZeroGyro(),
        new WaitCommand(1.0),
        new DrivebaseResetEncoders(),
        new DriveForDistance(-3.0, 0.0, 0.0, 1.0),
        new DriveForDistance(3.0, 0.0, 0.0, 1.0),
        new DriveForDistance(-2.0, 0.0, 0.0, 2.4));
  }
}
