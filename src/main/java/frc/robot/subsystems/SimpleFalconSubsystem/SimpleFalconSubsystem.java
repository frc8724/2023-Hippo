// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class SimpleFalconSubsystem extends SubsystemBase {
  private TalonFX motor;
  private String name;

  /** Creates a new SimpleFalconSubsystem. */
  public SimpleFalconSubsystem(String name, int id) {
    motor = new TalonFX(id);
    this.name = name;
  }

  public void set(double percent)
  {
    motor.set(TalonFXControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(this.name + " velocity", motor.getSelectedSensorVelocity());
    SmartDashboard.putNumber(this.name + " position", motor.getSelectedSensorPosition());
  }
}
