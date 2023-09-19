// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class SwerveTurningFalcon extends SubsystemBase {
  private TalonFX motor;
  private String name;

  final double Turning0p5RotationTicks = 21943.0;

  /** Creates a new SimpleFalconSubsystem. */
  public SwerveTurningFalcon(String name, int id, boolean invert) {
    motor = new TalonFX(id);
    motor.setInverted(invert);
    this.name = name;
    motor.setSelectedSensorPosition(0);

    motor.config_kP(0, 0.05);
    motor.config_kI(0, 0.0);
    motor.config_kD(0, 0.0);
    motor.config_kF(0, 0.0);

    motor.configNominalOutputForward(0.0);
    motor.configNominalOutputReverse(0.0);
    motor.configPeakOutputForward(+12.0);
    motor.configPeakOutputReverse(-12.0);
    motor.configNeutralDeadband(0.0);
    motor.setNeutralMode(NeutralMode.Coast);
  }

  double m_set;

  double convertRadiansToTicks(double rads) {
    return rads * Turning0p5RotationTicks / Math.PI;
  }

  /**
   * 
   * @param value radians
   */
  public void set(double value) {
    double ticks = convertRadiansToTicks(value);
    motor.set(TalonFXControlMode.Position, ticks);
    m_set = ticks;
  }

  public double getRotationRadians() {
    return motor.getSelectedSensorPosition() / Turning0p5RotationTicks * Math.PI;
  }

  public void reset() {
    set(0.0);
    motor.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(this.name + " position", motor.getSelectedSensorPosition());
    SmartDashboard.putNumber(this.name + " rads", this.getRotationRadians());
    SmartDashboard.putNumber(this.name + " m_set", m_set);
  }
}
