// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends SubsystemBase {
  private CANSparkMax m_wristMotor;
  private SparkMaxPIDController m_wristPidController;
  private RelativeEncoder m_wristEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new Wrist. */
  public Wrist() {
    m_wristMotor = new CANSparkMax(WristConstants.kWirstMotorPort, MotorType.kBrushless);

    m_wristMotor.restoreFactoryDefaults();

    m_wristPidController = m_wristMotor.getPIDController();

    m_wristEncoder = m_wristMotor.getEncoder();

    kP = 0.1;
    kI = 0.0001;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    m_wristPidController.setP(kP);
    m_wristPidController.set
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
