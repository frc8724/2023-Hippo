// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.TestDriveFigureEight;
import frc.robot.commands.TestDriveInACircle;
import frc.robot.commands.TestTurnWheelPid;
import frc.robot.subsystems.DriveBase.DriveBaseSubsystem;
import frc.robot.subsystems.DriveBase.DriveZeroWheels;
import frc.robot.subsystems.DriveBase.DrivebaseResetEncoders;
import frc.robot.subsystems.DriveBase.SwerveModule;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.IntakeJaw.IntakeJaw;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;
import frc.robot.subsystems.SimpleFalconSubsystem.SwerveTurnWheelTo;
import frc.robot.subsystems.Wrist.Wrist;

import org.mayheminc.util.MayhemExtreme3dPro;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static final DriveBaseSubsystem m_robotDrive = new DriveBaseSubsystem();
	// private final SwerveModule swerveModule = new SwerveModule(
	// "frontLeftDriveMotor",
	// DriveConstants.kFrontLeftDriveMotorPort,
	// "frontLeftTurningMotor",
	// DriveConstants.kFrontLeftTurningMotorPort,
	// DriveConstants.kFrontLeftDriveEncoderReversed,
	// DriveConstants.kFrontLeftTurningEncoderReversed,
	// DriveConstants.MagModule1);

	private final IntakeRollers m_rollers = new IntakeRollers();
	private final IntakeJaw m_jaw = new IntakeJaw();
	private final Elevator m_neck = new Elevator();
	private final Wrist m_wrist = new Wrist();

	private final MayhemExtreme3dPro DriverStick = new MayhemExtreme3dPro(5);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		// Configure default commands
		// m_robotDrive.setDefaultCommand(
		// new RunCommand(
		// () -> m_robotDrive.drive(
		// DriverStick.DeadbandAxis(MayhemExtreme3dPro.Axis.Y, 0.10)
		// * DriveConstants.kMaxSpeedMetersPerSecond,
		// DriverStick.DeadbandAxis(MayhemExtreme3dPro.Axis.X, 0.10)
		// * DriveConstants.kMaxSpeedMetersPerSecond,
		// DriverStick.DeadbandAxis(MayhemExtreme3dPro.Axis.Z, 0.10)
		// * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
		// false),
		// m_robotDrive));

		// m_robotDrive.resetEncoders();

	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {

		DriverStick.Button(9).onTrue(
				new SequentialCommandGroup(
						new DriveZeroWheels(),
						new WaitCommand(1.0),
						new DrivebaseResetEncoders(),
						new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive)));

		// DriverStick.Button(5).whileTrue(new InstantCommand(() ->
		// m_robotDrive.drive(.2, 0, 0, false), m_robotDrive));
		// DriverStick.Button(6).whileTrue(new InstantCommand(() ->
		// m_robotDrive.drive(-.2, 0, 0, false), m_robotDrive));
		// DriverStick.Button(3).whileTrue(new InstantCommand(() ->
		// m_robotDrive.drive(0, -.2, 0, false), m_robotDrive));
		// DriverStick.Button(4).whileTrue(new InstantCommand(() ->
		// m_robotDrive.drive(0, .2, 0, false), m_robotDrive));
		DriverStick.Button(3).onTrue(new TestDriveFigureEight());

		DriverStick.Button(5).onTrue(new TestTurnWheelPid(0.0));
		DriverStick.Button(6).onTrue(new TestTurnWheelPid(Math.PI / 2));

		DriverStick.Button(4).onTrue(new TestDriveInACircle());

		// DriverStick.Button(7).onTrue(new InstantCommand(() ->
		// m_robotDrive.wheelsAt(3.14), m_robotDrive));

		// DriverStick.Button(11).onFalse(new InstantCommand(() ->
		// swerveModule.setTurningWheel(0.0), swerveModule));
		// DriverStick.Button(11).onTrue(new InstantCommand(() ->
		// swerveModule.setTurningWheel(3.14), swerveModule));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		// return Autos.exampleAuto(m_exampleSubsystem);
		return null;
	}
}
