// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveBase.DriveBaseSubsystem;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.IntakeJaw.IntakeJaw;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;
import frc.robot.subsystems.Wrist.Wrist;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
	private final DriveBaseSubsystem m_robotDrive = new DriveBaseSubsystem();
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

	private final IntakeRollers m_rollers = new IntakeRollers();
	private final IntakeJaw m_jaw = new IntakeJaw();
	private final Elevator m_neck = new Elevator();
	private final Wrist m_wrist = new Wrist();

	private final Joystick m_joystick = new Joystick(0);

	private final Trigger m_buttonTrigger0 = new JoystickButton(m_joystick, 0);
	private final Trigger m_buttonTrigger1 = new JoystickButton(m_joystick, 1);
	private final Trigger m_buttonTrigger2 = new JoystickButton(m_joystick, 2);
	private final Trigger m_buttonTrigger3 = new JoystickButton(m_joystick, 3);
	private final Trigger m_buttonTrigger4 = new JoystickButton(m_joystick, 4);
	private final Trigger m_buttonTrigger5 = new JoystickButton(m_joystick, 5);
	private final Trigger m_buttonTrigger6 = new JoystickButton(m_joystick, 6);
	private final Trigger m_buttonTrigger7 = new JoystickButton(m_joystick, 7);
	private final Trigger m_buttonTrigger8 = new JoystickButton(m_joystick, 8);
	private final Trigger m_buttonTrigger9 = new JoystickButton(m_joystick, 9);
	private final Trigger m_buttonTrigger10 = new JoystickButton(m_joystick, 10);
	private final Trigger m_buttonTrigger11 = new JoystickButton(m_joystick, 11);
	private final Trigger m_buttonTrigger12 = new JoystickButton(m_joystick, 12);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		// Configure default commands
		m_robotDrive.setDefaultCommand(
				new RunCommand(
						() -> m_robotDrive.drive(
								m_joystick.getY() * DriveConstants.kMaxSpeedMetersPerSecond,
								m_joystick.getX() * DriveConstants.kMaxSpeedMetersPerSecond,
								m_joystick.getZ() * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
								false),
						m_robotDrive));
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
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		new Trigger(m_exampleSubsystem::exampleCondition)
				.onTrue(new ExampleCommand(m_exampleSubsystem));

		// Schedule `exampleMethodCommand` when the joysticks trigger/button1 is
		// pressed,
		// cancelling on release.
		m_buttonTrigger0.whileTrue(m_exampleSubsystem.exampleMethodCommand());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(m_exampleSubsystem);
	}
}
