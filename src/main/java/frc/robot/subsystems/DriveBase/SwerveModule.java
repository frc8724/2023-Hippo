package frc.robot.subsystems.DriveBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SimpleFalconSubsystem.SimpleFalconSubsystem;

public class SwerveModule {
    private final SimpleFalconSubsystem m_driveMotor;
    private final SimpleFalconSubsystem m_turningMotor;

    private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController m_turningPIDController =
    new ProfiledPIDController(
        ModuleConstants.kPModuleTurningController,
        0,
        0,
        new TrapezoidProfile.Constraints(
            ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
            ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    public SwerveModule(
            String driveMotorName,
            int driveMotorID,
            String turningMotorName,
            int turningMotorID,
            boolean driveReversed,
            boolean turningReversed) 
        {
        m_driveMotor = new SimpleFalconSubsystem(driveMotorName, driveMotorID, driveReversed);
        m_turningMotor = new SimpleFalconSubsystem(turningMotorName, turningMotorID, turningReversed);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        //m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

        // Set whether drive encoder should be reversed or not
        //m_driveEncoder.setReverseDirection(driveEncoderReversed);

        // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        //m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

        // Set whether turning encoder should be reversed or not
        //m_turningEncoder.setReverseDirection(turningEncoderReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveMotor.getRotationalVelocity(), new Rotation2d(m_turningMotor.getRotationAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveMotor.getDistance(), new Rotation2d(m_turningMotor.getRotationAngle()));
      }

      public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_driveMotor.getDistance()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getRotationalVelocity(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(m_turningMotor.getDistance(), state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(driveOutput);
        m_turningMotor.set(turnOutput);
    }
      
    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_driveMotor.set(0);
        m_turningMotor.reset();
    }

}
