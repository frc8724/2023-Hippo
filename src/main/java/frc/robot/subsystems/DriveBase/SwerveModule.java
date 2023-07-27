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

public class SwerveModule {
    private final Spark m_driveMotor;
    private final Spark m_turningMotor;

    private final Encoder m_driveEncoder;
    private final Encoder m_turningEncoder;

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
            int driveMotorChannel,
            int turningMotorChannel,
            int[] driveEncoderChannels,
            int[] turningEncoderChannels,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed) 
        {
        m_driveMotor = new Spark(driveMotorChannel);
        m_turningMotor = new Spark(turningMotorChannel);

        m_driveEncoder = new Encoder(driveEncoderChannels[0], driveEncoderChannels[1]);
        m_turningEncoder = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

        // Set whether drive encoder should be reversed or not
        m_driveEncoder.setReverseDirection(driveEncoderReversed);

        // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

        // Set whether turning encoder should be reversed or not
        m_turningEncoder.setReverseDirection(turningEncoderReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
      }

      public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(driveOutput);
        m_turningMotor.set(turnOutput);
    }
      
    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_driveEncoder.reset();
        m_turningEncoder.reset();
    }

}
