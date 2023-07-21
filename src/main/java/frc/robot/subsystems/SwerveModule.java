package frc.robot.subsystems;

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
    private final Spark driveMotor;
    private final Spark turningMotor;

    private final Encoder driveEncoder;
    private final Encoder turningEncoder;

    private final PIDController drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController turningPIDController =
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
         driveMotor = new Spark(driveMotorChannel);
         turningMotor = new Spark(turningMotorChannel);

         driveEncoder = new Encoder(driveEncoderChannels[0], driveEncoderChannels[1]);
         turningEncoder = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

        // Set whether drive encoder should be reversed or not
        driveEncoder.setReverseDirection(driveEncoderReversed);

        // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

        // Set whether turning encoder should be reversed or not
        turningEncoder.setReverseDirection(turningEncoderReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getRate(), new Rotation2d(turningEncoder.getDistance()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getDistance(), new Rotation2d(turningEncoder.getDistance()));
      }

      public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.getDistance()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(turningEncoder.getDistance(), state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        driveMotor.set(driveOutput);
        turningMotor.set(turnOutput);
    }
      
    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        driveEncoder.reset();
        turningEncoder.reset();
    }

}
