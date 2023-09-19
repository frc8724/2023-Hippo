package frc.robot.subsystems.DriveBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SimpleFalconSubsystem.SwerveDriveFalcon;
import frc.robot.subsystems.SimpleFalconSubsystem.SwerveTurningFalcon;

public class SwerveModule {
    private final SwerveDriveFalcon m_driveMotor;
    private final SwerveTurningFalcon m_turningMotor;
    private final SwerveEncoder m_magEncoder;

    public SwerveModule(
            String driveMotorName,
            int driveMotorID,
            String turningMotorName,
            int turningMotorID,
            boolean driveReversed,
            boolean turningReversed,
            int magInput) {
        m_driveMotor = new SwerveDriveFalcon(driveMotorName, driveMotorID, driveReversed);
        m_turningMotor = new SwerveTurningFalcon(turningMotorName, turningMotorID, turningReversed);
        m_magEncoder = new SwerveEncoder(magInput);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveMotor.getRotationalVelocity(),
                new Rotation2d(m_turningMotor.getRotationRadians()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveMotor.getDistance(),
                new Rotation2d(m_turningMotor.getRotationRadians()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_turningMotor.getRotationRadians()));

        // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(state.speedMetersPerSecond);
        m_turningMotor.set(state.angle.getRadians());
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_driveMotor.reset();
        m_turningMotor.reset();
    }

}
