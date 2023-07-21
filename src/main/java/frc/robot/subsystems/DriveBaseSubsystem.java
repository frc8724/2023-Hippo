package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

public class DriveBaseSubsystem extends SubsystemBase{

    private final SwerveModule frontLeftSwerveModule = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderPorts,
        DriveConstants.kFrontLeftTurningEncoderPorts,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed);

    private final SwerveModule rearLeftSwerveModule =
        new SwerveModule(
            DriveConstants.kRearLeftDriveMotorPort,
            DriveConstants.kRearLeftTurningMotorPort,
            DriveConstants.kRearLeftDriveEncoderPorts,
            DriveConstants.kRearLeftTurningEncoderPorts,
            DriveConstants.kRearLeftDriveEncoderReversed,
            DriveConstants.kRearLeftTurningEncoderReversed);
  
    private final SwerveModule frontRightSwerveModule =
        new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderPorts,
            DriveConstants.kFrontRightTurningEncoderPorts,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed);
  
    private final SwerveModule rearRightSwerveModule =
        new SwerveModule(
            DriveConstants.kRearRightDriveMotorPort,
            DriveConstants.kRearRightTurningMotorPort,
            DriveConstants.kRearRightDriveEncoderPorts,
            DriveConstants.kRearRightTurningEncoderPorts,
            DriveConstants.kRearRightDriveEncoderReversed,
            DriveConstants.kRearRightTurningEncoderReversed);

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeftSwerveModule.getPosition(),
            frontRightSwerveModule.getPosition(),
            rearLeftSwerveModule.getPosition(),
            rearRightSwerveModule.getPosition()
          });

    public DriveBaseSubsystem() {}

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeftSwerveModule.getPosition(),
                frontRightSwerveModule.getPosition(),
                rearLeftSwerveModule.getPosition(),
                rearRightSwerveModule.getPosition()
            }
        );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeftSwerveModule.getPosition(),
                frontRightSwerveModule.getPosition(),
                rearLeftSwerveModule.getPosition(),
                rearRightSwerveModule.getPosition()
            },
            pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds roboChassisSpeeds = null;
        if (fieldRelative) {
            roboChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
        } else {
            roboChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }
        // As of July 18th, the fromDiscreteSpeeds methods were added to ChassisSpeeds to fix drift in complex turns
        // We would need to update to get that method
        //SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromDiscreteSpeeds(roboChassisSpeeds, DriveConstants.kDrivePeriod));
        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(roboChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
        frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
        rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
        rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        frontLeftSwerveModule.setDesiredState(desiredStates[0]);
        frontRightSwerveModule.setDesiredState(desiredStates[1]);
        rearLeftSwerveModule.setDesiredState(desiredStates[2]);
        rearRightSwerveModule.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeftSwerveModule.resetEncoders();
        frontRightSwerveModule.resetEncoders();
        rearLeftSwerveModule.resetEncoders();
        rearRightSwerveModule.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}
