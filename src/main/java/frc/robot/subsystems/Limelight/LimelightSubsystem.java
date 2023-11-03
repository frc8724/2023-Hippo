// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv; // Whether the limelight has any valid targets (0-1)
    NetworkTableEntry tx; // Horizontal Offset from Crosshair to target (-27 degrees to 27 degrees)
    NetworkTableEntry ty; // Vertical Offset From Crosshair to target (-20.5 degrees to 20.5 degrees)
    NetworkTableEntry ta; // Target Area (0% of image to 100% of image)
    NetworkTableEntry ts; // Skew or Rotation (-90 degrees to 0 degrees)
    NetworkTableEntry tl; // The pipeline's latency contribution (ms) Add at least 11ms for image capture
                          // latency.
    NetworkTableEntry tshort; // Sidelength of shortest side of the fitted bounding box (pixels)
    NetworkTableEntry tlong; // Sidelength of longest side of the fitted bounding box (pixels)
    NetworkTableEntry thoriz; // Horizontal sidelength of the rough bounding box (0-320 pixels)
    NetworkTableEntry tvert; // Vertical sidelength of the rough bounding box (0-320 pixels)

    public LimelightSubsystem() {
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
        tl = table.getEntry("tl");
        tlong = table.getEntry("tshort");
        tlong = table.getEntry("tlong");
        thoriz = table.getEntry("thoriz");
        tvert = table.getEntry("tvert");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LimelightImage?", tv.getDouble(0.0));
        SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
        SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
        SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));
    }
}
