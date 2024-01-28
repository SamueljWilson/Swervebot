// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  MedianFilter xFilter = new MedianFilter(5);
  MedianFilter yFilter = new MedianFilter(5);
  double filteredX = 0.0;
  double filteredY = 0.0;

  double limelightMountDegrees = 0;
  double limelightLensHeightInches = 10.5;
  double noteHeightInches = 2.0;

  public Limelight() {}

  @Override
  public void periodic() {
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);

    filteredX = xFilter.calculate(x);
    filteredY = yFilter.calculate(y);

    SmartDashboard.putNumber("LimeLightX", filteredX);
    SmartDashboard.putNumber("LimeLightY", filteredY);
    SmartDashboard.putNumber("LimeLightArea", area);
    SmartDashboard.putNumber("LimelightV", v);
  }

  public double getX() {
    return filteredX;
  }

  public double getY() {
    return filteredY;
  }

  public double getDistance() {
    double noteOffsetAngle_Vertical = getY();
    double angleToNoteDegrees = limelightMountDegrees + noteOffsetAngle_Vertical;
    double angleToNoteRadians = angleToNoteDegrees * (3.14159 / 180.0);
    double distanceFromLightToNote = (noteHeightInches - limelightLensHeightInches) / Math.tan(angleToNoteRadians);

    SmartDashboard.putNumber("Distance to Note (in)", distanceFromLightToNote);
    return distanceFromLightToNote;
  }
}