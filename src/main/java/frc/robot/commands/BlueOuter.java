// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueOuter {
  public static Command autoBlueOuter(DriveSubsystem drive) {
    return (
      new DriveTrajectory(
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(1.91, 0.53, new Rotation2d(270)),
          List.of(),
          new Pose2d(5.789, 0.53, new Rotation2d(270)),
        AutoConstants.kDriveTrajectoryConfig),
        drive)
       .andThen(() -> drive.drive(0, 0, 0, true))
       );
    }
  }

/**
 * Full Trajectory looks like this:
 * new DriveTrajectory(
        TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
        AutoConstants.kDriveTrajectoryConfig),
       m_drive)
 */
