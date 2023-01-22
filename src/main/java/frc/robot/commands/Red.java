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
public class Red {
  public static Command RedOuter(DriveSubsystem drive) {
    return (
      new DriveTrajectory(
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(),
          new Pose2d(5, 0, new Rotation2d(0)),
          AutoConstants.kDriveTrajectoryConfig),
        drive)
       .andThen(() -> drive.drive(0, 0, 0, true))
       );
    }
  
  public static Command RedMiddle(DriveSubsystem drive) {
    return (
      new DriveTrajectory(
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(),
          new Pose2d(0, 0, new Rotation2d(0)),
          AutoConstants.kDriveTrajectoryConfig),
        drive)
        .andThen(() -> drive.drive(0, 0, 0, true))
        );
      }
    
  public static Command RedInner(DriveSubsystem drive) {
    return (
      new DriveTrajectory(
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(),
          new Pose2d(0, 0, new Rotation2d(0)),
          AutoConstants.kDriveTrajectoryConfig),
        drive)
        .andThen(() -> drive.drive(0, 0, 0, true))
        );
      }
  }
