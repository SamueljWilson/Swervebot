// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class Auto {
/** Creates a new Auto. */
  DriveSubsystem m_drive;
  Pose2d m_pose;
  Command m_command;
  

  public Auto(DriveSubsystem drive, Pose2d pose, Command command) {
    m_drive = drive;
    m_pose = pose;
    m_command = command;
    drive.resetOdometry(pose);
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getPose() {
    return m_pose;
  }

  private static Transform2d redTransform() {
    return new Transform2d(
      new Pose2d(0, 0, new Rotation2d(90)), 
      new Pose2d(16.542, 0, new Rotation2d(270))
    );
  }

  public static Transform2d blueTransform() {
    return new Transform2d();
  }

  public static Auto redOuterAuto(DriveSubsystem drive) {
    Trajectory trajectory = outerTrajectory();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, command(drive, trajectory, transform));
  }

  public static Auto redMiddleAuto(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectory();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, command(drive, trajectory, transform));
  }
  
  public static Auto redInnerAuto(DriveSubsystem drive) {
    Trajectory trajectory = innerTrajectory();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, command(drive, trajectory, transform));
  }


  
  public static Auto blueOuterAuto( DriveSubsystem drive) {
    Trajectory trajectory = outerTrajectory();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, command(drive, trajectory, transform));
  }

  public static Auto blueMiddleAuto(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectory();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, command(drive, trajectory, transform));
  }

  public static Auto blueInnerAuto(DriveSubsystem drive) {
    Trajectory trajectory = innerTrajectory();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, command(drive, trajectory, transform));
  }

  private static Trajectory outerTrajectory() {
    return (
    TrajectoryGenerator.generateTrajectory(
          new Pose2d(1.91, 0.53, new Rotation2d(90)),
          List.of(),
          new Pose2d(5.789, 0.53, new Rotation2d(270)),
          AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory middleTrajectory() {
    return (
    TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(90)),
          List.of(),
          new Pose2d(0, 0, new Rotation2d(270)),
          AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory innerTrajectory() {
    return (
    TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(90)),
          List.of(),
          new Pose2d(0, 0, new Rotation2d(270)),
          AutoConstants.kDriveTrajectoryConfig)
    );
  }

  public static Command command(DriveSubsystem drive, Trajectory trajectory, Transform2d transform) {
    return (
      new DriveTrajectory(
        trajectory.transformBy(transform),
        drive)
        .andThen(() -> drive.drive(0, 0, 0, true))
        );
    }
}
