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
import edu.wpi.first.math.geometry.Translation2d;
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

  public static Auto redOuterCross(DriveSubsystem drive) {
    Trajectory trajectory = outerTrajectoryCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto redMiddleCross(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectoryCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }
  
  public static Auto redInnerCross(DriveSubsystem drive) {
    Trajectory trajectory = innerTrajectoryCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }
  
  public static Auto blueOuterCross( DriveSubsystem drive) {
    Trajectory trajectory = outerTrajectoryCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto blueMiddleCross(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectoryCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto blueInnerCross(DriveSubsystem drive) {
    Trajectory trajectory = innerTrajectoryCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto blueOuterPlaceCross(DriveSubsystem drive) {
    Trajectory trajectory = outerTrajectoryPlaceCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }
  
  public static Auto redOuterPlaceCross(DriveSubsystem drive) {
    Trajectory trajectory = outerTrajectoryPlaceCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

    public static Auto redInnerPlaceCross(DriveSubsystem drive) {
      Trajectory trajectory = innerTrajectoryPlaceCross();
      Pose2d startingPose = trajectory.getInitialPose();
      Transform2d transform = redTransform();
      return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto blueInnerPlaceCross(DriveSubsystem drive) {
    Trajectory trajectory = innerTrajectoryPlaceCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto blueMiddlePlaceCross(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectoryPlaceCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto redMiddlePlaceCross(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectoryPlaceCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  private static Trajectory outerTrajectoryCross() {
    return (
    TrajectoryGenerator.generateTrajectory(
          new Pose2d(1.908, 0.530, new Rotation2d(90)),
          List.of(),
          new Pose2d(5.789, 0.530, new Rotation2d(270)),
          AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory outerTrajectoryPlaceCross() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(90)),
        List.of(),
        new Pose2d(0, 0, new Rotation2d(270)),
        AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory middleTrajectoryPlaceCross() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(90)),
        List.of(),
        new Pose2d(0, 0, new Rotation2d(270)),
        AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory innerTrajectoryPlaceCross() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(90)),
        List.of(),
        new Pose2d(0, 0, new Rotation2d(270)),
        AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory middleTrajectoryCross() {
    return (
    TrajectoryGenerator.generateTrajectory(
          new Pose2d(1.908, 3.130, new Rotation2d(90)),
          List.of(new Translation2d(5.939, 3.130)),
          new Pose2d(4.375, 3.130, new Rotation2d(90)),
          AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory innerTrajectoryCross() {
    return (
    TrajectoryGenerator.generateTrajectory(
          new Pose2d(1.908, 4.969, new Rotation2d(90)),
          List.of(),
          new Pose2d(4.243, 4.969, new Rotation2d(270)),
          AutoConstants.kDriveTrajectoryConfig)
    );
  }

  public static Command trajectoryCommand(DriveSubsystem drive, Trajectory trajectory, Transform2d transform) {
    return (
      new DriveTrajectory(
        trajectory.transformBy(transform),
        drive)
        .andThen(() -> drive.drive(0, 0, 0, true))
        );
    }
}
