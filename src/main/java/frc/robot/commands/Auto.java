// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class Auto {
/** Creates a new Auto. */
  Pose2d m_pose;
  Command m_command;
  
  public Auto(DriveSubsystem drive, Pose2d pose, Command command) {
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
 
  public static Auto redOuterCrossCharge(DriveSubsystem drive) {
    Trajectory trajectory = outerTrajectoryCrossCharge();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }
  
  // public static Auto redOuterPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper) {
  //   Trajectory trajectory = outerTrajectoryPlaceCross();
  //   Pose2d startingPose = trajectory.getInitialPose();
  //   Transform2d transform = redTransform();
  //   Command autoCommand = arm.moveToBottom()
  //     .andThen(gripper.openGrippers())
  //     .andThen(trajectoryCommand(drive, trajectory, transform));
  //   return new Auto(drive, startingPose, autoCommand);
  // }

  public static Auto redMiddleCrossCharge(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectoryCrossCharge();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto redMiddlePlaceCross(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectoryPlaceCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = redTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto redMiddlePlaceCrossCharge(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectoryPlaceCrossCharge();
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

  public static Auto redInnerCrossCharge(DriveSubsystem drive) {
    Trajectory trajectory = innerTrajectoryCrossCharge();
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
  
  public static Auto blueOuterCross( DriveSubsystem drive) {
    Trajectory trajectory = outerTrajectoryCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto blueOuterCrossCharge(DriveSubsystem drive) {
    Trajectory trajectory = outerTrajectoryCrossCharge();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }
  
  public static Auto blueOuterPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper) {
    Trajectory trajectory = outerTrajectoryPlaceCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    Command command =
      arm.moveToTop()
      .andThen(gripper.openGrippers())
      .andThen(trajectoryCommand(drive, trajectory, transform));
    return new Auto(drive, startingPose, command);
  }

  public static Auto blueMiddleCrossCharge(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectoryCrossCharge();
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

  public static Auto blueMiddlePlaceCrossCharge(DriveSubsystem drive) {
    Trajectory trajectory = middleTrajectoryPlaceCrossCharge();
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

  public static Auto blueInnerCrossCharge(DriveSubsystem drive) {
    Trajectory trajectory = innerTrajectoryCrossCharge();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  public static Auto blueInnerPlaceCross(DriveSubsystem drive) {
    Trajectory trajectory = innerTrajectoryPlaceCross();
    Pose2d startingPose = trajectory.getInitialPose();
    Transform2d transform = blueTransform();
    return new Auto(drive, startingPose, trajectoryCommand(drive, trajectory, transform));
  }

  private static Trajectory outerTrajectoryCross() {
    return (
    TrajectoryGenerator.generateTrajectory(
          new Pose2d(2.094, 0.530, new Rotation2d(0)),
          List.of(),
          new Pose2d(5.924, 0.530, new Rotation2d(0)),
          AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory outerTrajectoryCrossCharge() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(2.094, 0.530, new Rotation2d(0)),
        List.of(new Translation2d(5.924, 0.530), new Translation2d(5.924, 3.130)),
        new Pose2d(4.510, 3.130, new Rotation2d(0)),
        AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory outerTrajectoryPlaceCross() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(2.094, 0.413, new Rotation2d(Math.PI)),
        List.of(), //TODO: LIST OF MIDDLE TRAJECTORIES
        new Pose2d(5.924, 0, new Rotation2d(0)), //TODO: FAKE VALUES FOR TESTING CHANGE LATER
        AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory middleTrajectoryPlaceCross() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        List.of(), // Not implemented
        new Pose2d(0, 0, new Rotation2d(0)),
        AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory middleTrajectoryPlaceCrossCharge() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        List.of(), // Not implemented
        new Pose2d(0, 0, new Rotation2d(0)),
        AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory innerTrajectoryPlaceCross() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        List.of(), // Not implemented
        new Pose2d(0, 0, new Rotation2d(0)),
        AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory middleTrajectoryCrossCharge() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(2.094, 3.130, new Rotation2d(0)),
        List.of(new Translation2d(5.939, 3.130)),
        new Pose2d(4.510, 3.130, new Rotation2d(0)),
        AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory innerTrajectoryCrossCharge() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(2.094, 4.969, new Rotation2d(0)),
        List.of(new Translation2d(5.924, 4.969), new Translation2d(5.924, 3.130)),
        new Pose2d(4.510, 3.130, new Rotation2d(0)),
        AutoConstants.kDriveTrajectoryConfig)
    );
  }

  private static Trajectory innerTrajectoryCross() {
    return (
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(2.094, 4.969, new Rotation2d(0)),
        List.of(),
        new Pose2d(4.378, 4.969, new Rotation2d(0)),
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