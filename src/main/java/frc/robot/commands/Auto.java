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
  private static final double kRobotHalfLength = 0.515;
  // private static final double kRobotHalfWidth = 0.390;
  private static final double kGridMaxX = 1.42875;
  private static final double kGridMaxY = 5.49910;
  private static final double kPickupOffset = 0.2;
  private static final double kChargingStationMinEdgeX = 2.975;
  private static final double kChargingStationMinEdgeY = 1.51448;
  private static final double kChargingStationMaxEdgeX = 4.909;
  private static final double kChargingStationMaxEdgeY = 3.98463;
  private static final double kECrossX = 6.074;
  private static final double kEChargingStationX = 4.515;

  private static final double kStartingGridX = kGridMaxX + kRobotHalfLength + 0.15 + 1;
  private static final double kPlacingGridX = kGridMaxX + kRobotHalfLength;
  private static final double kBGridY = 0.413;
  private static final double kHGridY = kGridMaxY - kBGridY;
  private static final double kEGridY = kGridMaxY / 2;

  private static final Pose2d kStartingGridB = new Pose2d(kStartingGridX, kBGridY, new Rotation2d(Math.PI));
  private static final Pose2d kPlacingGridB = new Pose2d(kPlacingGridX, kBGridY, new Rotation2d(Math.PI));

  private static final Pose2d kStartingGridE = new Pose2d(kStartingGridX, kEGridY, new Rotation2d(Math.PI));
  private static final Pose2d kPlacingGridE = new Pose2d(kPlacingGridX, kEGridY, new Rotation2d(Math.PI));
  private static final Pose2d kCrossChargingStationE = new Pose2d(kECrossX, kEGridY, new Rotation2d(Math.PI));
  private static final Pose2d kCenterChargingStationE = new Pose2d(kEChargingStationX, kEGridY, new Rotation2d(Math.PI));

  private static final Pose2d kStartingGridH = new Pose2d(kStartingGridX, kHGridY, new Rotation2d(Math.PI));
  private static final Pose2d kPlacingGridH = new Pose2d(kPlacingGridX, kHGridY, new Rotation2d(Math.PI));

  private static final Translation2d kSM1 = new Translation2d(5.690, 0.921);
  private static final Translation2d kSM2 = new Translation2d(5.690, 2.140);
  // private static final Translation2d kSM3 = new Translation2d(5.690, 3.359);
  private static final Translation2d kSM4 = new Translation2d(5.690, 4.578);

  private static final Pose2d kPickupStagingMarker1 = new Pose2d(
    kSM1.getX() - kRobotHalfLength - kPickupOffset,
    kSM1.getY(),
    new Rotation2d(0)
  );

  private static final Pose2d kPickupStagingMarker4 = new Pose2d(
    kSM4.getX() - kRobotHalfLength - kPickupOffset,
    kSM4.getY(),
    new Rotation2d(0)
  );

  private static final Pose2d kPickupStagingMarker2 = new Pose2d(
    kSM2.getX() - kRobotHalfLength - kPickupOffset,
    kSM2.getY(),
    new Rotation2d(0)
  );

  Pose2d m_pose;
  Command m_command;
  
  private static Command trajectoryCommand(DriveSubsystem drive, Trajectory trajectory, Transform2d transform) {
    return (
      new DriveTrajectory(
        trajectory.transformBy(transform),
        drive)
        .andThen(() -> drive.drive(0, 0, 0, true))
    );
  }

  private static Pose2d copyPose(Pose2d pose) {
    return new Pose2d (pose.getX(), pose.getY(), pose.getRotation());
  }
  
  private static Pose2d getFinalPose(Trajectory trajectory) {
    List<Trajectory.State> states = trajectory.getStates();
    Trajectory.State state = states.get(states.size()-1);
    return copyPose(state.poseMeters);
  }
  
  public Auto(DriveSubsystem drive, Pose2d pose, Command command) {
    m_pose = pose;
    m_command = command;
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_pose;
  }

  private static Transform2d redTransform() {
    return new Transform2d(
      new Pose2d(0, 0, new Rotation2d(90)), 
      new Pose2d(16.542, 0, new Rotation2d(270))
    );
  }

  private static Transform2d blueTransform() {
    return new Transform2d();
  }
  
  public static Auto bPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Transform2d transform) {
    Trajectory trajectory0 = bTrajectoryPlaceCross0();
    Trajectory trajectory1 = bTrajectoryPlaceCross1(trajectory0);
    Trajectory trajectory2 = bTrajectoryPlaceCross2(trajectory1);
    Pose2d startingPose = trajectory0.getInitialPose();
    Command command =
      arm.moveToTop()
      .andThen(trajectoryCommand(drive, trajectory0, transform))
      .andThen(gripper.openGrippers())
      .andThen(trajectoryCommand(drive, trajectory1, transform))
      .andThen(arm.moveHome())
      .andThen(trajectoryCommand(drive, trajectory2, transform))
      .andThen(arm.moveToOffFloor());
    return new Auto(drive, startingPose, command);
  }

  public static Auto blueBPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper) {
    return bPlaceCross(drive, arm, gripper, blueTransform());
  }

  public static Auto redBPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper) {
    return bPlaceCross(drive, arm, gripper, redTransform());
  }

  public static Auto ePlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Transform2d transform) {
    Trajectory trajectory0 = eTrajectoryPlaceCross0();
    Trajectory trajectory1 = eTrajectoryPlaceCross1(trajectory0);
    Trajectory trajectory2 = eTrajectoryPlaceCross2(trajectory1);
    Trajectory trajectory3 = eTrajectoryPlaceCross3(trajectory2);
    Pose2d startingPose = trajectory0.getInitialPose();
    Command command =
      arm.moveToTop()
      .andThen(trajectoryCommand(drive, trajectory0, transform))
      .andThen(gripper.openGrippers())
      .andThen(trajectoryCommand(drive, trajectory1, transform))
      .andThen(arm.moveHome())
      .andThen(trajectoryCommand(drive, trajectory2, transform))
      .andThen(trajectoryCommand(drive, trajectory3, transform))
      .andThen(arm.moveToOffFloor());
    return new Auto(drive, startingPose, command);
  }
  
  public static Auto blueEPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper) {
    return ePlaceCross(drive, arm, gripper, blueTransform());
  }
   
  public static Auto redEPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper) {
    return ePlaceCross(drive, arm, gripper, redTransform());
  }

  public static Auto ePlaceCrossCharge(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Transform2d transform) {
    Trajectory trajectory0 = eTrajectoryPlaceCross0();
    Trajectory trajectory1 = eTrajectoryPlaceCross1(trajectory0);
    Trajectory trajectory2 = eTrajectoryPlaceCross2(trajectory1);
    Trajectory trajectory3 = eTrajectoryPlaceCrossCharge3(trajectory2);
    Pose2d startingPose = trajectory0.getInitialPose();
    Command command =
      arm.moveToTop()
      .andThen(trajectoryCommand(drive, trajectory0, transform))
      .andThen(gripper.openGrippers())
      .andThen(trajectoryCommand(drive, trajectory1, transform))
      .andThen(arm.moveHome())
      .andThen(trajectoryCommand(drive, trajectory2, transform))
      .andThen(trajectoryCommand(drive, trajectory3, transform));
    return new Auto(drive, startingPose, command);
  }

  public static Auto blueEPlaceCrossCharge(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper) {
    return ePlaceCrossCharge(drive, arm, gripper, blueTransform());
  }

  public static Auto redEPlaceCrossCharge(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper) {
    return ePlaceCrossCharge(drive, arm, gripper, redTransform());
  }

  public static Auto hPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Transform2d transform) {
    Trajectory trajectory0 = hTrajectoryPlaceCross0();
    Trajectory trajectory1 = hTrajectoryPlaceCross1(trajectory0);
    Trajectory trajectory2 = hTrajectoryPlaceCross2(trajectory1);
    Pose2d startingPose = trajectory0.getInitialPose();
    Command command =
      arm.moveToTop()
      .andThen(trajectoryCommand(drive, trajectory0, transform))
      .andThen(gripper.openGrippers())
      .andThen(trajectoryCommand(drive, trajectory1, transform))
      .andThen(arm.moveHome())
      .andThen(trajectoryCommand(drive, trajectory2, transform))
      .andThen(arm.moveToOffFloor());
    return new Auto(drive, startingPose, command);
  }

  public static Auto blueHPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper) {
    return hPlaceCross(drive, arm, gripper, blueTransform());
  }

  public static Auto redHPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper) {
    return hPlaceCross(drive, arm, gripper, redTransform());
  }

  private static Rotation2d orientation(Translation2d fr, Translation2d to) {
    // Compute the angle of the fr->to vector.
    Translation2d orientationVector = to.minus(fr);
    return new Rotation2d(orientationVector.getX(), orientationVector.getY());
  }

  private static Trajectory generateTrajectory(Pose2d start, List<Translation2d> waypoints, Pose2d end) {
    Translation2d next = waypoints.isEmpty() ? end.getTranslation() : waypoints.get(0);
    Rotation2d startOrientation = orientation(start.getTranslation(), next);

    Translation2d prev = waypoints.isEmpty() ? start.getTranslation() : waypoints.get(waypoints.size() - 1);
    Rotation2d endOrientation = orientation(prev, end.getTranslation());

    List<Trajectory.State> states = TrajectoryGenerator.generateTrajectory(
      new Pose2d(start.getTranslation(), startOrientation),
      waypoints,
      new Pose2d(end.getTranslation(), endOrientation),
      AutoConstants.kDriveTrajectoryConfig
    ).getStates();

    Trajectory.State startStateOrig = states.get(0);
    Trajectory.State startState = new Trajectory.State(
      startStateOrig.timeSeconds,
      startStateOrig.velocityMetersPerSecond,
      startStateOrig.accelerationMetersPerSecondSq,
      start,
      startStateOrig.curvatureRadPerMeter);
    states.set(0, startState);

    Trajectory.State endStateOrig = states.get(states.size() - 1);
    Trajectory.State endState = new Trajectory.State(
      endStateOrig.timeSeconds,
      endStateOrig.velocityMetersPerSecond,
      endStateOrig.accelerationMetersPerSecondSq,
      end,
      endStateOrig.curvatureRadPerMeter);
    states.set(states.size() - 1, endState);

    return new Trajectory(states);
  }

  private static Trajectory bTrajectoryPlaceCross0() {
    return generateTrajectory(
      kStartingGridB,
      List.of(),
      kPlacingGridB);
  }

  private static Trajectory bTrajectoryPlaceCross1(Trajectory prevTrajectory) {
    return generateTrajectory(
      getFinalPose(prevTrajectory),
      List.of(),
      kStartingGridB);
  }

  private static Trajectory bTrajectoryPlaceCross2(Trajectory prevTrajectory) {
    return
        generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(
          new Translation2d(kChargingStationMinEdgeX, kChargingStationMinEdgeY / 2),
          new Translation2d(kChargingStationMaxEdgeX, kChargingStationMinEdgeY / 2)
        ),
        kPickupStagingMarker1);
  }

  private static Trajectory eTrajectoryPlaceCross0() {
    return
      generateTrajectory(
        kStartingGridE,
        List.of(),
        kPlacingGridE);
  }

  private static Trajectory eTrajectoryPlaceCross1(Trajectory prevTrajectory) {
    return
      generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(),
        kStartingGridE);
  }

  private static Trajectory eTrajectoryPlaceCross2(Trajectory prevTrajectory) {
    return
      generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(),
        kCrossChargingStationE);
  }

  private static Trajectory eTrajectoryPlaceCross3(Trajectory prevTrajectory) {
    return
      generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(),
        kPickupStagingMarker2);
  }

  private static Trajectory eTrajectoryPlaceCrossCharge3(Trajectory prevTrajectory) {
    return
      generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(),
        kCenterChargingStationE);
  }

  private static Trajectory hTrajectoryPlaceCross0() {
    return
      generateTrajectory(
        kStartingGridH,
        List.of(),
        kPlacingGridH);
  }

  private static Trajectory hTrajectoryPlaceCross1(Trajectory prevTrajectory) {
    return
      generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(),
        kStartingGridH);
  }

  private static Trajectory hTrajectoryPlaceCross2(Trajectory prevTrajectory) {
    return
      generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(
          new Translation2d(kChargingStationMinEdgeX, (kGridMaxY + kChargingStationMaxEdgeY) / 2),
          new Translation2d(kChargingStationMaxEdgeX, (kGridMaxY + kChargingStationMaxEdgeY) / 2)
        ),
        kPickupStagingMarker4);
  }
}