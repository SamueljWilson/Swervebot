// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.constant.Constable;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Constants;

public class Auto {
  /** Creates a new Auto. */
  public enum Team {
    BLUE,
    RED
  }

  private static final double kRobotHalfLength = 0.515;
  // private static final double kRobotHalfWidth = 0.390;
  private static final double kGridMaxX = 1.42875;
  private static final double kGridMaxY = 5.49910;
  private static final double kPickupOffset = 0.2;
  private static final double kChargingStationMinEdgeX = 2.975;
  private static final double kChargingStationMinEdgeY = 1.51448;
  private static final double kChargingStationMaxEdgeX = 4.909;
  private static final double kChargingStationMaxEdgeY = 3.98463;
  // private static final double kEChargingStationX = 4.515;

  private static final double kStartingGridX = kGridMaxX + kRobotHalfLength + 0.700;
  private static final double kPlacingGridX = kGridMaxX + kRobotHalfLength;
  private static final double kBGridY = 1.074;
  private static final double kHGridY = kGridMaxY - kBGridY;
  private static final double kEGridY = kGridMaxY / 2;

  private static final Pose2d kStartingGridB = new Pose2d(kStartingGridX, kBGridY, new Rotation2d(Math.PI));
  private static final Pose2d kPlacingGridB = new Pose2d(kPlacingGridX, kBGridY, new Rotation2d(Math.PI));

  private static final Pose2d kStartingGridE = new Pose2d(kStartingGridX, kEGridY, new Rotation2d(Math.PI));
  private static final Pose2d kPlacingGridE = new Pose2d(kPlacingGridX, kEGridY, new Rotation2d(Math.PI));

  private static final Pose2d kStartingGridH = new Pose2d(kStartingGridX, kHGridY, new Rotation2d(Math.PI));
  private static final Pose2d kPlacingGridH = new Pose2d(kPlacingGridX, kHGridY, new Rotation2d(Math.PI));

  private static final Translation2d kSM1 = new Translation2d(7.118, 0.921);
  // private static final Translation2d kSM2 = new Translation2d(7.118, 2.140);
  // private static final Translation2d kSM3 = new Translation2d(7.118, 3.359);
  private static final Translation2d kSM4 = new Translation2d(7.118, 4.578);

  private static final Pose2d kPickupStagingMarker1 = new Pose2d(
    kSM1.getX() - kRobotHalfLength - kPickupOffset,
    kSM1.getY(),
    new Rotation2d(Math.PI)
  );

  private static final Pose2d kPickupStagingMarker4 = new Pose2d(
    kSM4.getX() - kRobotHalfLength - kPickupOffset,
    kSM4.getY(),
    new Rotation2d(Math.PI)
  );

  private final Pose2d m_initialPose;
  private final Command m_command;
  private final Team m_team;

  private static Pose2d copyPose(Pose2d pose) {
    return new Pose2d (pose.getX(), pose.getY(), pose.getRotation());
  }
  
  private static Pose2d getFinalPose(Trajectory trajectory) {
    List<Trajectory.State> states = trajectory.getStates();
    Trajectory.State state = states.get(states.size()-1);
    return copyPose(state.poseMeters);
  }
  
  public Auto(Pose2d pose, Command command, Team team) {
    m_initialPose = pose;
    m_command = command;
    m_team = team;
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_initialPose;
  }

  public Team getTeam() {
    return m_team;
  }

  public static Auto doNothing(DriveSubsystem drive) {
    return new Auto(new Pose2d(0, 0, new Rotation2d(0)), Commands.runOnce(() -> {}), Team.BLUE);
  }

  private static interface MirrorInterface {
    public Pose2d apply(Pose2d pose);
  }

  private static MirrorInterface blueMirror = (Pose2d pose) -> {return copyPose(pose);};

  private static MirrorInterface redMirror = (Pose2d pose) -> {
    return new Pose2d(
      Constants.kMaxX - pose.getX(), pose.getY(),
      new Rotation2d(Math.PI - pose.getRotation().getRadians()));
  };

  public static Auto bPlaceCross(
      DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team, MirrorInterface mirror) {
    Trajectory trajectory0 = bTrajectoryPlaceCross0(mirror);
    Trajectory trajectory1 = bTrajectoryPlaceCross1(trajectory0, mirror);
    Trajectory trajectory2 = bTrajectoryPlaceCross2(trajectory1, mirror);
    Pose2d startingPose = copyPose(trajectory0.getInitialPose());
    Command command =
      MoveArmSync.moveToTop(arm)
      .andThen(DriveTrajectory.trajectoryCommandField(drive, trajectory0))
      .andThen(gripper.openGrippers())
      .andThen(DriveTrajectory.trajectoryCommandField(drive, trajectory1))
      .andThen(MoveArmSync.moveHome(arm))
      .andThen(DriveTrajectory.trajectoryCommandField(drive, trajectory2))
      .andThen(MoveArmSync.moveToOffFloor(arm));
    return new Auto(startingPose, command, team);
  }

  public static Auto blueBPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team) {
    return bPlaceCross(drive, arm, gripper, team, blueMirror);
  }

  public static Auto redBPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team) {
    return bPlaceCross(drive, arm, gripper, team, redMirror);
  }

  public static Auto ePlaceCross(
      DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team, MirrorInterface mirror) {
    Trajectory trajectory0 = eTrajectoryPlaceCross0(mirror);
    Trajectory trajectory1 = eTrajectoryPlaceCross1(trajectory0, mirror);
    Trajectory trajectory2 = eTrajectoryPlaceCross2(mirror);
    Pose2d startingPose = copyPose(trajectory0.getInitialPose());
    Command command =
      // Commands.runOnce(() -> SmartDashboard.putString("debug", "A"))
      MoveArmSync.moveToTop(arm)
      .andThen(DriveTrajectory.trajectoryCommandField(drive, trajectory0))
      .andThen(gripper.openGrippers())
      .andThen(DriveTrajectory.trajectoryCommandField(drive, trajectory1))
      .andThen(MoveArmSync.moveHome(arm))
      .andThen(new CrossCharger(team, drive))
      .andThen(DriveTrajectory.trajectoryCommandRobot(drive, trajectory2, team))
      .andThen(arm.moveToOffFloorCommand());
    return new Auto(startingPose, command, team);
  }
  
  public static Auto blueEPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team) {
    return ePlaceCross(drive, arm, gripper, team, blueMirror);
  }
   
  public static Auto redEPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team) {
    return ePlaceCross(drive, arm, gripper, team, redMirror);
  }

  public static Auto ePlaceCrossCharge(
      DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team, MirrorInterface mirror) {
    Trajectory trajectory0 = eTrajectoryPlaceCross0(mirror);
    Trajectory trajectory1 = eTrajectoryPlaceCross1(trajectory0, mirror);
    Trajectory trajectory2 = eTrajectoryPlaceCross2(mirror);
    Pose2d startingPose = copyPose(trajectory0.getInitialPose());
    Command command =
      new CrossCharger(team, drive) //TODO: REMOVE WHEN TESTED
      // arm.moveToTopCommand()
      // .andThen(DriveTrajectory.trajectoryCommand(drive, trajectory0, true))
      // .andThen(gripper.openGrippers())
      // .andThen(DriveTrajectory.trajectoryCommand(drive, trajectory1, true))
      // .andThen(arm.moveHome())
      // .andThen(new CrossCharger(team, drive))
      .andThen(DriveTrajectory.trajectoryCommandRobot(drive, trajectory2, team));
      // .andThen(new AutoBalance(team, drive));
      return new Auto(startingPose, command, team);
  }

  public static Auto blueEPlaceCrossCharge(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team) {
    return ePlaceCrossCharge(drive, arm, gripper, team, blueMirror);
  }

  public static Auto redEPlaceCrossCharge(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team) {
    return ePlaceCrossCharge(drive, arm, gripper, team, redMirror);
  }

  public static Auto hPlaceCross(
      DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team, MirrorInterface mirror) {
    Trajectory trajectory0 = hTrajectoryPlaceCross0(mirror);
    Trajectory trajectory1 = hTrajectoryPlaceCross1(trajectory0, mirror);
    Trajectory trajectory2 = hTrajectoryPlaceCross2(trajectory1, mirror);
    Pose2d startingPose = copyPose(trajectory0.getInitialPose());
    Command command =
      arm.moveToTopCommand()
      .andThen(DriveTrajectory.trajectoryCommandField(drive, trajectory0))
      .andThen(gripper.openGrippers())
      .andThen(DriveTrajectory.trajectoryCommandField(drive, trajectory1))
      .andThen(arm.moveHomeCommand())
      .andThen(DriveTrajectory.trajectoryCommandField(drive, trajectory2))
      .andThen(arm.moveToOffFloorCommand());
    return new Auto(startingPose, command, team);
  }

  public static Auto blueHPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team) {
    return hPlaceCross(drive, arm, gripper, team, blueMirror);
  }

  public static Auto redHPlaceCross(DriveSubsystem drive, ArmSubsystem arm, GripperSubsystem gripper, Team team) {
    return hPlaceCross(drive, arm, gripper, team, redMirror);
  }

  private static Trajectory bTrajectoryPlaceCross0(MirrorInterface mirror) {
    return DriveTrajectory.generateTrajectory(
      mirror.apply(kStartingGridB),
      List.of(),
      mirror.apply(kPlacingGridB));
  }

  private static Trajectory bTrajectoryPlaceCross1(Trajectory prevTrajectory, MirrorInterface mirror) {
    return DriveTrajectory.generateTrajectory(
      getFinalPose(prevTrajectory),
      List.of(),
      mirror.apply(kStartingGridB));
  }

  private static Trajectory bTrajectoryPlaceCross2(Trajectory prevTrajectory, MirrorInterface mirror) {
    return
        DriveTrajectory.generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(
          mirror.apply(new Pose2d(kChargingStationMinEdgeX, kChargingStationMinEdgeY / 2, new Rotation2d())).getTranslation(),
          mirror.apply(new Pose2d(kChargingStationMaxEdgeX, kChargingStationMinEdgeY / 2, new Rotation2d())).getTranslation()
        ),
        mirror.apply(kPickupStagingMarker1));
  }

  private static Trajectory eTrajectoryPlaceCross0(MirrorInterface mirror) {
    return
      DriveTrajectory.generateTrajectory(
        mirror.apply(kStartingGridE),
        List.of(),
        mirror.apply(kPlacingGridE));
  }

  private static Trajectory eTrajectoryPlaceCross1(Trajectory prevTrajectory, MirrorInterface mirror) {
    return
      DriveTrajectory.generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(),
        mirror.apply(kStartingGridE));
  }

  private static Trajectory eTrajectoryPlaceCross2(MirrorInterface mirror) {
    return
      DriveTrajectory.generateTrajectory(
        mirror.apply(new Pose2d(0, 0, new Rotation2d(Math.PI))),
        List.of(),
        mirror.apply(new Pose2d(0.5, 0, new Rotation2d(Math.PI))));
  }

  private static Trajectory hTrajectoryPlaceCross0(MirrorInterface mirror) {
    return
      DriveTrajectory.generateTrajectory(
        mirror.apply(kStartingGridH),
        List.of(),
        mirror.apply(kPlacingGridH));
  }

  private static Trajectory hTrajectoryPlaceCross1(Trajectory prevTrajectory, MirrorInterface mirror) {
    return
      DriveTrajectory.generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(),
        mirror.apply(kStartingGridH));
  }

  private static Trajectory hTrajectoryPlaceCross2(Trajectory prevTrajectory, MirrorInterface mirror) {
    return
      DriveTrajectory.generateTrajectory(
        getFinalPose(prevTrajectory),
        List.of(
          mirror.apply(new Pose2d(kChargingStationMinEdgeX, (kGridMaxY + kChargingStationMaxEdgeY) / 2, new Rotation2d())).getTranslation(),
          mirror.apply(new Pose2d(kChargingStationMaxEdgeX, (kGridMaxY + kChargingStationMaxEdgeY) / 2, new Rotation2d())).getTranslation()
        ),
        mirror.apply(kPickupStagingMarker4));
  }
}