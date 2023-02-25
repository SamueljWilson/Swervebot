// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTrajectory extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;

  private Rotation2d m_initialRotation = new Rotation2d();
  private Pose2d m_finalPose = new Pose2d();
  private boolean m_finished = false;

  // private DriveSubsystem m_drive;

  // Normalize to [0..2*pi).
  private static final Rotation2d normalizeRotation(Rotation2d rotation) {
    double radians = MathUtil.inputModulus(
      MathUtil.angleModulus(
        rotation.getRadians()
      ),
      0.0,
      2.0*Math.PI
    );
    return new Rotation2d(radians);
  }

  /**
   * Constructs a new DriveTrajectory that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param controller The HolonomicDriveController for the drivetrain.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param requirements The subsystems to require.
   */
  public DriveTrajectory(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      HolonomicDriveController controller,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    m_trajectory = trajectory;
    m_pose = pose;
    m_kinematics = kinematics;
    m_controller = controller;
    m_outputModuleStates = outputModuleStates;

    addRequirements(requirements);
  }

  private final Rotation2d getDesiredRotation() {
    double timeElapsed = m_timer.get();
    double timeTotal = m_trajectory.getTotalTimeSeconds();
    return m_initialRotation.interpolate(m_finalPose.getRotation(), timeElapsed / timeTotal);
  }

  private static final ProfiledPIDController initTheta() {
    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      return thetaController;
  }
  private static final HolonomicDriveController initDriveController() {
    return new HolonomicDriveController(
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      initTheta()
    );
  }
  /** Creates a new DriveTrajectory. */
  public DriveTrajectory(Trajectory trajectory, DriveSubsystem drive) {
    this(
      trajectory,
      drive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      initDriveController(),
      drive::setModuleStates,
      drive);

    // m_drive = drive;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_initialRotation = normalizeRotation(m_pose.get().getRotation());
    Pose2d finalPose = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1).poseMeters;
    Rotation2d finalRotation = normalizeRotation(finalPose.getRotation());
    Rotation2d rotationDelta = normalizeRotation(finalRotation.minus(m_initialRotation));
    // Minimize rotation angle by choosing which direction to rotate.
    // Positive rotation is left (counter-clockwise), i.e increasing radians.
    boolean rotateRight = (rotationDelta.getRadians() > Math.PI);
    Rotation2d finalRotationOptimized = rotateRight ? finalRotation.minus(Rotation2d.fromRadians(2*Math.PI)) : finalRotation;
    m_finalPose = new Pose2d(finalPose.getTranslation(), finalRotationOptimized);

    m_timer.restart();
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_pose.get();
    double curTime = m_timer.get();
    Trajectory.State sampledState = m_trajectory.sample(curTime);
    Rotation2d desiredRotation = getDesiredRotation();

    // Overriding the current rotation with our desired rotation
    Trajectory.State desiredState = new Trajectory.State(
      sampledState.timeSeconds,
      sampledState.velocityMetersPerSecond,
      sampledState.accelerationMetersPerSecondSq,
      new Pose2d(sampledState.poseMeters.getX(), sampledState.poseMeters.getY(), desiredRotation),
      sampledState.curvatureRadPerMeter);

    ChassisSpeeds targetChassisSpeeds =
        m_controller.calculate(currentPose, desiredState, desiredRotation);
    SwerveModuleState[] targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);

    // System.out.println("===");
    // System.out.printf("DriveTrajectory.execute() current/total time: %f/%f\n", curTime, m_trajectory.getTotalTimeSeconds());
    // System.out.printf("DriveTrajectory.execute() currentPose: %s\n", currentPose);
    // SwerveModuleState[] currentModuleStates = m_drive.getStates();
    // for (int i = 0; i < currentModuleStates.length; i++) {
    //   System.out.printf("DriveTrajectory.execute() currentModuleStates[%d]: %s\n", i, currentModuleStates[i]);
    // }
    // System.out.printf("DriveTrajectory.execute() desiredState: %s\n", desiredState);
    // System.out.printf("DriveTrajectory.execute() desiredRotation: %s\n", desiredRotation);
    // System.out.printf("DriveTrajectory.execute() targetChassisSpeeds: %s\n", targetChassisSpeeds);
    // for (int i = 0; i < targetModuleStates.length; i++) {
    //   System.out.printf("DriveTrajectory.execute() targetModuleStates[%d]: %s\n", i, targetModuleStates[i]);
    // }
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    if (m_finished) {
      return true;
    }
    if (!m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds())) {
      return false;
    }
    // Pose2d currentPose = m_pose.get();
    // Rotation2d rotationDelta = normalizeRotation(m_finalPose.getRotation().minus(currentPose.getRotation()));
    // if (Math.abs(MathUtil.inputModulus(rotationDelta.getDegrees(), -180, 180)) > 5/* XXX Magic number. */) {
    //   System.out.printf("rotationDelta: %s\n", rotationDelta);
    //   return false;
    // }
    // double distance = currentPose.getTranslation().getDistance(m_finalPose.getTranslation());
    // if (distance > 0.025 /* XXX Magic number. */) {
    //   System.out.printf("distance: %f\n", distance);
    //   return false;
    // }
    m_finished = true;
    return true;
  }
}