// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import java.util.List;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;

public class DriveTrajectory extends SwerveControllerCommand {
  DriveSubsystem m_drive;
  ProfiledPIDController m_thetaController;
  Trajectory m_trajectory;
  boolean m_finished;

  private static final ProfiledPIDController initTheta() {
    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      return thetaController;
  }
  /** Creates a new DriveTrajectory. */
  public DriveTrajectory(Trajectory trajectory, DriveSubsystem drive) {
    super(
      trajectory,
      drive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      initTheta(),
      drive::setModuleStates,
      drive);
    
    m_drive = drive;
    m_trajectory = trajectory;
    m_finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        // Create config for trajectory

    // An example trajectory to follow.  All units in meters.

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(m_trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    m_finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
