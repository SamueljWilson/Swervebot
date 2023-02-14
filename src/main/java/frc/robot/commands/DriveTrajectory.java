// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

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
}
