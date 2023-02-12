// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft = //Q1
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPort,
          DriveConstants.kFrontLeftDriveReversed,
          DriveConstants.kFrontLeftTurningMotorReversed,
          DriveConstants.kFrontLeftEncoderReversed,
          DriveConstants.kFrontLeftEncoderOffset);

  private final SwerveModule m_rearLeft = //Q2
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningEncoderPorts,
          DriveConstants.kRearLeftDriveReversed,
          DriveConstants.kRearLeftTurningMotorReversed,
          DriveConstants.kRearLeftEncoderReversed,
          DriveConstants.kRearLeftEncoderOffset);

  private final SwerveModule m_rearRight = //Q3
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningEncoderPorts,
          DriveConstants.kRearRightDriveReversed,
          DriveConstants.kRearRightTurningMotorReversed,
          DriveConstants.kRearRightEncoderReversed,
          DriveConstants.kRearRightEncoderOffset);

private final SwerveModule m_frontRight = //Q4
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningEncoderPorts,
          DriveConstants.kFrontRightDriveReversed,
          DriveConstants.kFrontRightTurningMotorReversed,
          DriveConstants.kFrontRightEncoderReversed,
          DriveConstants.kFrontRightEncoderOffset);
  // The gyro sensor uses NavX
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = {
      m_frontLeft.getPosition(), //Q1
      m_rearLeft.getPosition(), //Q2
      m_rearRight.getPosition(), //Q3
      m_frontRight.getPosition() //Q4
    };
    return positions;
  }

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), getPositions());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.enableBoardlevelYawReset(true);
    while (m_gyro.isCalibrating()) {}
    zeroHeading();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getRotation2d(),
        getPositions()
        );
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), getPositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (right).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]); //Q1
    m_rearLeft.setDesiredState(swerveModuleStates[1]); //Q2
    m_rearRight.setDesiredState(swerveModuleStates[2]); //Q3
    m_frontRight.setDesiredState(swerveModuleStates[3]); //Q4
    SmartDashboard.putNumber("Front Left Desired State", (swerveModuleStates[0].angle.getDegrees() + 360) % 360); //Q1
    SmartDashboard.putNumber("Rear Left Desired State", (swerveModuleStates[1].angle.getDegrees() + 360) % 360); //Q2
    SmartDashboard.putNumber("Rear Right Desired State", (swerveModuleStates[2].angle.getDegrees() + 360) % 360); //Q3
    SmartDashboard.putNumber("Front Right Desired State", (swerveModuleStates[3].angle.getDegrees() + 360) % 360); //Q4
    dashboardPublish();
  }

  public void dashboardPublish() {
    SmartDashboard.putNumber("Front Left Actual State", m_frontLeft.getPosition().angle.getDegrees()); //Q1
    SmartDashboard.putNumber("Rear Left Actual State", m_rearLeft.getPosition().angle.getDegrees()); //Q2
    SmartDashboard.putNumber("Rear Right Actual State", m_rearRight.getPosition().angle.getDegrees()); //Q3
    SmartDashboard.putNumber("Front Right Actual State", m_frontRight.getPosition().angle.getDegrees()); //Q4
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]); //Q1
    m_rearLeft.setDesiredState(desiredStates[1]); //Q2
    m_rearRight.setDesiredState(desiredStates[2]); //Q3
    m_frontRight.setDesiredState(desiredStates[3]); //Q4
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders(); //Q1
    m_rearLeft.resetEncoders(); //Q2
    m_rearRight.resetEncoders(); //Q3
    m_frontRight.resetEncoders(); //Q4
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(-m_gyro.getYaw()));
  }
}
