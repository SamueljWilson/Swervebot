// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

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
  private double m_pitch0; // Radians
  private double m_roll0; // Radians
  private Pose2d m_initialPose = new Pose2d();
  private CameraSubsystem m_cameraSystem;

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
  SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getPositions(),
        m_initialPose,
        DriveConstants.stateStdDeviations,
        DriveConstants.visionStdDeviations);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(CameraSubsystem cameraSystem) {
    m_cameraSystem = cameraSystem;
    m_gyro.enableBoardlevelYawReset(true);
    // We have to wait for the gyro to callibrate before we can reset the gyro
    while (m_gyro.isCalibrating()) {Thread.yield();}
    zeroGyro();

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::initOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          return false; // Return true to mirror to red alliance
        },
        this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getRotation2d(),
        getPositions()
        );
    
    ArrayList<Optional<EstimatedRobotPose>> photonRobotPoseList = m_cameraSystem.getFieldRelativePoseEstimators();
    photonRobotPoseList.forEach(robotPoseEstimator -> {
      if (!robotPoseEstimator.isEmpty()) {
        m_odometry.addVisionMeasurement(robotPoseEstimator.get().estimatedPose.toPose2d(), robotPoseEstimator.get().timestampSeconds);
      } 
    });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  private double getPitchUncorrected() {
    return Math.toRadians(-m_gyro.getRoll());
  }

  private double getRollUncorrected() {
    return Math.toRadians(m_gyro.getPitch());
  }

  public double getPitch() {
    // Corrections for how we mounted our RoboRIO
    return getPitchUncorrected() - m_pitch0;
  }

  public double getRoll() {
    // Corrections for how we mounted our RoboRIO
    return getRollUncorrected() - m_roll0;
  }

  public double getYaw() {
    // Corrections for how we mounted our RoboRIO
    return Math.toRadians(-m_gyro.getYaw());
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void lock() {
    SwerveModuleState desiredState0 = new SwerveModuleState(0.0, new Rotation2d(Math.PI/4));
    SwerveModuleState desiredState1 = new SwerveModuleState(0.0, new Rotation2d((3*Math.PI)/4));
    m_frontLeft.setDesiredState(desiredState0);
    m_rearRight.setDesiredState(desiredState0);
    m_frontRight.setDesiredState(desiredState1);
    m_rearLeft.setDesiredState(desiredState1);
  }

  public void initOdometry(Pose2d initialPose) {
    m_initialPose = initialPose;
    m_odometry.resetPosition(getRotation2d(), getPositions(), initialPose);
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
    ChassisSpeeds chassisSpeeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
      : new ChassisSpeeds(xSpeed, ySpeed, rot);
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.discretize(chassisSpeeds, Constants.kDt));

    setModuleStates(swerveModuleStates);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, Constants.kDt);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.SwerveModuleConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]); //Q1
    m_rearLeft.setDesiredState(desiredStates[1]); //Q2
    m_rearRight.setDesiredState(desiredStates[2]); //Q3
    m_frontRight.setDesiredState(desiredStates[3]); //Q4
  }
  
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_rearLeft.getState(),
      m_frontRight.getState(),
      m_rearRight.getState()
    };
    return states;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders(); //Q1
    m_rearLeft.resetEncoders(); //Q2
    m_rearRight.resetEncoders(); //Q3
    m_frontRight.resetEncoders(); //Q4
  }

  /** Zeroes the heading of the robot. */
  public void zeroGyro() {
    m_gyro.reset();
    // Calculates the the offsets we need to apply for the pitch and roll
    m_pitch0 = getPitchUncorrected();
    m_roll0 = getRollUncorrected();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(getYaw() + m_initialPose.getRotation().getRadians());
  }
}