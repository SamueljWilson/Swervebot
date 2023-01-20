// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class SwerveModule {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  private final WPI_CANCoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(
        ModuleConstants.kPModuleDriveController,
        ModuleConstants.kIModuleDriveController,
        ModuleConstants.kDModuleDriveController
      );

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          ModuleConstants.kIModuleTurningController,
          ModuleConstants.kDModuleTurningController,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  private final double m_encoderOffset;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      double encoderOffset) {
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    m_turningEncoder = new WPI_CANCoder(turningEncoderChannel);

    // Set whether drive encoder should be reversed or not
    m_driveMotor.setInverted(driveEncoderReversed);

    // Set whether turning encoder should be reversed or not
    m_turningMotor.setInverted(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_encoderOffset = encoderOffset;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_driveMotor.getSelectedSensorVelocity()*ModuleConstants.kDriveEncoderDistancePerPulse,
      getPosR2d()
    );
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, getPosR2d());

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        MathUtil.clamp(m_drivePIDController.calculate(
          m_driveMotor.getSelectedSensorVelocity()*ModuleConstants.kDriveEncoderDistancePerPulse,
          state.speedMetersPerSecond),
          -1,
          1
        );

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        MathUtil.clamp(
          m_turningPIDController.calculate(getPosR2d().getRadians(),
          state.angle.getRadians()),
           -1, 
           1
          );

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0, 0, 10);
    m_turningEncoder.setPosition(0, 10);
  }

  private Rotation2d getPosR2d() {
  return Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition() - m_encoderOffset);
  }
}
