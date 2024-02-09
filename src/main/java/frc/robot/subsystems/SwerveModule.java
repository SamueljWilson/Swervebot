// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.PID;
import frc.robot.ValueCache;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private PID m_drivePid;
  private PID m_turningPid;

  private final SparkPIDController m_drivePidController;
  private final SparkPIDController m_turningPidController;

  private double m_idealVelocity = 0;

  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_absoluteRotationEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final Rotation2d m_absoluteRotationEncoderOffset;
  private Rotation2d m_turningEncoderOffset;

  private final ValueCache<Double> m_drivePositionCache;
  private final ValueCache<Double> m_driveVelocityCache;
  private final ValueCache<Double> m_absoluteRotationCache;
  private final ValueCache<Double> m_turningCache;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
    new ProfiledPIDController(
      SwerveModuleConstants.kTurningPID.p(),
      SwerveModuleConstants.kTurningPID.i(),
      SwerveModuleConstants.kTurningPID.d(),
      new TrapezoidProfile.Constraints(
        SwerveModuleConstants.kMaxAngularSpeedRadiansPerSecond,
        SwerveModuleConstants.kMaxAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveMotorReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      boolean turningEncoderReversed,
      Rotation2d encoderOffset) {
    
    m_drivePid = SwerveModuleConstants.kDrivePID;
    m_turningPid = SwerveModuleConstants.kTurningPID;
    
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setClosedLoopRampRate(SwerveModuleConstants.kDriveMotorRampRate);
    m_driveMotor.setInverted(driveMotorReversed);
    m_driveMotor.setSmartCurrentLimit(SwerveModuleConstants.kDriveMotorCurrentLimit);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDrivePositionConversionFactor);
    m_driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveVelocityConversionFactor);
    m_drivePositionCache = new ValueCache<Double>(m_driveEncoder::getPosition, SwerveModuleConstants.kValueCacheTtlMicroseconds);
    m_driveVelocityCache = new ValueCache<Double>(m_driveEncoder::getVelocity, SwerveModuleConstants.kValueCacheTtlMicroseconds);

    m_drivePidController = m_driveMotor.getPIDController();
    m_drivePidController.setFeedbackDevice(m_driveEncoder);
    m_drivePidController.setP(m_drivePid.p(), 0);
    m_drivePidController.setI(m_drivePid.i(), 0);
    m_drivePidController.setD(m_drivePid.d(), 0);
    m_drivePidController.setFF(0);

    m_absoluteRotationEncoderOffset = encoderOffset;

    m_absoluteRotationEncoder = new CANcoder(turningEncoderChannel);
    var turningEncoderConfigurator = m_absoluteRotationEncoder.getConfigurator();
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = turningEncoderReversed
      ? SensorDirectionValue.Clockwise_Positive
      : SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    turningEncoderConfigurator.apply(encoderConfig);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_absoluteRotationCache =
      new ValueCache<Double>(() -> {
        return m_absoluteRotationEncoder.getAbsolutePosition().getValue();
      }, SwerveModuleConstants.kValueCacheTtlMicroseconds);

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setClosedLoopRampRate(SwerveModuleConstants.kTurningMotorRampRate);
    m_turningMotor.setInverted(turningMotorReversed);
    m_turningMotor.setSmartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);

    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningPositionConversionFactor);
    m_turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningVelocityConversionFactor);
    m_turningCache = new ValueCache<Double>(m_turningEncoder::getPosition,
      SwerveModuleConstants.kValueCacheTtlMicroseconds);
    updateTurningEncoderOffset();

    m_turningPidController = m_turningMotor.getPIDController();
    m_turningPidController.setFeedbackDevice(m_turningEncoder);
    m_turningPidController.setP(m_turningPid.p(), 0);
    m_turningPidController.setI(m_turningPid.i(), 0);
    m_turningPidController.setD(m_turningPid.d(), 0);
    m_turningPidController.setFF(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      m_driveVelocityCache.get(),
      getRotation2d()
    );
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_drivePositionCache.get(),
      getRotation2d()
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
      SwerveModuleState.optimize(desiredState, getRotation2d());

    final double driveVelocityDesired = state.speedMetersPerSecond;
    
    m_idealVelocity = SwerveModuleConstants.kVelocityProfile.calculate(driveVelocityDesired, m_idealVelocity, Constants.kDt);
    
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        MathUtil.clamp(
          m_turningPIDController.calculate(getRotation2d().getRadians(),
          state.angle.getRadians()),
           -1, 
           1
          );

    m_drivePidController.setReference(m_idealVelocity, ControlType.kVelocity);

    m_turningMotor.set(turnOutput);
  }

  private Rotation2d getAbsoluteRotation2d() {
    double absolutePositionRotations = m_absoluteRotationCache.get();
    double absolutePositionRadians = Units.rotationsToRadians(absolutePositionRotations);
    return new Rotation2d(absolutePositionRadians).minus(m_absoluteRotationEncoderOffset);
  }

  public void updateTurningEncoderOffset() {
    Rotation2d absolute = getAbsoluteRotation2d();
    double relativeRadians = m_turningCache.get();
    m_turningEncoderOffset = Rotation2d.fromRadians(relativeRadians - absolute.getRadians());
  }

  private Rotation2d getRotation2d() {
    return new Rotation2d(m_turningCache.get()).minus(m_turningEncoderOffset);
  }
}