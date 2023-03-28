// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.WaitUntilArmRetracted;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
  SparkMaxLimitSwitch m_forwardLimitSwitch = m_armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  SparkMaxLimitSwitch m_reverseLimitSwitch = m_armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  WristSubsystem m_wrist;

  public enum InitState {
    UNINITIALIZED,
    INITIALIZED,
  }

  private enum MotorState {
    DOWN,
    STATIONARY,
    UP
  }

  private MotorState m_motorState = MotorState.STATIONARY;
  private InitState m_initState = InitState.UNINITIALIZED;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(WristSubsystem wrist) {
    m_wrist = wrist;
    m_armMotor.setInverted(false);
    m_armMotor.getPIDController().setP(ArmConstants.kPosP, ArmConstants.kPosPIDSlot);
    m_armMotor.getPIDController().setI(ArmConstants.kPosI, ArmConstants.kPosPIDSlot);
    m_armMotor.getPIDController().setD(ArmConstants.kPosD, ArmConstants.kPosPIDSlot);
    m_armMotor.getPIDController().setFF(ArmConstants.kPosFF, ArmConstants.kPosPIDSlot);
    m_armMotor.getPIDController().setIZone(ArmConstants.kPosIz, ArmConstants.kPosPIDSlot);
    m_armMotor.getPIDController().setOutputRange(-ArmConstants.kMaxOutput, ArmConstants.kMaxOutput,
      ArmConstants.kPosPIDSlot);
    
    m_armMotor.getPIDController().setP(ArmConstants.kVelP, ArmConstants.kVelPIDSlot);
    m_armMotor.getPIDController().setI(ArmConstants.kVelI, ArmConstants.kVelPIDSlot);
    m_armMotor.getPIDController().setD(ArmConstants.kVelD, ArmConstants.kVelPIDSlot);
    m_armMotor.getPIDController().setFF(ArmConstants.kVelFF, ArmConstants.kVelPIDSlot);
    m_armMotor.getPIDController().setIZone(ArmConstants.kVelIz, ArmConstants.kVelPIDSlot);
    m_armMotor.getPIDController().setOutputRange(-ArmConstants.kMaxOutput, ArmConstants.kMaxOutput,
      ArmConstants.kVelPIDSlot);

    m_armMotor.setIdleMode(IdleMode.kBrake);
  }

  private DoubleSolenoid.Value getWristPosition() {
    return m_wrist.getWristPosition();
  }

  public Command moveHomeCommand() {
    return Commands.runOnce(
      () -> {
        m_wrist.retractWrist();
        m_armMotor.getPIDController().setReference(ArmInterp.cyclesToHeight(ArmConstants.kHomeHeight),
          ControlType.kPosition, ArmConstants.kPosPIDSlot);
        m_motorState = MotorState.DOWN;
      },
      this, m_wrist
    );
  }

  public void moveToHeight(double height) {
    double currentHeight = ArmInterp.cyclesToHeight(getCycles());
    m_armMotor.getPIDController().setReference(ArmInterp.heightToCycles(height), ControlType.kPosition,
      ArmConstants.kPosPIDSlot);
    if (currentHeight > height) {
      m_motorState = MotorState.DOWN;
    } else if (currentHeight == height) {
      m_motorState = MotorState.STATIONARY;
    } else {
      m_motorState = MotorState.UP;
    }
  }

  private Command moveToHeightCommand(double height) {
    return Commands.runOnce(
      () -> {
        moveToHeight(height);
      },
      this, m_wrist
    );
  }

  public Command moveToOffFloorCommand() {
    return moveToHeightCommand(ArmConstants.kOffFloorHeight);
  }

  public Command moveToMiddleCommand() {
    return moveToHeightCommand(ArmConstants.k2ndRowHeight);
  }

  public Command moveToTopCommand() {
    return moveToHeightCommand(ArmConstants.k3rdRowHeight);
  }

  public Command moveToHumanStationCommand() {
    return moveToHeightCommand(ArmConstants.kHumanStationHeight);
  }

  public Command moveVHeightCommand(double metersPerSecond) {
    return runOnce(
      () -> {
        if (metersPerSecond < 0 && getCycles() <= ArmConstants.kWristExtensionCycles 
            || metersPerSecond > 0 && getCycles() >= ArmConstants.kMaxExtensionCycles) {
          stopVHeight();
          return;
        }
        double height = ArmInterp.cyclesToHeight(getCycles());
        double velocity = ArmInterp.vheightToRPM(metersPerSecond, height);
        m_armMotor.getPIDController().setReference(velocity, ControlType.kVelocity, ArmConstants.kVelPIDSlot);
        if (metersPerSecond < 0) {
          m_motorState = MotorState.DOWN;
        } else {
          m_motorState = MotorState.UP;
        }
      }
    );
  }

  private void stopVHeight() {
    // stopMotor does not brake the motor. Use set to current position to cause braking instead
    // m_armMotor.stopMotor();
    m_armMotor.getPIDController().setReference(getCycles(), ControlType.kPosition, ArmConstants.kPosPIDSlot);
    m_motorState = MotorState.STATIONARY;
  }

  public Command stopVHeightCommand() {
    return runOnce(
      () -> {
        stopVHeight();
      }
    );
  }

  public void setCycles(double cycles) {
    m_armMotor.getEncoder().setPosition(cycles);
  }

  public double getCycles() {
    return m_armMotor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return m_armMotor.getEncoder().getVelocity();
  }

  public Command initCommand() {
    return 
      runOnce(
        () -> {
          m_armMotor.getPIDController().setReference(ArmConstants.KArmInitializeSpeed,
            ControlType.kVelocity, ArmConstants.kVelPIDSlot);
          // m_armMotor.getPIDController().setReference(-0.3, ControlType.kDutyCycle);
        }
      )
      .andThen(new WaitUntilArmRetracted(this))
      .andThen(
        () -> {
          m_armMotor.getPIDController().setReference(ArmConstants.kHomeCyclesOffset, ControlType.kPosition, 
            ArmConstants.kPosPIDSlot);
          setCycles(0.0);
          m_initState = InitState.INITIALIZED;
        }, this
      );
  }

  public boolean isReverseLimitSwitchPressed() {
    return m_reverseLimitSwitch.isPressed();
  }

  public boolean isInitialized() {
    return m_initState == InitState.INITIALIZED;
  }

  @Override
  public void periodic() {
    if (m_initState != InitState.INITIALIZED) return;
    double cycles = getCycles();
    SmartDashboard.putNumber("Cycles", cycles);
    SmartDashboard.putNumber("Height", ArmInterp.cyclesToHeight(cycles));
    double armPosition = getCycles();
    assert(ArmConstants.kWristRetractionCycles < ArmConstants.kWristExtensionCycles);
    if (armPosition <= ArmConstants.kWristRetractionCycles && m_motorState == MotorState.DOWN) {
      m_wrist.retractWrist();
      // System.out.println("Retrating in the wrist periodic");
    }
    else if (armPosition >= ArmConstants.kWristExtensionCycles && m_motorState == MotorState.UP) {
      m_wrist.extendWrist();
      // System.out.println("Extending in the wrist periodic");
    }
  }
}