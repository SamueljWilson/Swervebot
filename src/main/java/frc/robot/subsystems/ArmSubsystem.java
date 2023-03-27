// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.concurrent.atomic.AtomicBoolean;

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
  AtomicBoolean m_vHeightInUse = new AtomicBoolean();

  public enum InitState {
    UNINITIALIZED,
    INITIALIZED,
  }

  InitState m_initState = InitState.UNINITIALIZED;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(WristSubsystem wrist) {
    m_wrist = wrist;
    m_armMotor.setInverted(false);
    m_armMotor.getPIDController().setP(ArmConstants.kP);
    m_armMotor.getPIDController().setI(ArmConstants.kI);
    m_armMotor.getPIDController().setD(ArmConstants.kD);
    m_armMotor.getPIDController().setOutputRange(-ArmConstants.kMaxOutput, ArmConstants.kMaxOutput);
    m_armMotor.setIdleMode(IdleMode.kBrake);
  }

  private DoubleSolenoid.Value getWristPosition() {
    return m_wrist.getWristPosition();
  }

  public Command moveHome() {
    return Commands.runOnce(
      () -> {
        if (m_wrist.getWristPosition() == WristConstants.kWristExtended) {
          m_wrist.retractWristCommand();
        }
        m_armMotor.getPIDController().setReference(ArmInterp.cyclesToHeight(ArmConstants.kHomeHeight),
        ControlType.kPosition);
      },
      this, m_wrist
    );
  }

  private Command moveToHeight(double height) {
    return Commands.runOnce(
      () -> {
        m_armMotor.getPIDController().setReference(ArmInterp.heightToCycles(height), ControlType.kPosition);
        System.out.printf("Move To Height: %f\n", height);
      },
      this, m_wrist
    );
  }

  public Command moveToOffFloor() {
    return moveToHeight(ArmConstants.kOffFloorHeight);
  }

  public Command moveToMiddle() {
    return moveToHeight(ArmConstants.k2ndRowHeight);
  }

  public Command moveToTop() {
    return moveToHeight(ArmConstants.k3rdRowHeight);
  }

  public Command moveToHumanStation() {
    return moveToHeight(ArmConstants.kHumanStationHeight);
  }

  public Command moveVHeight(double metersPerSecond) {
    return runOnce(
      () -> {
        if (m_vHeightInUse.compareAndSet(false, true)) {
          double height = ArmInterp.cyclesToHeight(getCycles());
          double velocity = ArmInterp.vheightToRPM(metersPerSecond, height);
          SmartDashboard.putNumber("Arm Velocity", velocity);
          System.out.printf("MoveVHeight %f\n", metersPerSecond);
          m_armMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
          m_vHeightInUse.set(false);
        }
      }
    );
  }

  public Command stopVHeight() {
    return runOnce(
      () -> {
        m_armMotor.getPIDController().setReference(0.0, ControlType.kVelocity);
        System.out.printf("Stopping\n");
      }
    );
  }

  public void setCycles(double cycles) {
    m_armMotor.getEncoder().setPosition(cycles);
  }

  public double getCycles() {
    return m_armMotor.getEncoder().getPosition();
  }

  public Command initCommand() {
    if (ArmConstants.kArmStubOut) {
      return Commands.none();
    }
    return 
      runOnce(
        () -> {
          m_armMotor.getPIDController().setReference(-1095, ControlType.kVelocity); // TODO: ADD A CONSTANT FOR THE VELOCITY
        }
      )
      .andThen(new WaitUntilArmRetracted(this))
      .andThen(
        () -> {
          // m_armMotor.getPIDController().setReference(0.0, ControlType.kCurrent);
          m_armMotor.getPIDController().setReference(ArmConstants.kHomeCyclesOffset, ControlType.kPosition);
          setCycles(0.0);
          m_initState = InitState.INITIALIZED;
        }, this
      );
  }

  public boolean isReverseLimitSwitchPressed() {
    return m_reverseLimitSwitch.isPressed();
  }

  @Override
  public void periodic() {
    if (m_initState != InitState.INITIALIZED) return;
    double cycles = getCycles();
    SmartDashboard.putNumber("Cycles", cycles);
    SmartDashboard.putNumber("Height", ArmInterp.cyclesToHeight(cycles));
    DoubleSolenoid.Value wristPosition = getWristPosition();
    double armPosition = getCycles();
    assert(ArmConstants.kWristRetractionCycles < ArmConstants.kWristExtensionCycles);
    if (wristPosition == WristConstants.kWristExtended && armPosition <= ArmConstants.kWristRetractionCycles) {
      m_wrist.retractWrist();
    }
    else if (wristPosition == WristConstants.kWristRetracted && armPosition >= ArmConstants.kWristExtensionCycles) {
      m_wrist.extendWrist();
    }
  }
}