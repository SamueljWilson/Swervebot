// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
  SparkMaxLimitSwitch m_forwardLimitSwitch = m_armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  SparkMaxLimitSwitch m_reverseLimitSwitch = m_armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  WristSubsystem m_wrist;

  public enum InitState {
    UNINITIALIZED,
    INITIALIZED,
  }

  InitState m_initState = InitState.UNINITIALIZED;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(WristSubsystem wrist) {
    m_wrist = wrist;
  }

  private DoubleSolenoid.Value getWristPosition() {
    return m_wrist.getWristPosition();
  }

  public Command moveHome() {
    return runOnce(
      () -> {
        if (m_wrist.getWristPosition() == WristConstants.kWristExtended) {
          m_wrist.retractWrist();
        }
        m_armMotor.getPIDController().setReference(ArmInterp.cyclesToHeight(ArmConstants.kHomeHeight),
        ControlType.kPosition);
      }
    );
  }

  private Command moveToHeight(double height) {
    return runOnce(
      () -> {
        m_armMotor.getPIDController().setReference(ArmInterp.heightToCycles(height), ControlType.kPosition);
      }
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
        double height = ArmInterp.cyclesToHeight(getCycles());
        double velocity = ArmInterp.vheightToRPM(metersPerSecond, height);
        m_armMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
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
          m_armMotor.getPIDController().setReference(-0.5, ControlType.kCurrent);
        }
      )
      .andThen(() -> {}, this).until(m_forwardLimitSwitch::isPressed)
      .andThen(
        () -> {
          setCycles(0.0);
          m_initState = InitState.INITIALIZED;
        }
      );
  }

  @Override
  public void periodic() {
    if (m_initState != InitState.INITIALIZED) return;
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
