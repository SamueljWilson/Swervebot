// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {
  public enum State {
    OPEN,
    CLOSED_CUBE,
    CLOSED_CONE
  }

  State m_state = State.OPEN;
  DoubleSolenoid m_leftSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH,
    GripperConstants.kLeftSolenoidForwardChannel,
    GripperConstants.kLeftSolenoidBackwardChannel
  );

  DoubleSolenoid m_rightSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH,
    GripperConstants.kRightSolenoidForwardChannel,
    GripperConstants.kRightSolenoidBackwardChannel
  );

  /** Creates a new GrpperSubsystem. */
  public GripperSubsystem() {
    grabCone();
  }

  public Command openGrippers() {
    return runOnce(
      () -> {
        if (m_state != State.OPEN) {
          m_state = State.OPEN;
          m_leftSolenoid.set(Value.kForward);
          m_rightSolenoid.set(Value.kForward);
        }
      }
    );
  }

  public Command grabCone() {
    return runOnce(
      () -> {
        if (m_state != State.CLOSED_CONE) {
          m_state = State.CLOSED_CONE;
          m_leftSolenoid.set(Value.kReverse);
          m_rightSolenoid.set(Value.kReverse);
        }
      }
    );
  }

  public Command grabCube() {
    return runOnce(
      () -> {
        if (m_state != State.CLOSED_CUBE) {
          m_state = State.CLOSED_CUBE;
          m_leftSolenoid.set(Value.kForward);
          m_rightSolenoid.set(Value.kReverse);
        }
      }
    ); 
  }

  public State getState() {
    return m_state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}