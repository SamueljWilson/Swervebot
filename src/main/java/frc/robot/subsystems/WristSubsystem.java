// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class WristSubsystem extends SubsystemBase {

  DoubleSolenoid m_wristPiston = ArmConstants.kStubOut ? null : new DoubleSolenoid(
    PneumaticsModuleType.REVPH,
     WristConstants.kWristSolenoidForwardChannel, 
     WristConstants.kWristSolenoidBackwardChannel);
  
  private DoubleSolenoid.Value getWristPosition() {
    if (ArmConstants.kStubOut) return DoubleSolenoid.Value.kOff;
    return m_wristPiston.get();
  }

  public Command extendWrist() {
    return runOnce(
      () -> {
        DoubleSolenoid.Value wristPosition = getWristPosition();
        if (wristPosition != WristConstants.kWristExtended) {
          m_wristPiston.set(WristConstants.kWristExtended);
        } 
      }
    );
  }

  public Command retractWrist() {
    return runOnce(
      () -> {
        if (ArmConstants.kStubOut) return;
        DoubleSolenoid.Value wristPosition = getWristPosition();
        if (wristPosition != WristConstants.kWristRetracted) {
          m_wristPiston.set(WristConstants.kWristRetracted);
        } 
      }
    );
  }

  public Command initCommand() {
    return runOnce(
      () -> {
        if (ArmConstants.kStubOut) return;
        m_wristPiston.set(WristConstants.kWristRetracted);
      }
    );
  }

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
