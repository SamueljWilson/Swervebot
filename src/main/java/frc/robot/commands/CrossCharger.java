// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Orientation3d;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class CrossCharger extends CommandBase {
  /** Creates a new CrossCharger. */
  private enum State {
    FLAT,
    UPWARDS,
    ENGAGED,
    DOWNWARDS,
    ENDING
  }
  State m_state = State.FLAT;
  DriveSubsystem m_drive;

  public CrossCharger(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Orientation3d orientation = new Orientation3d(m_drive.getPitch(), m_drive.getRoll(), m_drive.getYaw());
    switch (m_state) {
      case FLAT:
        m_drive.drive(AutoConstants.kMaxSpeedMetersPerSecond, 0, 0, true);
        if (orientation.getTilt() >= AutoConstants.kChargeAdjustingThreshold) {
          m_state = State.UPWARDS;
          break;
        }
        break;
      
      case UPWARDS:
        if (orientation.getTilt() <= AutoConstants.kEngagedThreshold && orientation.getTilt() >= -AutoConstants.kEngagedThreshold) {
          m_state = State.ENGAGED;
          break;
        }
        m_drive.drive(AutoConstants.kMaxSpeedMetersPerSecond, 0, 0, true);
        break;
      
      case ENGAGED:
        if (orientation.getTilt() >= AutoConstants.kEngagedThreshold || orientation.getTilt() <= -AutoConstants.kEngagedThreshold) {
          m_state = State.DOWNWARDS;
          break;
        }
        m_drive.drive(AutoConstants.kMaxSpeedMetersPerSecond, 0, 0, true);
        break;
      
      case DOWNWARDS:
        if (orientation.getTilt() <= AutoConstants.kEngagedThreshold && orientation.getTilt() >= -AutoConstants.kEngagedThreshold) {
          m_state = State.ENDING;
          break;
        }
        break;
      
      case ENDING:
        break;
      
      default:
        assert(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_state == State.ENDING;
  }
}
