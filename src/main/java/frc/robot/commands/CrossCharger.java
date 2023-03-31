// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Orientation3d;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Auto.Team;
import frc.robot.subsystems.DriveSubsystem;

public class CrossCharger extends CommandBase {
  /** Creates a new CrossCharger. */
  private enum State {
    FLAT,
    CLIMBING,
    CROSSING, 
    DESCENDING,
    ENDING
  }
  State m_state = State.FLAT;
  Team m_team;
  double m_mirrorFactor;

  DriveSubsystem m_drive;

  public CrossCharger(Team team, DriveSubsystem drive) {
    m_team = team;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_mirrorFactor = (m_team == Auto.Team.BLUE ? 1 : -1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Orientation3d orientation = new Orientation3d(m_drive.getPitch(), m_drive.getRoll(), m_drive.getYaw());
    switch (m_state) {
      case FLAT:
        m_drive.drive(AutoConstants.kMaxSpeedMetersPerSecond*m_mirrorFactor, 0, 0, true);
        if (orientation.getTilt() >= AutoConstants.kClimbingThreshold) {
          m_state = State.CLIMBING;
          break;
        }
        break;
      
      case CLIMBING:
        if (orientation.isTiltedUp()) {
          m_state = State.CROSSING;
          break;
        }
        m_drive.drive(AutoConstants.kMaxSpeedMetersPerSecond*m_mirrorFactor, 0, 0, true);
        break;
      
      case CROSSING:
        if (orientation.getTilt() >= AutoConstants.kDescendingThreshold) {
          m_state = State.DESCENDING;
          break;
        }
        m_drive.drive(AutoConstants.kMaxSpeedMetersPerSecond*m_mirrorFactor, 0, 0, true);
        break;
      
      case DESCENDING:
        if (orientation.getTilt() <= AutoConstants.kLevelThreshold) {
          m_state = State.ENDING;
          break;
        }
        m_drive.drive(AutoConstants.kMaxSpeedMetersPerSecond*m_mirrorFactor, 0, 0, true);
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