// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Orientation3d;
import frc.robot.commands.Auto.Team;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private enum State {
    STOPPED,
    DRIVING,
    ADJUSTING,
    ENGAGED
  }

  private State m_state = State.STOPPED;

  private PIDController m_balancePID = new PIDController(
      AutoConstants.kPBalanceController, 
      AutoConstants.kIBalanceController,
      AutoConstants.kDBalanceController);
  private boolean m_withinTolerance = false;
  private final Timer m_timer = new Timer();
  private final double m_mirrorFactor;

  DriveSubsystem m_drive;
  Team m_team;

  public AutoBalance(Team team, DriveSubsystem drive) {
    m_drive = drive;
    m_team = team;
    m_balancePID.setSetpoint(0);
    m_balancePID.setTolerance(AutoConstants.kLevelThreshold);
    m_mirrorFactor = (team == Auto.Team.BLUE ? 1 : -1);
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Orientation3d orientation = new Orientation3d(m_drive.getPitch(), m_drive.getRoll(), m_drive.getYaw());
    switch (m_state) {
      case STOPPED:
        m_drive.drive(-AutoConstants.kMaxSpeedMetersPerSecond*m_mirrorFactor, 0, 0, true);
        m_state = State.DRIVING;
        break;

      case DRIVING:
        m_drive.drive(-AutoConstants.kMaxSpeedMetersPerSecond*m_mirrorFactor, 0, 0, true);
        if (orientation.getTilt() >= AutoConstants.kChargeAdjustingThreshold) {
          m_state = State.ADJUSTING;
        }
        break;

      case ADJUSTING:
        boolean atSetpoint = m_balancePID.atSetpoint();
        if (m_withinTolerance) {
          if (atSetpoint) {
            if (m_timer.get() >= AutoConstants.kEngagedTimeThreshold) {
              m_state = State.ENGAGED;
              break;
            }
          } else {
            m_withinTolerance = false;
          }
        } else if (atSetpoint) {
          m_withinTolerance = true;
          m_timer.reset();
        }
        double tilt = orientation.getTilt() * (orientation.isTiltedUp() ? 1.0 : -1.0);
        double driveOutput = MathUtil.clamp(m_balancePID.calculate(-1.0 * tilt * m_mirrorFactor),
          -AutoConstants.kMaxSpeedMetersPerSecondBalancing,
          AutoConstants.kMaxSpeedMetersPerSecondBalancing);
        double finalDriveOutput = m_mirrorFactor * driveOutput;
        m_drive.drive(finalDriveOutput, 0, 0, true);
        break;

      case ENGAGED:
        m_drive.lock();
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
    return m_state == State.ENGAGED;
  }
}