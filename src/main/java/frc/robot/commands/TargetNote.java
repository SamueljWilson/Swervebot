package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Relay;

public class TargetNote extends Command {
  private DriveSubsystem m_drive;
  private Limelight m_limelight;
  private Relay m_relay = new Relay(VisionConstants.kRelayPort);
  private ProfiledPIDController thetaController = new ProfiledPIDController(
      SwerveModuleConstants.kPTurningController,
      SwerveModuleConstants.kITurningController,
      SwerveModuleConstants.kDTurningController,
      new TrapezoidProfile.Constraints(
        SwerveModuleConstants.kMaxAngularSpeedRadiansPerSecond,
        SwerveModuleConstants.kMaxAngularAccelerationRadiansPerSecondSquared));

  public TargetNote (DriveSubsystem drive, Limelight limelight) {
    m_drive = drive;
    m_limelight = limelight;
    addRequirements(drive, limelight);
  }

  @Override
  public void initialize() {
    thetaController.reset(0);
    thetaController.setTolerance(VisionConstants.kTargetingTolerance);
    m_relay.set(Relay.Value.kOn);
  }

  @Override
  public void execute() {
    double x = m_limelight.getX();
    double xVelocity = thetaController.calculate(Units.degreesToRadians(x)) * VisionConstants.kTargetingTolerance;
    m_drive.drive(0, 0, xVelocity, false);
  }

  @Override
  public boolean isFinished() {
    return thetaController.atGoal();
  }

  @Override
  public void end(boolean isInterrupted) {
    m_relay.set(Relay.Value.kOff);
  }
}