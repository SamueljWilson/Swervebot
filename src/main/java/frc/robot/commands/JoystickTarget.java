package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class JoystickTarget extends Command {
  private DriveSubsystem m_drive;
  private Limelight m_limelight;
  private GenericHID m_driverController;
  private ProfiledPIDController thetaController = new ProfiledPIDController(
      SwerveModuleConstants.kPTurningController,
      SwerveModuleConstants.kITurningController,
      SwerveModuleConstants.kDTurningController,
      new TrapezoidProfile.Constraints(
        SwerveModuleConstants.kMaxAngularSpeedRadiansPerSecond,
        SwerveModuleConstants.kMaxAngularAccelerationRadiansPerSecondSquared));
  
  public JoystickTarget(DriveSubsystem drive, Limelight limelight, GenericHID driverController) {
    m_drive = drive;
    m_limelight = limelight;
    m_driverController = driverController;
    addRequirements(drive, limelight);
  }

  @Override
  public void initialize() {
    thetaController.reset(0);
    thetaController.setTolerance(VisionConstants.kTargetingTolerance);
  }

  @Override
  public void execute() {
    double x = m_limelight.getX();
    double xVelocity = thetaController.calculate(Units.degreesToRadians(x));
    double reverseFactor = -1.0;
    m_drive.drive(
      reverseFactor * joystickTransform(m_driverController.getRawAxis(OIConstants.kLeftJoyYAxis)) * OIConstants.kMaxMetersPerSec,
      reverseFactor * joystickTransform(m_driverController.getRawAxis(OIConstants.kLeftJoyXAxis)) * OIConstants.kMaxMetersPerSec,
      -joystickTransform(m_driverController.getRawAxis(OIConstants.kRightJoyXAxis)) + xVelocity,
      true
    );
  }

  private static double joystickTransform(double value) {
    double speedCoef = 1;
    double postDeadbandValue = MathUtil.applyDeadband(value, OIConstants.kJoystickDeadband);
    double postDeadbandValueSquared = postDeadbandValue * Math.abs(postDeadbandValue);
    return postDeadbandValueSquared * speedCoef;
  }
}
