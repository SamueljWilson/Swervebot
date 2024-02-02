package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    SmartDashboard.putNumber("Max Angular Speed", SmartDashboard.getNumber("Max Angular Speed", SwerveModuleConstants.kMaxAngularSpeedRadiansPerSecond));
    SmartDashboard.putNumber("Max Angular Acceleration", SmartDashboard.getNumber("Max Angular Acceleration", SwerveModuleConstants.kMaxAngularAccelerationRadiansPerSecondSquared));
    SmartDashboard.putNumber("kP", SmartDashboard.getNumber("kP", SwerveModuleConstants.kPTurningController));
    SmartDashboard.putNumber("kI", SmartDashboard.getNumber("kI", SwerveModuleConstants.kITurningController));
    SmartDashboard.putNumber("kD", SmartDashboard.getNumber("kD", SwerveModuleConstants.kDTurningController));
  }

  @Override
  public void execute() {
    updateConstants();
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

  private void updateConstants() {
    double maxAngularSpeed = SmartDashboard.getNumber("Max Angular Speed", SwerveModuleConstants.kMaxAngularSpeedRadiansPerSecond);
    double maxAngularAcceleration = SmartDashboard.getNumber("Max Angular Acceleration", SwerveModuleConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
    double currentMaxAngularSpeed = thetaController.getConstraints().maxVelocity;
    double currentMaxAngularAcceleration = thetaController.getConstraints().maxAcceleration;
    if (currentMaxAngularSpeed != maxAngularSpeed || currentMaxAngularAcceleration != maxAngularAcceleration) {
      System.out.println("updating constraints");
      thetaController.setConstraints(new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAcceleration));
    }

    double p = SmartDashboard.getNumber("kP", SwerveModuleConstants.kPTurningController);
    double i = SmartDashboard.getNumber("kI", SwerveModuleConstants.kITurningController);
    double d = SmartDashboard.getNumber("kD", SwerveModuleConstants.kDTurningController);
    if (thetaController.getP() != p || thetaController.getI() != i || thetaController.getD() != d) {
      System.out.println("updating pid");
      thetaController.setPID(p, i, d);
    }
  }
}
