// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.PickupCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TargetNote;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final Limelight m_limelight = new Limelight();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final CameraSubsystem m_cameraSystem = new CameraSubsystem(PhotonVisionConstants.kCameraName1, PhotonVisionConstants.kCameraName2);
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_cameraSystem);

  // The driver's controller
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);

  private enum DriveSpeed {
    FAST,
    SLOW
  }
  private static DriveSpeed m_driveSpeed = DriveSpeed.FAST;

  // Applies deadband and slows down the robot if the m_driveSpeed enum is set to SLOW
  private static double joystickTransform(double value) {
    double speedCoef;
    switch (m_driveSpeed) {
      case SLOW:
        speedCoef = OIConstants.kSlowCoef;
        break;
      default:
        speedCoef = 1.0;
    }
    double postDeadbandValue = MathUtil.applyDeadband(value, OIConstants.kJoystickDeadband);
    double postDeadbandValueSquared = postDeadbandValue * Math.abs(postDeadbandValue);
    return postDeadbandValueSquared*speedCoef;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("ShooterCommand", new ShooterCommand());
    NamedCommands.registerCommand("PickupCommand", new PickupCommand());
    // Configure the button bindings
    configureButtonBindings();
    configureAutoRoutines();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
        new RunCommand(
          () -> {
            // double reverseFactor = getTeam() == Auto.Team.BLUE ? -1 : 1;
            double reverseFactor = -1.0;
            m_robotDrive.drive(
              reverseFactor*joystickTransform(m_driverController.getRawAxis(OIConstants.kLeftJoyYAxis))*OIConstants.kMaxMetersPerSec,
              reverseFactor*joystickTransform(m_driverController.getRawAxis(OIConstants.kLeftJoyXAxis))*OIConstants.kMaxMetersPerSec,
              -joystickTransform(m_driverController.getRawAxis(OIConstants.kRightJoyXAxis))*OIConstants.kMaxRadPerSec,
              true);
          }, m_robotDrive
        )
      );

      m_chooser.setDefaultOption("TargetNote", new TargetNote(m_robotDrive, m_limelight));
      m_chooser.addOption("Two Note Path", new PathPlannerAuto("two note auto"));
      SmartDashboard.putData(m_chooser);
  }

  private void configureAutoRoutines() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {}
   

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}