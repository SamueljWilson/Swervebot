// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final SendableChooser<Auto> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    m_driverController.getLeftY()*OIConstants.kMaxMetersPerSec,
                    m_driverController.getLeftX()*OIConstants.kMaxMetersPerSec,
                    m_driverController.getRightX()*OIConstants.kMaxRadPerSec,
                    true),
                m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_chooser.setDefaultOption("Red Outer", Auto.redOuterAuto(m_robotDrive));
    m_chooser.addOption("Red Inner", Auto.redInnerAuto(m_robotDrive));
    m_chooser.addOption("Red Middle", Auto.redMiddleAuto(m_robotDrive));

    m_chooser.addOption("Blue Outer", Auto.blueOuterAuto(m_robotDrive));
    m_chooser.addOption("Blue Inner", Auto.blueInnerAuto(m_robotDrive));
    m_chooser.addOption("Blue Middle", Auto.blueMiddleAuto(m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected().getCommand();
  }

  public Pose2d getAutonomousStartingPose() {
    return m_chooser.getSelected().getPose();
  }
}
