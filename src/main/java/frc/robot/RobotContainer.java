// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Auto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final GripperSubsystem m_gripper = new GripperSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();
  public final WristSubsystem m_wrist = new WristSubsystem();

  private boolean armStubOut = true;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final SendableChooser<Auto> m_chooser = new SendableChooser<>();
  private static double joystickDeadband(double value) {
    return MathUtil.applyDeadband(value, OIConstants.kJoystickDeadband);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutoRoutines();
    m_arm.initCommand().schedule();
    m_wrist.initCommand().schedule();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
        new RunCommand(
          () -> {
            m_robotDrive.drive(
              joystickDeadband(m_driverController.getLeftY())*OIConstants.kMaxMetersPerSec,
              joystickDeadband(m_driverController.getLeftX())*OIConstants.kMaxMetersPerSec,
              joystickDeadband(m_driverController.getRightX())*OIConstants.kMaxRadPerSec,
              true);
          }, m_robotDrive
        )
      );
  }

  private void configureAutoRoutines() {
    if (armStubOut == false) {
      m_chooser.setDefaultOption("Blue B Place Cross", Auto.blueBPlaceCross(m_robotDrive, m_arm, m_gripper));
      m_chooser.addOption("Blue H Place Cross", Auto.blueHPlaceCross(m_robotDrive, m_arm, m_gripper));
      m_chooser.addOption("Blue E Place Cross", Auto.blueEPlaceCross(m_robotDrive, m_arm, m_gripper));
      m_chooser.addOption("Blue E Place Cross Charge", Auto.blueEPlaceCrossCharge(m_robotDrive, m_arm, m_gripper));

      m_chooser.addOption("Red B Place Cross", Auto.redBPlaceCross(m_robotDrive, m_arm, m_gripper));
      m_chooser.addOption("Red H Place Cross", Auto.redHPlaceCross(m_robotDrive, m_arm, m_gripper));
      m_chooser.addOption("Red E Place Cross", Auto.redEPlaceCross(m_robotDrive, m_arm, m_gripper));
      m_chooser.addOption("Red E Place Cross Charge", Auto.redEPlaceCrossCharge(m_robotDrive, m_arm, m_gripper));
    } else {
      m_chooser.setDefaultOption("Blue B Place Cross Wrist", Auto.blueBPlaceCrossWrist(m_robotDrive, m_wrist, m_gripper));
      m_chooser.addOption("Blue H Place Cross Wrist", Auto.blueHPlaceCrossWrist(m_robotDrive, m_wrist, m_gripper));
      m_chooser.addOption("Blue E Place Cross Wrist", Auto.blueEPlaceCrossWrist(m_robotDrive, m_wrist, m_gripper));

      m_chooser.addOption("Red B Place Cross Wrist", Auto.redBPlaceCrossWrist(m_robotDrive, m_wrist, m_gripper));
      m_chooser.addOption("Red H Place Cross Wrist", Auto.redHPlaceCrossWrist(m_robotDrive, m_wrist, m_gripper));
      m_chooser.addOption("Red E Place Cross Wrist", Auto.redEPlaceCrossWrist(m_robotDrive, m_wrist, m_gripper));
    }
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Creates the triggers for the cone and cube grab commands
    new JoystickButton(m_driverController, OIConstants.kCubeButtonPressed)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(m_gripper.grabCube());
    new JoystickButton(m_driverController, OIConstants.kConeButtonPressed)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(m_gripper.grabCone());
    new JoystickButton(m_driverController, OIConstants.kOpenButtonPressed)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(m_gripper.openGrippers());
    if (armStubOut == false) {
      new JoystickButton(m_driverController, OIConstants.kHomeButtonPressed)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(m_arm.moveHome());
      new JoystickButton(m_driverController, OIConstants.kPickOffFloorButtonPressed)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(m_arm.moveToOffFloor());
      new JoystickButton(m_driverController, OIConstants.k1stRowButtonPressed)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(m_arm.moveToBottom());
      new JoystickButton(m_driverController, OIConstants.k2ndRowButtonPressed)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(m_arm.moveToMiddle());
      new JoystickButton(m_driverController, OIConstants.k3rdRowButtonPressed)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(m_arm.moveToTop());
    } else {
      new JoystickButton(m_driverController, OIConstants.kExtendWristPressed)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(m_wrist.extendWrist());
      new JoystickButton(m_driverController, OIConstants.kRetractWristPressed)
        .debounce(OIConstants.kDebounceSeconds)
        .onTrue(m_wrist.retractWrist());
    }
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
    return m_chooser.getSelected().getInitialPose();
  }
}