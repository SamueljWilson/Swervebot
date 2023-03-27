// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
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
  public final WristSubsystem m_wrist = new WristSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem(m_wrist);
  private boolean m_ranInits = false;

  // The driver's controller
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  private final SendableChooser<Auto> m_chooser = new SendableChooser<>();

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
    // Configure the button bindings
    configureButtonBindings();
    configureAutoRoutines();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
        new RunCommand(
          () -> {
            double reverseFactor = getTeam() == Auto.Team.BLUE ? -1 : 1;
            m_robotDrive.drive(
              reverseFactor*joystickTransform(m_driverController.getRawAxis(OIConstants.kLeftJoyYAxis))*OIConstants.kMaxMetersPerSec,
              reverseFactor*joystickTransform(m_driverController.getRawAxis(OIConstants.kLeftJoyXAxis))*OIConstants.kMaxMetersPerSec,
              -joystickTransform(m_driverController.getRawAxis(OIConstants.kRightJoyXAxis))*OIConstants.kMaxRadPerSec,
              true);
          }, m_robotDrive
        )
      );
  }

  private void configureAutoRoutines() {
    m_chooser.setDefaultOption("Do Nothing", Auto.doNothing(m_robotDrive));
    m_chooser.addOption("Blue B Place Cross", Auto.blueBPlaceCross(m_robotDrive, m_arm, m_gripper, Auto.Team.BLUE));
    m_chooser.addOption("Blue E Place Cross", Auto.blueEPlaceCross(m_robotDrive, m_arm, m_gripper, Auto.Team.BLUE));
    m_chooser.addOption("Blue E Place Cross Charge", Auto.blueEPlaceCrossCharge(m_robotDrive, m_arm, m_gripper, Auto.Team.BLUE));
    m_chooser.addOption("Blue H Place Cross", Auto.blueHPlaceCross(m_robotDrive, m_arm, m_gripper, Auto.Team.BLUE));

    m_chooser.addOption("Red B Place Cross", Auto.redBPlaceCross(m_robotDrive, m_arm, m_gripper, Auto.Team.RED));
    m_chooser.addOption("Red E Place Cross", Auto.redEPlaceCross(m_robotDrive, m_arm, m_gripper, Auto.Team.RED));
    m_chooser.addOption("Red E Place Cross Charge", Auto.redEPlaceCrossCharge(m_robotDrive, m_arm, m_gripper, Auto.Team.RED));
    m_chooser.addOption("Red H Place Cross", Auto.redHPlaceCross(m_robotDrive, m_arm, m_gripper, Auto.Team.RED));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(()-> m_driverController.getRawAxis(OIConstants.kCloseAxis) >= 0.5)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(m_gripper.grab());
    new Trigger(()-> m_driverController.getRawAxis(OIConstants.kOpenAxis) >= 0.5)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(m_gripper.openGrippers());
    new Trigger(() -> {
          int povAngle = m_driverController.getPOV();
          return (povAngle == 315 || povAngle == 0 || povAngle == 45);
        }
      )
      .debounce(OIConstants.kDebounceSeconds)
      .whileActiveContinuous(m_arm.moveVHeight(OIConstants.kArmAdjustV))
      .onFalse(m_arm.stopVHeight());
    new Trigger(() -> {
          int povAngle = m_driverController.getPOV();
          return (povAngle == 225 || povAngle == 180 || povAngle == 135);
        }
      )
      .debounce(OIConstants.kDebounceSeconds)
      .whileActiveContinuous(m_arm.moveVHeight(-OIConstants.kArmAdjustV))
      .onFalse(m_arm.stopVHeight());
    new JoystickButton(m_driverController, OIConstants.kArmAdjustUpButton)
        .debounce(OIConstants.kDebounceSeconds)
        .whileActiveContinuous(m_arm.moveVHeight(OIConstants.kArmAdjustV))
        .onFalse(m_arm.stopVHeight());
    new JoystickButton(m_driverController, OIConstants.kArmAdjustDownButton)
        .debounce(OIConstants.kDebounceSeconds)
        .whileActiveContinuous(m_arm.moveVHeight(-OIConstants.kArmAdjustV))
        .onFalse(m_arm.stopVHeight());
    new JoystickButton(m_driverController, OIConstants.kSlowButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(Commands.runOnce(() -> {m_driveSpeed = DriveSpeed.SLOW;}))
      .onFalse(Commands.runOnce(() -> {m_driveSpeed = DriveSpeed.FAST;}));
    new JoystickButton(m_driverController, OIConstants.kHomeButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(m_arm.moveHome());
    new JoystickButton(m_driverController, OIConstants.kPickOffFloorButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(m_arm.moveToOffFloor());
    new JoystickButton(m_driverController, OIConstants.k2ndRowButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(m_arm.moveToMiddle());
    new JoystickButton(m_driverController, OIConstants.k3rdRowButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(m_arm.moveToTop());
    new JoystickButton(m_driverController, OIConstants.kHumanStationButton)
      .debounce(OIConstants.kDebounceSeconds)
      .onTrue(m_arm.moveToHumanStation());
  }

  public void initCommands() {
    if (!m_ranInits) {
      m_arm.initCommand().schedule();
      m_wrist.initCommand().schedule();
      m_ranInits = true;
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

  public Auto.Team getTeam() {
    return m_chooser.getSelected().getTeam();
  }
}