// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmInterp;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmSync extends Command {
  ArmSubsystem m_arm;
  double m_height;
  boolean m_isGoingHome = false;

  /** Creates a new MoveArmSync. */
  public MoveArmSync(ArmSubsystem arm, double height) {
    m_arm = arm;
    m_height = height;
    addRequirements(arm);
  }

  public static Command moveToTop(ArmSubsystem arm) {
    return new MoveArmSync(arm, ArmConstants.k3rdRowHeight);
  }

  public static Command moveToMiddle(ArmSubsystem arm) {
    return new MoveArmSync(arm, ArmConstants.k2ndRowHeight);
  }

  public static Command moveToOffFloor(ArmSubsystem arm) {
    return new MoveArmSync(arm, ArmConstants.kOffFloorHeight);
  }

  public static Command moveHome(ArmSubsystem arm) {
    return new MoveArmSync(arm, ArmConstants.kHomeHeight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Calling init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_height == ArmConstants.kHomeHeight) {
      m_arm.moveHome();
    } else {
      m_arm.moveToHeight(m_height);
    }
    // System.out.println("Executing");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double height = ArmInterp.cyclesToHeight(m_arm.getCycles());
    double lowerBound = m_height - ArmConstants.kEpsilonHeight;
    double upperBound = m_height + ArmConstants.kEpsilonHeight;
    return height >= lowerBound && height <= upperBound;
  }
}
