// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 1; //Q1
    public static final int kRearLeftDriveMotorPort = 2; //Q2
    public static final int kRearRightDriveMotorPort = 3; //Q3
    public static final int kFrontRightDriveMotorPort = 4; //Q4

    public static final int kFrontLeftTurningMotorPort = 5; //Q1
    public static final int kRearLeftTurningMotorPort = 6; //Q2
    public static final int kRearRightTurningMotorPort = 7; //Q3
    public static final int kFrontRightTurningMotorPort = 8; //Q4

    public static final int kFrontLeftTurningEncoderPort = 9; //Q1
    public static final int kRearLeftTurningEncoderPorts = 10; //Q2
    public static final int kRearRightTurningEncoderPorts = 11; //Q3
    public static final int kFrontRightTurningEncoderPorts = 12; //Q4

    public static final boolean kFrontLeftTurningMotorReversed = true; //Q1
    public static final boolean kRearLeftTurningMotorReversed = true; //Q2
    public static final boolean kRearRightTurningMotorReversed = true; //Q3
    public static final boolean kFrontRightTurningMotorReversed = true; //Q4

    public static final boolean kFrontLeftDriveReversed = true; //Q1
    public static final boolean kRearLeftDriveReversed = true; //Q2
    public static final boolean kRearRightDriveReversed = true; //Q3
    public static final boolean kFrontRightDriveReversed = true; //Q4

    public static final boolean kFrontLeftEncoderReversed = false; //Q1
    public static final boolean kRearLeftEncoderReversed = false; //Q2
    public static final boolean kRearRightEncoderReversed = false; //Q3
    public static final boolean kFrontRightEncoderReversed = false; //Q4

    public static final Rotation2d kFrontLeftEncoderOffset = new Rotation2d(Math.toRadians(95.186 + 180.0)); //Q1
    public static final Rotation2d kRearLeftEncoderOffset = new Rotation2d(Math.toRadians(11.426 + 180.0)); //Q2
    public static final Rotation2d kRearRightEncoderOffset = new Rotation2d(Math.toRadians(9.141 + 180.0)); //Q3
    public static final Rotation2d kFrontRightEncoderOffset = new Rotation2d(Math.toRadians(160.225 + 180.0)); //Q4

    public static final SupplyCurrentLimitConfiguration kSupplyCurrentLimit =
      new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5);

    public static final double kTrackWidth = 0.431;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.681;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //Q1
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //Q2
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //Q3
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2)); //Q4

    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 5;

    public static final double kGyroPitchOffset = -3.07;
    public static final double kGyroRollOffset = -0.47875;
  }

  public static final class GripperConstants {
    public static final int kLeftSolenoidForwardChannel = 1;
    public static final int kLeftSolenoidBackwardChannel = 2;

    public static final int kRightSolenoidForwardChannel = 3;
    public static final int kRightSolenoidBackwardChannel = 4;
  }

  public static final class ArmConstants {
    public static final int kArmMotorPort = 13;

    public static final double kHomeHeight = 0.0; //TODO: FIND REAL HEIGHT VALUES
    public static final double k1stRowHeight = 0.0; //TODO: FIND REAL HEIGHT VALUES
    public static final double kOffFloorHeight = 0.0; //TODO: FIND REAL HEIGHT VALUES
    public static final double k2ndRowHeight = 0.0; //TODO: FIND REAL HEIGHT VALUES
    public static final double k3rdRowHeight = 0.0; //TODO: FIND REAL HEIGHT VALUES

    public static final DoubleSolenoid.Value kWristExtended = Value.kForward;
    public static final DoubleSolenoid.Value kWristRetracted = Value.kReverse;

    public static final double kAdjustVelocity = 0.8;

    public static final int kWristSolenoidForwardChannel = 5;
    public static final int kWristSolenoidBackwardChannel = 6;
    // Extension > Retraction
    public static final int kWristExtensionCycles = 2000; //TODO: FIND REAL VALUES
    public static final int kWristRetractionCycles = 1500; //TODO: FIND REAL VALUES

    public static boolean kStubOut = true;
  }

  public static final class WristConstants {
    public static final int kWristSolenoidForwardChannel = 5;
    public static final int kWristSolenoidBackwardChannel = 6;

    public static final DoubleSolenoid.Value kWristExtended = Value.kForward;
    public static final DoubleSolenoid.Value kWristRetracted = Value.kReverse;
  }

  public static final class SwerveModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kDriveEncoderCPR = 2048.0;
    public static final double kAbsoluteEncoderCPR = 4096.0;
    public static final double kWheelDiameterMeters = 0.09525;

    public static final double kDriveGearRatio = 6.75;

    public static final double kDriveEncoderDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / (kDriveEncoderCPR * kDriveGearRatio);

    public static final double kPModuleTurningController = 0.25;
    public static final double kIModuleTurningController = 0.0;
    public static final double kDModuleTurningController = 0.0;

    public static final double kPModuleDriveController = 0.25;
    public static final double kIModuleDriveController = 0.0001;
    public static final double kDModuleDriveController = 7.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kMaxRadPerSec = SwerveModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond;
    public static final double kMaxMetersPerSec = DriveConstants.kMaxSpeedMetersPerSecond;
    
    public static final int kA = 1;
    public static final int kB = 2;
    public static final int kX = 3;
    public static final int kY = 4;
    public static final int kLeftBumper = 5;
    public static final int kRightBumper = 6;
    public static final int kBack = 7;
    public static final int kStart = 8;
    public static final int kLeftJoy = 9;
    public static final int kRightJoy = 10;

    public static int kLeftJoyXAxis = 0;
    public static int kLeftJoyYAxis = 1;
    public static int kLeftTriggerAxis = 2;
    public static int kRightTriggerAxis = 3;
    public static int kRightJoyXAxis = 4;
    public static int kRightJoyYAxis = 5;

    public static final int kDpadUp = 12; //TODO: FIND OUT THE REAL VALUES OF THIS TRIGGERS
    public static final int kDpadRight = 13; //TODO: FIND OUT THE REAL VALUES OF THIS TRIGGERS
    public static final int kDpadDown = 14; //TODO: FIND OUT THE REAL VALUES OF THIS TRIGGERS
    public static final int kDpadLeft = 15; //TODO: FIND OUT THE REAL VALUES OF THIS TRIGGERS

    public static final int kArmDown = kDpadDown;
    public static final int kArmUp = kDpadUp;
    public static final int kCubeAxis = kRightTriggerAxis;
    public static final int kConeAxis = kLeftTriggerAxis;
    public static final int kOpenButton = kLeftBumper;
    public static final int kExtendWristButton = kY;
    public static final int kRetractWristButton = kA;


    public static final int kHomeButton = 16; //TODO: ASSIGN BUTTON VALUES
    public static final int k1stRowButton = 16; //TODO: ASSIGN BUTTON VALUES
    public static final int kPickOffFloorButton = 16; //TODO: ASSIGN BUTTON VALUES
    public static final int k2ndRowButton = 16; //TODO: ASSIGN BUTTON VALUES
    public static final int k3rdRowButton = 16; //TODO: ASSIGN BUTTON VALUES
    public static final int kSlowButton = kRightBumper;

    public static final double kDebounceSeconds = 0.01;

    public static final double kJoystickDeadband = 0.05;
    public static final double kSlowCoef = 0.25;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final double kMaxSpeedMetersPerSecondBalancing = 0.25;
    public static final double kChargeAdjustingThreshold = Math.toRadians(5);
    public static final double kEngagedThreshold = Math.toRadians(2);
    public static double kEngagedTimeThreshold = 1.0;

    public static final TrajectoryConfig kDriveTrajectoryConfig =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}