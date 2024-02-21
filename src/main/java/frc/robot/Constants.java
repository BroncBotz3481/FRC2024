// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // CONSTANT MEMBERS

  public static final CommandXboxController driverController =
          new CommandXboxController(DriveteamConstants.kDriverControllerPort);
  public static final CommandXboxController operatorController =
          new CommandXboxController(DriveteamConstants.kOperatorControllerPort);
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  // CONTROL CONSTANT CLASSES

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static class DriveteamConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kOperatorControllerPort = 1;

  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  // SUBSYSTEM CONSTANT CLASSES

  public static class ClimberConstants {
    public static final int leftClimberMotorID = 26;
    public static final int rightClimberMotorID = 27;
    public static final int lowerID = 5;

  }
  public static class FeederConstants {
    public static final int feederMotorID = 28;

    public static final int beamBrakeChannel = 3;

  }

  public static class IntakeConstants {
    public static final int intakeMotorLeftID = 24;
    public static final int intakeMotorRightID = 23;
    public static final int centerMotorID = 25;
    public static final int forwardChannelPort = 0;
    public static final int reverseChannelPort = 1;

  }
  public static class ShooterConstants {
    public static final int leftShooterID = 20;
    public static final int rightShooterID = 21;

    public static final int forwardChannelPort = 2;
    public static final int reverseChannelPort = 3;

  }

  public static class LEDConstants {
    public static final int port = 36;
    public static final int length = 27;
  }

  public static class ElevatorConstants {

    public static final int leftLiftID = 32;
    public static final int rightLiftID = 33;

    public static final int limitSwitchTop = 6;
    public static final int limitSwitchBottom = 7;

  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

}