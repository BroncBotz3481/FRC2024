// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.superstructure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final LEDSubsystem m_LED = new LEDSubsystem();
  public final Superstructure superstructure = new Superstructure(m_climber,
                                                                  m_feeder,
                                                                  m_intake,
                                                                  m_shooter,
                                                                  m_elevator,
                                                                  m_LED,
                                                                  SwerveSubsystem.getInstance());


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Constants.operatorController.y().whileTrue(superstructure.toState(SuperState.SCORE_AMP_SETUP));
    Constants.operatorController.x().whileTrue(superstructure.toState(SuperState.SCORE_STAGE_PROTECTED_SETUP));
    Constants.operatorController.b().whileTrue(superstructure.toState(SuperState.SCORE_SPEAKER_SETUP));
    Constants.operatorController.a().whileTrue(superstructure.toState(SuperState.SAFE));
    Constants.operatorController.rightBumper().whileTrue(superstructure.toState(superstructure.getShootState()));
    Constants.operatorController.leftBumper().whileTrue(superstructure.toState(SuperState.GROUND_INTAKE));
    Constants.operatorController.leftTrigger().whileTrue(superstructure.toState(SuperState.SOURCE_INTAKE));
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(1)) > 0.1)
            .whileTrue(m_elevator.runManual(Constants.operatorController::getLeftY));
    Constants.driverController.rightBumper().whileTrue(m_climber.setRightSpeed(-1));
    Constants.driverController.rightTrigger(0.1).whileTrue(m_climber.setRightSpeed(1));
    Constants.driverController.leftBumper().whileTrue(m_climber.setLeftSpeed(-1));
    Constants.driverController.leftTrigger(0.1).whileTrue(m_climber.setLeftSpeed(1));
    Constants.driverController.b().whileTrue(superstructure.toState(SuperState.CLIMB_REACH));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
