// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import com.pathplanner.lib.auto.NamedCommands;

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

  private final SwerveSubsystem m_drivebase = SwerveSubsystem.getInstance();

  private final SendableChooser<Command> autoChooser;
  public final Superstructure superstructure = new Superstructure(m_climber,
                                                                  m_feeder,
                                                                  m_intake,
                                                                  m_shooter,
                                                                  m_elevator,
                                                                  m_LED,
                                                                  m_drivebase);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("Simple Auto");
    Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);
    configurePathPlanner();
    configureBindings(); // Configure the trigger bindings
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
//    Constants.operatorController.y().whileTrue(superstructure.toState(SuperState.SCORE_AMP_SETUP));
//    Constants.operatorController.x().whileTrue(superstructure.toState(SuperState.SCORE_STAGE_PROTECTED_SETUP));
//    Constants.operatorController.b().whileTrue(superstructure.toState(SuperState.SCORE_SPEAKER_SETUP));
//    Constants.operatorController.a().whileTrue(superstructure.toState(SuperState.SAFE));
  
//    Constants.operatorController.rightBumper().whileTrue(superstructure.toState(superstructure.getShootState()));
//    Constants.operatorController.leftBumper().whileTrue(superstructure.toState(SuperState.GROUND_INTAKE));
//    Constants.operatorController.leftTrigger(0.1).whileTrue(superstructure.toState(SuperState.SOURCE_INTAKE));
    // Manual controls
    new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(1)) > 0.1)
            .whileTrue(m_elevator.runManual(Constants.operatorController::getLeftY));
    Constants.operatorController.rightBumper().whileTrue(new ParallelCommandGroup(m_shooter.shootIt(2000),m_feeder.runFeeder(0.5)));
    Constants.operatorController.leftBumper().whileTrue(m_intake.manualIntake());
    Constants.operatorController.leftTrigger(0.1).whileTrue(new ParallelCommandGroup(m_shooter.shootIt(-500),m_feeder.runFeeder(-0.5)));


    Constants.driverController.rightBumper().whileTrue(m_climber.setRightSpeed(-1));
    Constants.driverController.rightTrigger(0.1).whileTrue(m_climber.setRightSpeed(1));
    Constants.driverController.leftBumper().whileTrue(m_climber.setLeftSpeed(-1));
    Constants.driverController.leftTrigger(0.1).whileTrue(m_climber.setLeftSpeed(1));
    //Constants.driverController.b().whileTrue(superstructure.toState(SuperState.CLIMB_REACH));

    // TODO: Change this to follow the run/runOnce paradigm used by the Superstructure
    Constants.driverController.a().onTrue(new InstantCommand(m_drivebase::zeroGyro));
    Constants.driverController.x().onTrue(new InstantCommand(m_drivebase::addFakeVisionReading));
    Constants.driverController.b().onTrue(new InstantCommand(m_drivebase::lock));
    Constants.driverController.y().whileTrue(Commands.deferredProxy(() -> m_drivebase.driveToPose(
            new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    ));
  }

  public void configurePathPlanner() {
    // TODO: These are example NamedCommands, import the real NamedCommands from the `swerve` branch
    NamedCommands.registerCommand("Ground Intake",
            superstructure.toState(SuperState.GROUND_INTAKE).withTimeout(3));
    NamedCommands.registerCommand("Safe", superstructure.toState(SuperState.SAFE).withTimeout(3));
    m_drivebase.setupPathPlanner();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A Path will be run in autonomous
    //(null)
//    return drivebase.spinCounterClockwise();//drivebase.getAutonomousCommand("TESTER", true);

    // Runs an Auto
//    return new PathPlannerAuto("Simple Auto");

//    Gets Selected Auto from Shuffleboard
    return autoChooser.getSelected();
//    return null;
  }

  public void setDriveMode()
  {
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = m_drivebase.driveCommand(
            () -> MathUtil.applyDeadband(Constants.driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(Constants.driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> Constants.driverController.getRightX(),
            () -> -Constants.driverController.getRightY());

    Command driveFieldOrientedDirectAngleSim = m_drivebase.simDriveCommand(
            () -> MathUtil.applyDeadband(Constants.driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(Constants.driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> Constants.driverController.getRawAxis(2));

    m_drivebase.setDefaultCommand(
            !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngle);

  }

  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }
}
