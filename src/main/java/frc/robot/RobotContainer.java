// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.superstructure.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
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
  //private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final LEDSubsystem m_LED = new LEDSubsystem();

  private final SwerveSubsystem m_drivebase = SwerveSubsystem.getInstance();

  private final SendableChooser<Command> autoChooser;
  public final Superstructure superstructure = new Superstructure(m_climber,
                                                                  m_feeder,
                                                                  m_shooter,
                                                                  m_elevator,
                                                                  m_LED,
                                                                  m_drivebase);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configurePathPlanner();
    autoChooser = AutoBuilder.buildAutoChooser("Complex");
    Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);
    configureBindings(); // Configure the trigger bindings
    // new Trigger(m_feeder::getBeamBrakeState).onTrue(m_shooter.runOnce(()->m_shooter.shoot(0.4)));
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
    //  new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(1)) > 0.1)
    //  .whileTrue(m_climber.setLeftSpeed(Constants.operatorController::getLeftY));
    //  new Trigger(() -> Math.abs(Constants.operatorController.getRawAxis(5)) > 0.1)
    //  .whileTrue(m_climber.setRightSpeed(Constants.operatorController::getRightY));





    // Operator Controls Below
    //Shooter and Feeder
    Constants.operatorController.a().whileTrue(new ParallelCommandGroup(m_feeder.runFeeder(0.8, -0.8, true), m_shooter.shootIt(65000))); //override to shoot High Power
    Constants.operatorController.leftBumper().whileTrue(new ParallelCommandGroup(m_feeder.runFeeder(0.8, -0.8, false), m_shooter.shootIt(65000))); //stage Note
    Constants.operatorController.y().whileTrue(new ParallelCommandGroup(m_feeder.runFeeder(-0.8, 0.8, true), m_shooter.shootIt(65000))); //Source Intake Reverse
    //TODO Find Correct HID for (Small Right)
    new Trigger(() -> Constants.operatorController.getHID().getRawButton(8)).whileTrue(m_feeder.runFeeder(-0.8, 0.8, false)); //SPIT Command

    //Elevator Controls
    Constants.operatorController.rightTrigger().onTrue(m_elevator.runElevator(0.085)); //MAX ANGLE
    Constants.operatorController.povLeft().onTrue(m_elevator.runElevator(0.06)); //Safe Zone or Far Shot Angle
    Constants.operatorController.leftTrigger().onTrue(m_elevator.runElevator(0.03)); //Amp Shot Angle
    Constants.operatorController.b().onTrue(m_elevator.runElevator(0.01)); //MIN Angle
    Constants.operatorController.povRight().onTrue(m_elevator.runElevator(0.07)); //Source Angle

    //Climbers
    Constants.operatorController.axisGreaterThan(1, 0.1).whileTrue(m_climber.setLeftSpeed(0.8));    
    Constants.operatorController.axisLessThan(1, -0.1).whileTrue(m_climber.setLeftSpeed(-0.8));  
    Constants.operatorController.axisGreaterThan(5, 0.1).whileTrue(m_climber.setRightSpeed(0.8));    
    Constants.operatorController.axisLessThan(5, -0.1).whileTrue(m_climber.setRightSpeed(-0.8));
    Constants.operatorController.povUp().whileTrue(m_climber.setBothSpeeds(-0.8));
    Constants.operatorController.povDown().whileTrue(m_climber.setBothSpeeds(0.8));



    // new Trigger(() -> Constants.operatorController.getHID().getRawButton(7)).whileTrue(m_elevator.runElevator(0.045));
    // new Trigger(() -> Constants.operatorController.getHID().getRawButton(8)).whileTrue(m_elevator.runElevator(0.08));
    // new Trigger(() -> Constants.operatorController.getHID().getRawButton(9)).whileTrue(m_elevator.runElevator(0.0));
    // Constants.operatorController.rightTrigger(0.1).whileTrue(new ParallelCommandGroup(m_elevator.raiseElevator(), m_shooter.manualShoot(0.7))); //Commands.waitUntil(m_shooter::rampedUp).andThen(m_feeder.runFeeder(0.5))
    // Constants.operatorController.leftBumper().whileTrue(m_shooter.shootIt(65000));
    // Constants.operatorController.rightBumper().whileTrue(new ParallelCommandGroup(m_shooter.manualShoot(-0.4),m_feeder.runFeeder(-0.7, 0, false)));

    //OLD MANUAL COMMANDS NOT USED
    //Constants.operatorController.leftTrigger(0.1).whileTrue(new ParallelCommandGroup(m_elevator.setAngle(41.5), m_shooter.manualShoot(0.3)));
    //Constants.operatorController.leftTrigger(0.1).whileTrue(m_intake.manualIntake());
    //Constants.operatorController.leftBumper().whileTrue(m_intake.stopIntaking());
    //Constants.operatorController.leftBumper().whileTrue(m_elevator.setAngle(43));
    //Constants.operatorController.leftBumper().whileTrue(m_shooter.manualShoot(0.8));

    // ! ROTATION VALUE IS IN RADIANS, 0 IS AWAY FROM YOU, PI IS TORWARD YOU
    Constants.driverController.povDown().whileTrue(m_drivebase.rotateToHeading(new Rotation2d(Units.degreesToRadians(180))).withTimeout(2));
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red) { // Red alliance
        Constants.driverController.povRight().whileTrue(m_drivebase.rotateToHeading(new Rotation2d(Units.degreesToRadians(270)))
                .withTimeout(2)); // Right -> amp
        Constants.driverController.povLeft().whileTrue(m_drivebase.rotateToHeading(new Rotation2d(Units.degreesToRadians(60)))
                .withTimeout(2)); // Left -> source
      }
      else { // Blue alliance
        Constants.driverController.povLeft().whileTrue(m_drivebase.rotateToHeading(new Rotation2d(Units.degreesToRadians(90)))
                .withTimeout(2)); // Left -> Amp
        Constants.driverController.povRight().whileTrue(m_drivebase.rotateToHeading(new Rotation2d(Units.degreesToRadians(300)))
                .withTimeout(2)); // Left -> source
      }
    } else { // No alliance -> Assume red
      Constants.driverController.povRight().whileTrue(m_drivebase.rotateToHeading(new Rotation2d(Units.degreesToRadians(270)))
              .withTimeout(2)); // Right -> amp
      Constants.driverController.povLeft().whileTrue(m_drivebase.rotateToHeading(new Rotation2d(Units.degreesToRadians(60)))
              .withTimeout(2)); // Left -> source
    }
                //Constants.operatorController.x().whileTrue(exampleSubsystem.runManual(()->0));
    //Constants.operatorController.x().whileTrue(new ParallelCommandGroup(m_shooter.manualShoot(0),m_feeder.runFeeder(0), m_climber.setRightSpeed(0), m_climber.setLeftSpeed(0), m_intake.stopIntaking(), m_elevator.stopManual()));
    
    // Constants.driverController.rightBumper().whileTrue(m_climber.setRightSpeed(-0.3));
    // Constants.driverController.rightTrigger(0.1).whileTrue(m_climber.setRightSpeed(0.3));
    // Constants.driverController.leftBumper().whileTrue(m_climber.setLeftSpeed(-0.3));
    // Constants.driverController.leftTrigger(0.1).whileTrue(m_climber.setLeftSpeed(0.3));
    //Constants.driverController.b().whileTrue(superstructure.toState(SuperState.CLIMB_REACH));

    // TODO: Change this to follow the run/runOnce paradigm used by the Superstructure
    // Driver Controls
    Constants.driverController.a().onTrue(new InstantCommand(m_drivebase::zeroGyro));
    Constants.driverController.b().onTrue(new InstantCommand(m_drivebase::lock));
    Constants.driverController.rightBumper().whileTrue(m_drivebase.aimAtSpeaker(1));
    Constants.driverController.y().whileTrue(Commands.deferredProxy(() -> m_drivebase.driveToPose(
            new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    ));
    // Constants.driverController.rightBumper().whileTrue(m_drivebase.setPowerScale(0.75));
    // Constants.driverController.leftBumper().whileTrue(m_drivebase.setPowerScale(0.50));
  }

  public void configurePathPlanner() {
//    NamedCommands.registerCommand("Ground Intake",
//            superstructure.toState(SuperState.GROUND_INTAKE).withTimeout(3));
//    NamedCommands.registerCommand("Safe", superstructure.toState(SuperState.SAFE).withTimeout(3));
//    NamedCommands.registerCommand("TestShoot1", m_shooter.shootIt(-5500));
//    NamedCommands.registerCommand("StopShooter", m_shooter.manualShoot(0));
//    NamedCommands.registerCommand("RunFeeder", m_feeder.runFeeder(0.15, false));
//    NamedCommands.registerCommand("StopFeeder", m_feeder.runFeeder(0, false));
//    NamedCommands.registerCommand("StartIntake", new StartIntakeCmd(m_intake));
    NamedCommands.registerCommand("RunShooterTest", new RunCommand(() ->{
      m_shooter.shoot(.5);}));
    m_drivebase.setupPathPlanner();

    NamedCommands.registerCommand("StageSubNote", new StageSubNoteCmd(m_feeder, m_shooter));
//    NamedCommands.registerCommand("ShootStagedNote", new ShootStagedNoteCmd(m_feeder, m_shooter));
    NamedCommands.registerCommand("StartStagedNote", new StartStageNoteCmd(m_feeder));
    NamedCommands.registerCommand("StopStagedNote", new StopStageNoteCmd(m_feeder));
    NamedCommands.registerCommand("StartShooter", new StartShooterCmd(m_shooter));
    NamedCommands.registerCommand("StopShooter", new StopShooterCmd(m_shooter));

    //For the Dummy Auto (Shooting from right in front of the speaker)
    NamedCommands.registerCommand("ShootFirstNote", new ShootFirstNote(m_shooter, m_feeder));
    NamedCommands.registerCommand("TomfooleryPickup", new TomfooleryPickupCmd(m_feeder, m_shooter));
    NamedCommands.registerCommand("TomfooleryShoot", new TomfooleryShootCmd(m_feeder, m_shooter));

    //For Complex Autos
    NamedCommands.registerCommand("Auto15SecondShoot", new Auto15SecondShootCmd(m_shooter));
    NamedCommands.registerCommand("FeedNote", m_feeder.runFeederCommand(0.9, -0.9).withTimeout(3));
    NamedCommands.registerCommand("AutoHalfSecondFeeder", new AutoHalfSecondFeederCmd(m_feeder));
    NamedCommands.registerCommand("SetElevatorCornerShot", m_elevator.runElevator(0.07).withTimeout(3));
    NamedCommands.registerCommand("SetElevatorCenterShot", m_elevator.runElevator(0.065).withTimeout(3));
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
            () -> -Constants.driverController.getRightX(),
            () -> -Constants.driverController.getRightY());

    Command driveFieldOrientedDirectAngleSim = m_drivebase.simDriveCommand(
            () -> MathUtil.applyDeadband(Constants.driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(Constants.driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> Constants.driverController.getRawAxis(2));

    m_drivebase.setDefaultCommand(
            !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngle);
    Constants.driverController.axisGreaterThan(2, .12)
            .or(Constants.driverController.axisGreaterThan(3, .12)).whileTrue(m_drivebase.driveCommand(
                    () -> (Math.abs(Constants.driverController.getRawAxis(1)) > OperatorConstants.LEFT_Y_DEADBAND)
                            ? Constants.driverController.getRawAxis(1)
                            : 0,
                    () -> (Math.abs(Constants.driverController.getRawAxis(0)) > OperatorConstants.LEFT_X_DEADBAND)
                            ? Constants.driverController.getRawAxis(0)
                            : 0,
                    () -> {
                      if (Math.abs(Constants.driverController.getRawAxis(2)) > .12) {
                        return Constants.driverController.getRawAxis(2) * .4; // CHANGE THIS CONSTANT IF YOU WANT IT TO BE FASTER OR SLOWER
                      } else if (Math.abs(Constants.driverController.getRawAxis(3)) > .12) {
                        return Constants.driverController.getRawAxis(3) * .4 * -1; // CHANGE THIS CONSTANT IF YOU WANT IT TO BE FASTER OR SLOWER
                      } else {
                        return 0;
                      }
                    }
            ));
  }

  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }
}
