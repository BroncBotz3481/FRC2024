package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class SuperstructureToState extends SequentialCommandGroup {
    private final Superstructure m_superstructure;

    private final SuperState m_targetState;

    private BooleanSupplier m_climberWait = () -> true;
    private BooleanSupplier m_feederWait = () -> true;
    private BooleanSupplier m_intakeWait = () -> true;
    private BooleanSupplier m_shooterWait = () -> true;
    private BooleanSupplier m_elevatorWait = () -> true;
    private BooleanSupplier m_climberUntil = () -> false;
    private BooleanSupplier m_feederUntil = () -> false;
    private BooleanSupplier m_intakeUntil = () -> false;
    private BooleanSupplier m_shooterUntil = () -> false;
    private BooleanSupplier m_elevatorUntil = () -> false;

    public SuperstructureToState(Superstructure superstructure,SuperState targetState){
        m_superstructure = superstructure;
        m_targetState = targetState;

        ClimberSubsystem climber = superstructure.m_climber;
        FeederSubsystem feeder = superstructure.m_feeder;
        IntakeSubsystem intake = superstructure.m_intake;
        LEDSubsystem LED = superstructure.m_LED;
        ShooterSubsystem shooter = superstructure.m_shooter;
        ElevatorSubsystem elevator = superstructure.m_elevator;


        CommandBase initCmd = Commands.runOnce(() -> {
            m_superstructure.updateState(m_targetState);
        });

        determineConditions();

        CommandBase shooterCmd = Commands.waitUntil(m_shooterWait).andThen(superstructure.m_shooter.shootIt(m_targetState.shoot.speed).until(m_shooterUntil));
        CommandBase feederCmd = Commands.waitUntil(m_feederWait).andThen(superstructure.m_feeder.setSpeed(m_targetState.feed.power).until(m_feederUntil));
        CommandBase elevatorCmd = Commands.waitUntil(m_elevatorWait).andThen(superstructure.m_elevator.setAngle(m_targetState.elevator.angle).until(m_elevatorUntil));
        CommandBase intakeCmd = Commands.waitUntil(m_intakeWait).andThen(superstructure.m_intake.positionIntake(m_targetState.intake.position).until(m_intakeUntil));
        CommandBase climberCmd = Commands.waitUntil(m_climberWait).andThen(superstructure.m_climber.setHeight(m_targetState.climb.height)).until(m_climberUntil);

        addCommands(initCmd,
                Commands.parallel(
                        shooterCmd,
                        feederCmd,
                        elevatorCmd,
                        intakeCmd,
                        climberCmd)
        );
    }

    private void determineConditions() {
        ClimberSubsystem climber = m_superstructure.m_climber;
        FeederSubsystem feeder = m_superstructure.m_feeder;
        IntakeSubsystem intake = m_superstructure.m_intake;
        ShooterSubsystem shooter = m_superstructure.m_shooter;
        ElevatorSubsystem elevator = m_superstructure.m_elevator;

        if (m_targetState == SuperState.SCORE_SPEAKER){
            m_shooterWait = () -> (elevator.getAngle() >= (m_targetState.elevator.angle));
            m_feederWait = () -> (shooter.getSpeed() >= (m_targetState.shoot.speed));
        }

        if (m_targetState == SuperState.SCORE_AMP) {
            m_shooterWait = () -> (elevator.getAngle() >= (m_targetState.elevator.angle));
            m_feederWait = () -> (shooter.getSpeed() >= (m_targetState.shoot.speed));
        }

        if (m_targetState == SuperState.SCORE_STAGE_PROTECTED) {
            m_shooterWait = () -> (elevator.getAngle() >= (m_targetState.elevator.angle));
            m_feederWait = () -> (shooter.getSpeed() >= (m_targetState.shoot.speed));
        }

        if (m_targetState == SuperState.SOURCE_INTAKE) {
            m_shooterWait = () -> (elevator.getAngle() >= (m_targetState.elevator.angle));
            m_shooterUntil = feeder::getBeamBrakeState;
            m_feederWait = () -> true;
            m_feederUntil = feeder::getBeamBrakeState;
        }

        if (m_targetState == SuperState.GROUND_INTAKE) {
            m_intakeWait = () -> true;
            m_feederWait = () -> (intake.getIntakePistonPosition() == IntakeSubsystem.intakePistonDownPosition);
            m_feederUntil = feeder::getBeamBrakeState;
        }

        if (m_targetState == SuperState.SAFE) {
            m_feederWait = () -> true;
            m_elevatorWait = () -> true;
            m_intakeWait = () ->  true;
            m_climberWait = () -> true;
        }

        if (m_targetState == SuperState.CLIMB_REACH) {
            m_climberWait = () -> (elevator.getAngle() >= (m_targetState.elevator.angle) && (intake.getIntakePistonPosition() == IntakeSubsystem.intakePistonUpPosition));
        }

    }

}
