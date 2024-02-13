package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class Superstructure {

    public final ClimberSubsystem m_climber;
    public final FeederSubsystem m_feeder;
    public final IntakeSubsystem m_intake;
    public final ShooterSubsystem m_shooter;
    public final ElevatorSubsystem m_elevator;
    public final SwerveSubsystem m_swerve;
    public final LEDSubsystem m_LED;

    public SuperState m_prevState = SuperState.SAFE;

    private SuperState m_curState = SuperState.SAFE;



    public Superstructure(ClimberSubsystem climber, FeederSubsystem feeder, IntakeSubsystem intake,
                          ShooterSubsystem shooter, ElevatorSubsystem elevator, LEDSubsystem LED,
                          SwerveSubsystem swerve) {
        m_climber = climber;
        m_feeder = feeder;
        m_intake = intake;
        m_shooter = shooter;
        m_elevator = elevator;
        m_LED = LED;
        m_swerve = swerve;
        m_climber.setDefaultCommand(m_climber.setHeight(ClimberSubsystem.ClimberState.RETRACTED.height));
        m_feeder.setDefaultCommand(m_feeder.runFeeder(FeederSubsystem.FeederState.OFF.power));
        m_intake.setDefaultCommand(m_intake.positionIntake(IntakeSubsystem.IntakeState.RETRACTED.position));
        m_shooter.setDefaultCommand(m_shooter.shootIt(ShooterSubsystem.ShooterState.OFF.speed));
        m_elevator.setDefaultCommand(m_elevator.setAngle(ElevatorSubsystem.ElevatorState.MINANGLE.angle));
    }

    public Command toState(SuperState state){
        return new SuperstructureToState(this, state);
    }

    protected void updateState(SuperState newState) {
        System.out.println(
                "[SS] updateState - WAS " + m_prevState +
                        ", FROM " + m_curState +
                        " TO " + newState);
        m_prevState = m_curState;
        m_curState = newState;
    }

    public SuperState getPrevState() {
        return m_prevState;
    }

    public SuperState getCurState() {
        return m_curState;
    }

    public SuperState getShootState() {
        return switch (m_curState) {
            case SCORE_STAGE_PROTECTED_SETUP -> SuperState.SHOOT_PROTECTED;
            case SCORE_SPEAKER_SETUP -> SuperState.SHOOT_SPEAKER;
            case SCORE_AMP_SETUP -> SuperState.SHOOT_AMP;
            default -> SuperState.SAFE;
        };
    }

}


