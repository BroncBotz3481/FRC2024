package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class StopStageNoteCmd extends Command {
    private FeederSubsystem m_feeder;
    private IntakeSubsystem m_intake;
    private Timer timer;

    public StopStageNoteCmd(FeederSubsystem feeder, IntakeSubsystem intake) {
        m_feeder = feeder;
        m_intake = intake;
        addRequirements(m_feeder);
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(timer.get());
        m_intake.runIntake(0);
        m_feeder.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return timer.get()>=0.1;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }
}
