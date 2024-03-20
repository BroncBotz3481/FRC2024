package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.FeederSubsystem;


public class StartStageNoteCmd extends Command {
    private FeederSubsystem m_feeder;
    private Timer timer;

    public StartStageNoteCmd(FeederSubsystem feeder) {
       m_feeder = feeder;
       addRequirements(m_feeder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_feeder.setSpeed(0, 0);
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(timer.get());
            m_feeder.setSpeed(0.8, 0.8);
    }

    @Override
    public boolean isFinished() {
        return timer.get()>=3;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }
}
