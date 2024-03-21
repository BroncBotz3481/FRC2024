package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.FeederSubsystem;


public class AutoHalfSecondFeederCmd extends Command {
    private FeederSubsystem m_feeder;
    private Timer timer;

    public AutoHalfSecondFeederCmd(FeederSubsystem feeder) {
        m_feeder = feeder;
        timer = new Timer();
        addRequirements(m_feeder);
    }

    @Override
    public void initialize() {
        m_feeder.setSpeed(0.0,0);
        timer.start();
    }

    @Override
    public void execute() {
        m_feeder.setSpeed(0.9,0);
    }

    @Override
    public boolean isFinished() {
        return timer.get()>=0.5;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.setSpeed(0,0);
        timer.stop();
        timer.reset();
    }
}
