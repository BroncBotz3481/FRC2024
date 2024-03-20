package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;


public class TomfooleryShootCmd extends Command {
    private FeederSubsystem m_feeder;
    private ShooterSubsystem m_shooter;
    private Timer timer;

    public TomfooleryShootCmd(FeederSubsystem feeder, ShooterSubsystem shooter) {
        m_feeder = feeder;
        m_shooter = shooter;
        addRequirements(m_feeder);
        addRequirements(m_shooter);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        m_feeder.setSpeed(0,0);
        m_shooter.shoot(0);
        timer.start();
    }

    @Override
    public void execute() {
        m_shooter.shoot(0.7); // Power 70 percent as shooting right up against the speaker
        if (timer.get()>=1)
            m_feeder.setSpeed(0.8,0);
    }

    @Override
    public boolean isFinished() {
        return timer.get()>=2; //1 Sec to ramp up and 1 sec to shoot
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0);
        m_feeder.setSpeed(0, 0);
        timer.stop();
        timer.reset();
    }
}
