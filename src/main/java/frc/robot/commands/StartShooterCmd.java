package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
public class StartShooterCmd extends Command {
    private ShooterSubsystem m_shooter;
    private Timer timer;
    public StartShooterCmd(ShooterSubsystem shooter)
    {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.shoot(0);
        timer.start();
    }

    @Override
    public void execute() {
        System.out.println(timer.get());
        m_shooter.shoot(0.8);
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
