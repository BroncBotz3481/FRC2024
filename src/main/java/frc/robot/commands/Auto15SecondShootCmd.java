package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;


public class Auto15SecondShootCmd extends Command {
    private ShooterSubsystem m_shooter;
    private Timer timer;

    public Auto15SecondShootCmd(ShooterSubsystem shooter) {
        m_shooter = shooter;
        timer = new Timer();
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.stop();
        timer.start();
    }

    @Override
    public void execute() {
        m_shooter.runPID(65000);
    }

    @Override
    public boolean isFinished() {
        return timer.get()>=15;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        timer.stop();
        timer.reset();
    }
}
