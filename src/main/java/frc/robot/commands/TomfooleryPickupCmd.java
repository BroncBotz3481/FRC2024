package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;


public class TomfooleryPickupCmd extends Command {
    private FeederSubsystem m_feeder;
    private ShooterSubsystem m_shooter;
    private Timer timer;

    public TomfooleryPickupCmd(FeederSubsystem feeder, ShooterSubsystem shooter) {
        m_feeder = feeder;
        m_shooter = shooter;
        addRequirements(m_feeder);
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_feeder.setSpeed(0,0);
        m_shooter.shoot(0);
        timer.start();
    }

    @Override
    public void execute() {
        m_feeder.setSpeed(0.8,-0.8);
        m_shooter.shoot(0.7); // Power 70 percent as shooting right up against the speaker (JUST RAMP UP)
    }

    @Override
    public boolean isFinished() {
        return timer.get()>=2; //2 Sec Pickup to stage note(SHOULD NOT SHOOT) Shooter also ramped up
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0);
        m_feeder.setSpeed(0, 0);
        timer.stop();
        timer.reset();
    }
}
