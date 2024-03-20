package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;


public class StageSubNoteCmd extends Command {
    private FeederSubsystem m_feeder;
    private ShooterSubsystem m_shooter;
    private Timer timer;

    public StageSubNoteCmd(FeederSubsystem feeder, ShooterSubsystem shooter) {
        m_feeder = feeder;
        m_shooter = shooter;
        timer = new Timer();
        addRequirements(m_feeder);
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_feeder.runFeederAuto(0,0,false);
        m_shooter.shoot(0);
        timer.start();
    }

    @Override
    public void execute() {
        m_feeder.runFeederAuto(0.8,-0.8,false);
        m_shooter.shoot(0.8); //Ramps up shooter so that note is staged with shooter fully up to speed
    }

    @Override
    public boolean isFinished() {
        return m_feeder.getBeamBrakeState();
    }


    @Override
    public void end(boolean interrupted) {
        m_feeder.runFeederAuto(0,0,false);
        m_shooter.shoot(0);
        timer.stop();
        timer.reset();
    }
}
