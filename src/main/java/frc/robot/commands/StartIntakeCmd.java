package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class StartIntakeCmd extends Command {
    private IntakeSubsystem m_intake;
    private Timer timer;

    public StartIntakeCmd(IntakeSubsystem intake)
    {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.runIntake(0);
        timer.start();
    }

    @Override
    public void execute() {
        System.out.println(timer.get());
        m_intake.runIntake(0.8);
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
