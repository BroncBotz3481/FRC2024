package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter.ShooterSubsystem;


public class ShooterHighPwrCmd extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;

    public ShooterHighPwrCmd(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        this.shooterSubsystem.stop();
    }

    @Override
    public void execute() {
        shooterSubsystem.highPwr();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
