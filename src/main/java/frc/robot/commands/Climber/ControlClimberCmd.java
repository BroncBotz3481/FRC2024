package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import edu.wpi.first.wpilibj.XboxController;

import java.util.function.DoubleSupplier;

public class ControlClimberCmd extends CommandBase {
    private final ClimberSubsystem climberSubsystem;

    private final DoubleSupplier powSupplier;

    public ControlClimberCmd(ClimberSubsystem climberSubsystem, DoubleSupplier powSupplier) {
        this.climberSubsystem = climberSubsystem;
        this.powSupplier = powSupplier;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climberSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        this.climberSubsystem.stop();

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        double power = powSupplier.getAsDouble();
        // Add logic to control the climber (e.g. a condition to determine whether to climb up or down)
        // For example, if you have a joystick or some manual control to move the climber
        //Don't have this type of logic inside of the command
//        if (power > 0.1) { // If joystick forwards (assuming positive Y is forward)
//            climberSubsystem.climbUp();
//        } else if (power < -0.1) { // If joystick backwards (assuming negative Y is backward)
//            climberSubsystem.climbDown();
//        } else { // If joystick is not being pushed significantly in Y direction
//            climberSubsystem.stop();
//        }

        climberSubsystem.run(power);
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        this.climberSubsystem.stop();
    }
}
