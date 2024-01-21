package frc.robot.subsystems.Swerve;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class SwerveState
{

    public Command        command;

    public SwerveState(Command driveCommand)
    {
        this.command = driveCommand;
    }

    public static SwerveState getDriveState()
    {
        return new SwerveState(
                SwerveSubsystem.getInstance()
                        .driveCommand(
                                () -> MathUtil.applyDeadband(Constants.driverController.getLeftX(),
                                        0.2),
                                () -> MathUtil.applyDeadband(Constants.driverController.getLeftY(),
                                        0.2),
                                () -> Constants.driverController.getRawAxis(2)
                        ));
    }
}