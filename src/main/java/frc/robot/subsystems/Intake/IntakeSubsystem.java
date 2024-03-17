package frc.robot.subsystems.Intake;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem extends SubsystemBase {
    
    private final CANSparkMax intakeMotor;

    private final DoubleSolenoid intakePiston;

    public static final DoubleSolenoid.Value intakePistonDownPosition = Value.kReverse;

    public static final DoubleSolenoid.Value intakePistonUpPosition = Value.kForward;


    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeMotor.setInverted(false);
        intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,Constants.IntakeConstants.forwardChannelPort, Constants.IntakeConstants.reverseChannelPort);
    }

    public void intakeOrOuttake(double power){
        intakeMotor.set(power);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public double getSpeed(){
        return intakeMotor.get();
    }

    public enum IntakeState{
        RETRACTED(intakePistonUpPosition,0),
        EXTENDED(intakePistonDownPosition,1);

        public DoubleSolenoid.Value position;
        public double power;

        private IntakeState(DoubleSolenoid.Value position, double power){
            this.position = position;
            this.power = power;
        }

    }

    public void drop() {
        intakePiston.set(intakePistonDownPosition);
    }

    public void raise() {
        intakePiston.set(intakePistonUpPosition);
    }

    public void toggle() {
        intakePiston.toggle();
    }

    public Command positionIntake(DoubleSolenoid.Value position) {
        return run(() -> {
            intakePiston.set(position);
        });
    }

    public Command runIntake(double power){
        return run(() -> {
            intakeOrOuttake(power);
        });
    }

    public Command manualIntake(){
        return run(() -> {
            drop();
            intakeOrOuttake(0.5);
        });
    }

    public Command stopIntaking(){
         return runOnce(() -> {
            intakeOrOuttake(0);
        });
    }

    public DoubleSolenoid.Value getIntakePistonPosition(){
        return intakePiston.get();
    }

    @Override
    public void periodic()
    {
        //System.out.println("This the position of the intake: " + getIntakePistonPosition());
        //System.out.println("This is the power of the intake: " + getSpeed());

    }
}

