package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax leftLift;

    private final CANSparkMax rightLift;

    private final SparkPIDController PIDController;
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;
    private final DigitalInput limitSwitchLATop;
    private final DigitalInput limitSwitchLABottom;


    public ElevatorSubsystem() {
        leftLift = new CANSparkMax(Constants.ElevatorConstants.leftLiftID, CANSparkLowLevel.MotorType.kBrushless);
        rightLift = new CANSparkMax(Constants.ElevatorConstants.rightLiftID, CANSparkLowLevel.MotorType.kBrushless);
        leftLift.restoreFactoryDefaults();
        rightLift.restoreFactoryDefaults();
        leftLift.follow(rightLift);
        rightLift.setInverted(true);
        leftLift.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightLift.setIdleMode(CANSparkMax.IdleMode.kBrake);
        limitSwitchLATop = new DigitalInput(Constants.ElevatorConstants.limitSwitchLATop);
        limitSwitchLABottom = new DigitalInput(Constants.ElevatorConstants.limitSwitchLABottom);
        rightEncoder = rightLift.getEncoder();
        leftEncoder = leftLift.getEncoder();
        PIDController = rightLift.getPIDController();
        PIDController.setFeedbackDevice(rightEncoder);
        set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
              PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);
    }

    public static class PIDF {

        /**
         * Feedforward constant for PID loop
         */
        public static final double FEEDFORWARD = 0.01;
        /**
         * Proportion constant for PID loop
         */
        public static final double PROPORTION = 0.05;
        /**
         * Integral constant for PID loop
         */
        public static final double INTEGRAL = 0.0;
        /**
         * Derivative constant for PID loop
         */
        public static final double DERIVATIVE = 0.0;
        /**
         * Integral zone constant for PID loop
         */
        public static final double INTEGRAL_ZONE = 0.0;
    }


    public void changeAngle(double liftPower) {
        rightLift.set(liftPower);
    }

    public void stop() {
        rightLift.set(0);
    }

    public double getAngle(){
        return rightEncoder.getPosition()*360;
    }

    public void set(double p, double i, double d, double f, double iz)
    {
        PIDController.setP(p);
        PIDController.setI(i);
        PIDController.setD(d);
        PIDController.setFF(f);
        PIDController.setIZone(iz);
    }

    public void runPID(double targetPosition)
    {
        PIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    }


    public Command setAngle(double degrees){
        return run(() -> {
            runPID(degrees);
        });
    }

    public Command runManual(DoubleSupplier supplier){
        double power = supplier.getAsDouble();
        return run(() -> {
            changeAngle(power);
        });
    }

    public enum ElevatorState {
        MAXANGLE(80),
        MIDANGLE(50),
        MINANGLE(30);

        public double angle;

        private ElevatorState(double angle){
            this.angle = angle;
        }
    }

    @Override
    public void periodic() {
        System.out.println("This is the angle of the elevator: " + getAngle());
    }

}
