package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final DigitalInput leftLimitSwitchTop;
    private final DigitalInput leftLimitSwitchBottom;

    private final DigitalInput rightLimitSwitchTop;
    private final DigitalInput rightLimitSwitchBottom;

    private double targetAngle;


    public ElevatorSubsystem() {
        leftLift = new CANSparkMax(Constants.ElevatorConstants.leftLiftID, CANSparkLowLevel.MotorType.kBrushless);
        rightLift = new CANSparkMax(Constants.ElevatorConstants.rightLiftID, CANSparkLowLevel.MotorType.kBrushless);
        leftLift.restoreFactoryDefaults();
        rightLift.restoreFactoryDefaults();
        leftLift.follow(rightLift);
        rightLift.setInverted(false);
        leftLift.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightLift.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftLimitSwitchTop = new DigitalInput(Constants.ElevatorConstants.leftLimitSwitchTop);
        leftLimitSwitchBottom = new DigitalInput(Constants.ElevatorConstants.leftLimitSwitchBottom);
        rightLimitSwitchTop = new DigitalInput(Constants.ElevatorConstants.rightLimitSwitchTop);
        rightLimitSwitchBottom = new DigitalInput(Constants.ElevatorConstants.rightLimitSwitchBottom);
        rightEncoder = rightLift.getEncoder();
        leftEncoder = leftLift.getEncoder();
        PIDController = rightLift.getPIDController();
        PIDController.setFeedbackDevice(rightEncoder);
        set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
              PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);
        leftLift.burnFlash();
        rightLift.burnFlash();
    }

    public static class PIDF {

        /**
         * Feedforward constant for PID loop
         */
        public static final double FEEDFORWARD = 0;
        /**
         * Proportion constant for PID loop
         */
        public static final double PROPORTION = 0;
        /**
         * Integral constant for PID loop
         */
        public static final double INTEGRAL = 0;
        /**
         * Derivative constant for PID loop
         */
        public static final double DERIVATIVE = 0;
        /**
         * Integral zone constant for PID loop
         */
        public static final double INTEGRAL_ZONE = 0;
    }


    public void changeAngle(double liftPower) {
        rightLift.set(liftPower);
    }

    public void stop() {
        rightLift.set(0);
    }

    //TODO: Actually do the math here to get the true angle of the elevator relative to the ground
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
        targetAngle = degrees;
        return run(() -> {
            runPID(degrees);
        });
    }

    public Command runManual(DoubleSupplier supplier){
        double power = supplier.getAsDouble();
        return run(() -> {
            changeAngle(power*0.3);
        });
    }

    public Command stopManual(){
        return run(()->{
            changeAngle(0);
        });
    }

    public Command lowerElevator() { 
        return run(() -> {rightLift.set(-0.3); leftLift.set(-0.3);}).until(() -> leftLimitSwitchBottom.get() || rightLimitSwitchBottom.get()).andThen(runOnce(() -> {
            // leftEncoder.setPosition(0);
            //leftEncoder.setPosition(22);
            //rightEncoder.setPosition(22);
            leftLift.set(0);
            rightLift.set(0);
        }));
    }

    public Command raiseElevator() {
        return run(() -> {rightLift.set(0.3); leftLift.set(0.3);}).until(() -> leftLimitSwitchTop.get() || rightLimitSwitchTop.get()).andThen(runOnce(() -> {
            //leftEncoder.setPosition(57);
            //rightEncoder.setPosition(57);
            leftLift.set(0);
            rightLift.set(0);
        }));
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
        SmartDashboard.putNumber("Lower Limit Switch Right", rightLimitSwitchBottom.get() ? 1 : 0);
        SmartDashboard.putNumber("Upper Limit Switch Right", rightLimitSwitchTop.get() ? 1 : 0);
        SmartDashboard.putNumber("Lower Limit Switch Left", leftLimitSwitchBottom.get() ? 1 : 0);
        SmartDashboard.putNumber("Upper Limit Switch Left", leftLimitSwitchTop.get() ? 1 : 0);
        SmartDashboard.putNumber("Right Position", rightEncoder.getPosition());
        SmartDashboard.putNumber("Left Position", leftEncoder.getPosition());

    }

}

