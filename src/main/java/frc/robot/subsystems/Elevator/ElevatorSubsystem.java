package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {


    private final CANSparkMax leftLift;

    private final CANSparkMax rightLift;

//    private final SparkPIDController PIDController;
//
//    private final SparkAbsoluteEncoder leftAbsoluteEncoder;
//    private final SparkAbsoluteEncoder rightAbsoluteEncoder;

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0); //Find the correct channels that the encoder is plugged in

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(PIDF.MAXVELOCITY, PIDF.MAXACCELERATION);

    private final ProfiledPIDController m_controller =  new ProfiledPIDController(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE, m_constraints);

//    private final RelativeEncoder rightEncoder;
//    private final RelativeEncoder leftEncoder;
//
//    private final DigitalInput leftLimitSwitchTop;
//    private final DigitalInput leftLimitSwitchBottom;
//
//    private final DigitalInput rightLimitSwitchTop;
//    private final DigitalInput rightLimitSwitchBottom;

    private double targetAngle;


    public ElevatorSubsystem() {
        leftLift = new CANSparkMax(Constants.ElevatorConstants.leftLiftID, CANSparkLowLevel.MotorType.kBrushless);
        rightLift = new CANSparkMax(Constants.ElevatorConstants.rightLiftID, CANSparkLowLevel.MotorType.kBrushless);
        leftLift.restoreFactoryDefaults();
        rightLift.restoreFactoryDefaults();
        leftLift.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightLift.setIdleMode(CANSparkMax.IdleMode.kBrake);
//        leftLimitSwitchTop = new DigitalInput(Constants.ElevatorConstants.leftLimitSwitchTop);
//        leftLimitSwitchBottom = new DigitalInput(Constants.ElevatorConstants.leftLimitSwitchBottom);
//        rightLimitSwitchTop = new DigitalInput(Constants.ElevatorConstants.rightLimitSwitchTop);
//        rightLimitSwitchBottom = new DigitalInput(Constants.ElevatorConstants.rightLimitSwitchBottom);
//        leftAbsoluteEncoder = leftLift.getAbsoluteEncoder();
//        rightAbsoluteEncoder = rightLift.getAbsoluteEncoder();
//        leftAbsoluteEncoder.setPositionConversionFactor((35.79)/42.95); //Dummy conversion factor
//        rightAbsoluteEncoder.setPositionConversionFactor(123134123); //Dummy conversion factor
//        rightEncoder = rightLift.getEncoder();
//        leftEncoder = leftLift.getEncoder();
//        rightEncoder.setPositionConversionFactor(28/40.09);
//        leftEncoder.setPositionConversionFactor(28/40.09);  //rotations to angles
//        rightEncoder.setPosition(53);
//        leftEncoder.setPosition(53);
//        PIDController = rightLift.getPIDController();
//        PIDController.setFeedbackDevice(rightAbsoluteEncoder);
//        set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
//              PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);
        leftLift.follow(rightLift,false);
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
        public static final double PROPORTION = 0.045;
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
        /**
         * Max velocity for ProfilePID
         */
        public static final double MAXVELOCITY = 300000; //Actually find good values for this
        /**
         * Max acceleration for ProfilePID
         */
        public static final double MAXACCELERATION = 200000; //Actually find good values for this
    }


    public void changeAngle(double liftPower) {
        rightLift.set(liftPower);
    }

    public void stop() {
        rightLift.set(0);
    }

    //TODO: Actually do the math here to get the true angle of the elevator relative to the ground
    public double getAngle(){
        return encoder.getAbsolutePosition();
    }

//    public void set(double p, double i, double d, double f, double iz)
//    {
//        PIDController.setP(p);
//        PIDController.setI(i);
//        PIDController.setD(d);
//        PIDController.setFF(f);
//        PIDController.setIZone(iz);
//    }

    public void runPID(double targetPosition)
    {
        m_controller.setGoal(targetPosition);
        rightLift.set(m_controller.calculate(encoder.get()));
        //PIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
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
        return run(() -> {
            rightLift.set(-0.15); leftLift.set(-0.15);
        });//.until(() -> leftLimitSwitchBottom.get() || rightLimitSwitchBottom.get()).andThen(runOnce(() -> {
            // leftEncoder.setPosition(0);
            //leftEncoder.setPosition(22);
            //rightEncoder.setPosition(22);
            //leftLift.set(0);
            //rightLift.set(0);
//            leftEncoder.setPosition(25);
//            rightEncoder.setPosition(25);
        
    }

    public Command raiseElevator() {
        return run(() -> {
            rightLift.set(0.15); leftLift.set(0.15);
        });//.until(() -> leftLimitSwitchTop.get() || rightLimitSwitchTop.get()).andThen(runOnce(() -> {
            //leftEncoder.setPosition(57);
            //rightEncoder.setPosition(57);
            //leftLift.set(0);
            //rightLift.set(0);
//            leftEncoder.setPosition(53);
//            rightEncoder.setPosition(53);

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
//        SmartDashboard.putNumber("Lower Limit Switch Right", rightLimitSwitchBottom.get() ? 1 : 0);
//        SmartDashboard.putNumber("Upper Limit Switch Right", rightLimitSwitchTop.get() ? 1 : 0);
//        SmartDashboard.putNumber("Lower Limit Switch Left", leftLimitSwitchBottom.get() ? 1 : 0);
//        SmartDashboard.putNumber("Upper Limit Switch Left", leftLimitSwitchTop.get() ? 1 : 0);
//        SmartDashboard.putNumber("Right Position", rightAbsoluteEncoder.getPosition());
//        SmartDashboard.putNumber("Left Position", leftAbsoluteEncoder.getPosition());

    }

}

