package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private final DutyCycleEncoder leftEncoder = new DutyCycleEncoder(5); //Find the correct channels that the encoder is plugged in
    private final DutyCycleEncoder rightEncoder = new DutyCycleEncoder(4); //Find the correct channels that the encoder is plugged in


    private final ProfiledPIDController m_leftPidController =  new ProfiledPIDController(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE, PIDF.Contraints);
    private final ProfiledPIDController m_rightPidController =  new ProfiledPIDController(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE, PIDF.Contraints);

//    private final RelativeEncoder rightEncoder;
//    private final RelativeEncoder leftEncoder;
//
//    private final DigitalInput leftLimitSwitchTop;
//    private final DigitalInput leftLimitSwitchBottom;
//
//    private final DigitalInput rightLimitSwitchTop;
//    private final DigitalInput rightLimitSwitchBottom;

    private double m_setpoint;
    private double m_manualValue;

    private Timer m_timer;

    public ElevatorSubsystem() {
        leftLift = new CANSparkMax(Constants.ElevatorConstants.leftLiftID, CANSparkLowLevel.MotorType.kBrushless);
        rightLift = new CANSparkMax(Constants.ElevatorConstants.rightLiftID, CANSparkLowLevel.MotorType.kBrushless);
        leftLift.restoreFactoryDefaults();
        rightLift.restoreFactoryDefaults();
        leftLift.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightLift.setIdleMode(CANSparkMax.IdleMode.kCoast);
        // rightLift.enableSoftLimit(SoftLimitDirection.kForward, true);
        // rightLift.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // leftLift.enableSoftLimit(SoftLimitDirection.kForward, true);
        // leftLift.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        leftEncoder.setPositionOffset(0.730);
        rightEncoder.setPositionOffset(0.247);

        
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
        // leftLift.setInverted(true);
        // leftLift.follow(rightLift,false);
        
        // m_leftPidController.reset(getLeftAngle());
        // m_rightPidController.reset(getRightAngle());
        leftLift.burnFlash();
        rightLift.burnFlash();

        m_leftPidController.setTolerance(0.002);
        m_rightPidController.setTolerance(0.002);

        m_leftPidController.calculate(getLeftAngle(), 0.045);
        m_rightPidController.calculate(getRightAngle(), 0.045);
    }

    public void setTargetPosition(double setpoint) {
        if (setpoint != m_setpoint) {
          m_setpoint = setpoint;
        //   updateMotionProfile();
        }
      }
    

    public static class PIDF {

        public static final Constraints Contraints = new Constraints(0.09, 0.09);
        /**
         * Feedforward constant for PID loop
         */
        public static final double FEEDFORWARD = 0;
        /**
         * Proportion constant for PID loop
         */
        public static final double PROPORTION = 40;
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
    public double getLeftAngle(){
        return leftEncoder.get();
    }

    public double getRightAngle(){
        return -rightEncoder.get();
    }


    public Command runElevator(double setpoint)
    {
        double modified_setpoint = MathUtil.clamp(setpoint, 0, 0.090); //Low is 0 and Max angle is 0.090
        m_leftPidController.setGoal(modified_setpoint);
        m_rightPidController.setGoal(modified_setpoint);
        return run(() -> {
            System.out.println("running");
            leftLift.set(m_leftPidController.calculate(getLeftAngle(), modified_setpoint));
            rightLift.set(m_rightPidController.calculate(getRightAngle(), modified_setpoint));
        })
        .until(() -> m_leftPidController.atSetpoint() && m_rightPidController.atSetpoint() && MathUtil.isNear(modified_setpoint, getLeftAngle(), 0.001))
        .andThen(runOnce(() -> {leftLift.set(0); rightLift.set(0);System.out.println("ending");}));
    }

    public boolean elevatorAtPoint()
    {
        return m_leftPidController.atSetpoint() && m_rightPidController.atSetpoint();
    }

//    public void set(double p, double i, double d, double f, double iz)
//    {
//        PIDController.setP(p);
//        PIDController.setI(i);
//        PIDController.setD(d);
//        PIDController.setFF(f);
//        PIDController.setIZone(iz);
//    }

    // public void runPID(double targetPosition)
    // {
    //     m_controller.setGoal(targetPosition);
    //     rightLift.set(m_controller.calculate(rightEncoder.get()));
    //     //PIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    // }

    // public Command setAngle(double degrees){
    //     targetAngle = degrees;
    //     return run(() -> {
    //         runPID(degrees);
    //     });
    // }

    public Command runManual(DoubleSupplier supplier){
        double power = supplier.getAsDouble();
        return run(() -> {
            changeAngle(power*0.3);
        });
    }

    public Command stopManual(){
        return run(()->{
            leftLift.set(0);
            rightLift.set(0);
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
        SmartDashboard.putNumber("Left Throughbore", leftEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Right Throughbore", rightEncoder.getAbsolutePosition());
    }

}

