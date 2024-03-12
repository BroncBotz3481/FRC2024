package frc.robot.subsystems.Shooter;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.encoders.SparkMaxAnalogEncoderSwerve;

public class ShooterSubsystem extends SubsystemBase {
    // private final CANSparkMax leftShooter;
    // private final CANSparkMax rightShooter;
    // private final SparkPIDController PIDController;
    private final TalonSRX leftShooter;
    private final TalonSRX rightShooter;
    //private final SparkAbsoluteEncoder absoluteEncoder;

    // private final RelativeEncoder rightEncoder;
    // private final RelativeEncoder leftEncoder;

    private double target_Speed;

    enum slotIdx {
        DISTANCE,
        TURNING, 
        VELOCITY,
        MOTIONPROFILE,
    }
    enum pidIdx {
        PRIMARY,
        AUXILLARY
        
    }

    public ShooterSubsystem() {
        leftShooter = new TalonSRX(Constants.ShooterConstants.leftShooterID);
        rightShooter = new TalonSRX(Constants.ShooterConstants.rightShooterID);
        leftShooter.setInverted(true);
        leftShooter.follow(rightShooter);
        set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE, PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);
        // leftShooter = new CANSparkMax(Constants.ShooterConstants.leftShooterID, CANSparkLowLevel.MotorType.kBrushless);
        // rightShooter = new CANSparkMax(Constants.ShooterConstants.rightShooterID, CANSparkLowLevel.MotorType.kBrushless);
        // leftShooter.restoreFactoryDefaults();
        // rightShooter.restoreFactoryDefaults();
        // leftShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        // rightShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        // rightEncoder = rightShooter.getEncoder();
        // leftEncoder = leftShooter.getEncoder();
        // PIDController = rightShooter.getPIDController();
        // PIDController.setFeedbackDevice(rightEncoder);
        // set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
        //         PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);
        // PIDController.setSmartMotionMaxVelocity(PIDF.MAXVELOCITY, 0);
        // PIDController.setSmartMotionMaxAccel(PIDF.MAXACCELERATION, 0);
        // leftShooter.follow(rightShooter,true);
        // leftShooter.burnFlash();
        // rightShooter.burnFlash();
    }

    public static class PIDF
    {

        /**
         * Feedforward constant for PID loop
         */
        public static final double FEEDFORWARD   = 0.00018;
        /**
         * Proportion constant for PID loop
         */
        public static final double PROPORTION    = 0.0001;
        /**
         * Integral constant for PID loop
         */
        public static final double INTEGRAL      = 0;
        /**
         * Derivative constant for PID loop
         */
        public static final double DERIVATIVE    = 0;
        /**
         * Integral zone constant for PID loop
         */
        public static final double INTEGRAL_ZONE = 0;
        /**
         * Max velocity for Smart Velocity
         */
        public static final double MAXVELOCITY = 7500;
        /**
         * Max acceleration for Smart Velocity
         */
        public static final double MAXACCELERATION = 3500;

    }

    public void shoot(double power){
        rightShooter.set(ControlMode.PercentOutput, power);
    }
    public void stop() {
        rightShooter.set(ControlMode.PercentOutput, 0);
    }

    public void set(double p, double i, double d, double f, double iz)
    {
        rightShooter.selectProfileSlot(slotIdx.VELOCITY.ordinal(), pidIdx.PRIMARY.ordinal()); // First parameter "2" correlates to velocity, second parameter correlates to primary PID
        rightShooter.config_kP(slotIdx.VELOCITY.ordinal(), p);  // 0.087 First parameter is primary PID, second parameter is velocity
        rightShooter.config_kI(slotIdx.VELOCITY.ordinal(), i);
        rightShooter.config_kD(slotIdx.VELOCITY.ordinal(), d);
        rightShooter.config_kF(slotIdx.VELOCITY.ordinal(), f);
        rightShooter.config_IntegralZone(slotIdx.VELOCITY.ordinal(), iz);
        rightShooter.configAllowableClosedloopError(slotIdx.VELOCITY.ordinal(), pidIdx.PRIMARY.ordinal());
        rightShooter.configClosedLoopPeriod(slotIdx.VELOCITY.ordinal(), 1);
        rightShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 50);
        rightShooter.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 50);
        // PIDController.setP(p);
        // PIDController.setI(i);
        // PIDController.setD(d);
        // PIDController.setFF(f);
        // PIDController.setIZone(iz);
    }

    public void runPID(double targetSpeed)
    {
        // PIDController.setReference(targetSpeed, CANSparkMax.ControlType.kSmartVelocity);
        rightShooter.set(ControlMode.Velocity, targetSpeed);
    }

    public double getSpeed(){
        return rightShooter.getSelectedSensorVelocity();
    }

    public double getPower(){
       return rightShooter.getMotorOutputPercent();
    }

    public Command shootIt(double targetSpeed){
        this.target_Speed = targetSpeed;
        return run(() -> runPID(targetSpeed));
    }

    public Command manualShoot(double power){
        return run(() -> shoot(power));
    }

    public enum ShooterState{
        HIGHPOWER(100),
        MIDPOWER(50),
        LOWPOWER(25),
        REVERSEDINTAKE(-10),
        OFF(0);

        public double speed;

        private ShooterState(double speed){
            this.speed = speed;
        }
    }

    @Override
    public void periodic()
    {
         SmartDashboard.putNumber("Shooter Velocity", rightShooter.getSelectedSensorVelocity());
    }

}

