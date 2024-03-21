package frc.robot.subsystems.Shooter;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

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
        leftShooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightShooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // leftShooter.setSensorPhase(true); // <<<<<< Adjust this

        rightShooter.setSensorPhase(false); // <<<<<< Adjust this
        rightShooter.setNeutralMode(NeutralMode.Coast);
        leftShooter.setNeutralMode(NeutralMode.Coast);
        set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE, PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);
       
    }

    public static class PIDF
    {

        /**
         * Feedforward constant for PID loop
         */
        public static final double FEEDFORWARD   = 0;  //0.00018
        /**
         * Proportion constant for PID loop
         */
        public static final double PROPORTION    = 0.55; //0.0001
        /**
         * Integral constant for PID loop
         */
        public static final double INTEGRAL      = 0;
        /**
         * Derivative constant for PID loop
         */
        public static final double DERIVATIVE    = 0.15;
        /**
         * Integral zone constant for PID loop
         */
        public static final double INTEGRAL_ZONE = 0;
        /**
         * Max velocity for Smart Velocity
         */
        public static final double MAXVELOCITY = 300000;
        /**
         * Max acceleration for Smart Velocity
         */
        public static final double MAXACCELERATION = 200000;

    }

    public void shoot(double power){
        rightShooter.set(ControlMode.PercentOutput, power);
        leftShooter.set(ControlMode.PercentOutput, power);
    }
    public void stop() {
        rightShooter.set(ControlMode.PercentOutput, 0);
        leftShooter.set(ControlMode.PercentOutput, 0);
    }

    public void set(double p, double i, double d, double f, double iz)
    {
        rightShooter.selectProfileSlot(slotIdx.VELOCITY.ordinal(), pidIdx.PRIMARY.ordinal()); // First parameter "2" correlates to velocity, second parameter correlates to primary PID
        rightShooter.config_kP(slotIdx.VELOCITY.ordinal(), p);  // 0.087 First parameter is primary PID, second parameter is velocity
        rightShooter.config_kI(slotIdx.VELOCITY.ordinal(), i);
        rightShooter.config_kD(slotIdx.VELOCITY.ordinal(), d);
        rightShooter.config_kF(slotIdx.VELOCITY.ordinal(), f);
        rightShooter.config_IntegralZone(slotIdx.VELOCITY.ordinal(), iz);
        rightShooter.configAllowableClosedloopError(slotIdx.VELOCITY.ordinal(), pidIdx.PRIMARY.ordinal());  // second parameter is absolute value of allowable error
        rightShooter.configClosedLoopPeriod(slotIdx.VELOCITY.ordinal(), 1);
        rightShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 50);
        rightShooter.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 50);

        leftShooter.selectProfileSlot(slotIdx.VELOCITY.ordinal(), pidIdx.PRIMARY.ordinal()); // First parameter "2" correlates to velocity, second parameter correlates to primary PID
        leftShooter.config_kP(slotIdx.VELOCITY.ordinal(), p);  // 0.087 First parameter is primary PID, second parameter is velocity
        leftShooter.config_kI(slotIdx.VELOCITY.ordinal(), i);
        leftShooter.config_kD(slotIdx.VELOCITY.ordinal(), d);
        leftShooter.config_kF(slotIdx.VELOCITY.ordinal(), f);
        leftShooter.config_IntegralZone(slotIdx.VELOCITY.ordinal(), iz);
        leftShooter.configAllowableClosedloopError(slotIdx.VELOCITY.ordinal(), pidIdx.PRIMARY.ordinal());  // second parameter is absolute value of allowable error
        leftShooter.configClosedLoopPeriod(slotIdx.VELOCITY.ordinal(), 1);
        leftShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 50);
        leftShooter.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 50);
    }

    public void runPID(double targetSpeed)
    {
        rightShooter.set(ControlMode.Velocity, targetSpeed);
        leftShooter.set(ControlMode.Velocity, targetSpeed);

    }

    public double getSpeed(){
        return rightShooter.getSelectedSensorVelocity();
    }

    public double getPower(){
       return rightShooter.getMotorOutputPercent();
    }

    public boolean rampedUp(){
        double error = Math.abs(getSpeed()-this.target_Speed);
        return error<=150; // error is less than x rpm
        // If getSpeed() is within acceptable error
    }


    public Command shootIt(double targetSpeed){
        this.target_Speed = targetSpeed;
        return run(() -> {
            rightShooter.set(ControlMode.Velocity, targetSpeed);
            leftShooter.set(ControlMode.Velocity, targetSpeed);
        });
        /*/.until(() ->rightShooter.getClosedLoopError() < 100)
        .finallyDo(()->{System.out.println("ending");});*/
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
         SmartDashboard.putNumber("Right Shooter Velocity", rightShooter.getSelectedSensorVelocity());
         SmartDashboard.putNumber("Left Shooter Velocity", leftShooter.getSelectedSensorVelocity());
    }

}

