package frc.robot.subsystems.Shooter;


import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftShooter;
    private final CANSparkMax rightShooter;
    private final SparkPIDController PIDController;
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private double target_Speed;


    public ShooterSubsystem() {
        leftShooter = new CANSparkMax(Constants.ShooterConstants.leftShooterID, CANSparkLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkMax(Constants.ShooterConstants.rightShooterID, CANSparkLowLevel.MotorType.kBrushless);
        leftShooter.restoreFactoryDefaults();
        rightShooter.restoreFactoryDefaults();
        leftShooter.follow(rightShooter);
        rightShooter.setInverted(true);
        leftShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightEncoder = rightShooter.getEncoder();
        leftEncoder = leftShooter.getEncoder();
        PIDController = rightShooter.getPIDController();
        PIDController.setFeedbackDevice(rightEncoder);
        set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
                PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);
    }

    public static class PIDF
    {

        /**
         * Feedforward constant for PID loop
         */
        public static final double FEEDFORWARD   = 0.01;
        /**
         * Proportion constant for PID loop
         */
        public static final double PROPORTION    = 0.05;
        /**
         * Integral constant for PID loop
         */
        public static final double INTEGRAL      = 0.0;
        /**
         * Derivative constant for PID loop
         */
        public static final double DERIVATIVE    = 0.0;
        /**
         * Integral zone constant for PID loop
         */
        public static final double INTEGRAL_ZONE = 0.0;
    }

    public void shoot(double power){
        rightShooter.set(power);;
    }
    public void stop() {
        rightShooter.set(0);;
    }

    public void set(double p, double i, double d, double f, double iz)
    {
        PIDController.setP(p);
        PIDController.setI(i);
        PIDController.setD(d);
        PIDController.setFF(f);
        PIDController.setIZone(iz);
    }

    public void runPID(double targetSpeed)
    {
        target_Speed = targetSpeed;
        PIDController.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity);
    }

    public double getSpeed(){
        return target_Speed;
    }

    public double getPower(){
       return rightShooter.get();
    }

    public Command shootIt(double targetSpeed){
        return run(() -> runPID(targetSpeed));
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
        System.out.println("This is the speed of the shooter: " + getSpeed());
        //encoderVelocity = shooterMotorRight.getSelectedSensorVelocity(pidIdx.PRIMARY.ordinal());
    }

}

