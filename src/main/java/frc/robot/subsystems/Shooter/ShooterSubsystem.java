package frc.robot.subsystems.Shooter;


import com.revrobotics.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.encoders.SparkMaxAnalogEncoderSwerve;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftShooter;
    private final CANSparkMax rightShooter;
    private final SparkPIDController PIDController;

    //private final SparkAbsoluteEncoder absoluteEncoder;

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private final DoubleSolenoid trapPiston;

    public static final DoubleSolenoid.Value pistonRetractedPosition = DoubleSolenoid.Value.kReverse;

    public static final DoubleSolenoid.Value pistonExtendedPosition = DoubleSolenoid.Value.kForward;

    private double target_Speed;


    public ShooterSubsystem() {
        leftShooter = new CANSparkMax(Constants.ShooterConstants.leftShooterID, CANSparkLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkMax(Constants.ShooterConstants.rightShooterID, CANSparkLowLevel.MotorType.kBrushless);
        leftShooter.restoreFactoryDefaults();
        rightShooter.restoreFactoryDefaults();
        leftShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightEncoder = rightShooter.getEncoder();
        leftEncoder = leftShooter.getEncoder();
        PIDController = rightShooter.getPIDController();
        PIDController.setFeedbackDevice(rightEncoder);
        set(PIDF.PROPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
                PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);
        PIDController.setSmartMotionMaxVelocity(PIDF.MAXVELOCITY, 0);
        PIDController.setSmartMotionMaxAccel(PIDF.MAXACCELERATION, 0);
        trapPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,Constants.ShooterConstants.forwardChannelPort, Constants.ShooterConstants.reverseChannelPort);
        leftShooter.follow(rightShooter,true);
        leftShooter.burnFlash();
        rightShooter.burnFlash();
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

        public static final double MAXVELOCITY = 7500;

        public static final double MAXACCELERATION = 3500;

    }

    public void shoot(double power){
        rightShooter.set(power);
    }
    public void stop() {
        rightShooter.set(0);
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
        PIDController.setReference(targetSpeed, CANSparkMax.ControlType.kSmartVelocity);
    }

    public double getSpeed(){
        return target_Speed;
    }

    public double getPower(){
       return rightShooter.get();
    }

    public Command shootIt(double targetSpeed){
        this.target_Speed = targetSpeed;
        return run(() -> runPID(targetSpeed));
    }

    public Command manualShoot(double power){
        return run(() -> shoot(power));
    }

    public void retract() {
        trapPiston.set(pistonRetractedPosition);
    }

    public void extend() {
        trapPiston.set(pistonExtendedPosition);
    }

    public Command positionPiston(DoubleSolenoid.Value position) {
        return run(() -> {
            trapPiston.set(position);
        });
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
         SmartDashboard.putNumber("Shooter Velocity", rightEncoder.getVelocity());
        //System.out.println("This is the speed of the shooter: " + getSpeed());
        //encoderVelocity = shooterMotorRight.getSelectedSensorVelocity(pidIdx.PRIMARY.ordinal());
    }

}

