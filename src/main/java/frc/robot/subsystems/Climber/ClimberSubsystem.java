package frc.robot.subsystems.Climber;


import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax rightClimberMotor;
    private final CANSparkMax leftClimberMotor;

    private final SparkPIDController leftPIDController;

    private final SparkPIDController rightPIDController;

    private double leftTargetHeight;

    private double rightTargetHeight;

    private RelativeEncoder       rightEncoder;
    private RelativeEncoder       leftEncoder;
    private final DigitalInput lowerLimitSwitch;


    public ClimberSubsystem() {
        leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.leftClimberMotorID, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.rightClimberMotorID, MotorType.kBrushless);
        leftClimberMotor.restoreFactoryDefaults();
        rightClimberMotor.restoreFactoryDefaults();
        leftClimberMotor.setInverted(true);
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
        rightEncoder = rightClimberMotor.getEncoder();
        leftEncoder = leftClimberMotor.getEncoder();
        leftPIDController = leftClimberMotor.getPIDController();
        rightPIDController = rightClimberMotor.getPIDController();
        leftPIDController.setFeedbackDevice(leftEncoder);
        rightPIDController.setFeedbackDevice(rightEncoder);
        lowerLimitSwitch = new DigitalInput(Constants.ClimberConstants.lowerID);
        set(ClimberSubsystem.PIDF.PROPORTION, ClimberSubsystem.PIDF.INTEGRAL, ClimberSubsystem.PIDF.DERIVATIVE,
                ClimberSubsystem.PIDF.FEEDFORWARD, ClimberSubsystem.PIDF.INTEGRAL_ZONE);
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

    public void set(double p, double i, double d, double f, double iz)
    {
        leftPIDController.setP(p);
        leftPIDController.setI(i);
        leftPIDController.setD(d);
        leftPIDController.setFF(f);
        leftPIDController.setIZone(iz);
        rightPIDController.setP(p);
        rightPIDController.setI(i);
        rightPIDController.setD(d);
        rightPIDController.setFF(f);
        rightPIDController.setIZone(iz);
    }

    public void runRightMotor(double power) {
        rightClimberMotor.set(power);
    }

    public void runLeftMotor(double power) {
        leftClimberMotor.set(power);
    }

    public void runBothClimbers(double power){
        rightClimberMotor.set(power);
        leftClimberMotor.set(power);
    }

    public void stopRightMotor() {
        rightClimberMotor.set(0);
    }

    public void stopLeftMotor() {
        leftClimberMotor.set(0);
    }

    public void stopBothClimbers(){
        rightClimberMotor.set(0);
        leftClimberMotor.set(0);
    }

    public void runPIDLeft(double height){
        leftTargetHeight = height;
        leftPIDController.setReference(height, CANSparkMax.ControlType.kPosition);
    }

    public void runPIDRight(double height){
        rightTargetHeight = height;
        rightPIDController.setReference(height, CANSparkMax.ControlType.kPosition);
    }


    // Adds getter methods for the encoders
    public double getLeftEncoderPosition(){
        return leftEncoder.getPosition();
    }

    public double getRightEncoderPosition(){
        return rightEncoder.getPosition();
    }

    public enum ClimberState {

//        REST(0,0),
//
//        LEFTRUN(1,0),
//
//        RIGHTRUN(0,1),
//
//        LEFTRETRACT(-1,0),
//
//        RIGHTRETRACT(0,-1);

        RETRACTED(0),
        EXTENDED(100);

        public double height;

//        public double leftPower;
//
//        public double rightPower;

//        private ClimberState(double leftPower, double rightPower){
//            this.leftPower = leftPower;
//            this.rightPower = rightPower;
//        }

        private ClimberState(double height){
            this.height = height;
        }

    }

    public Command setHeight(double height){
        return run(()-> {
            runPIDLeft(height);
            runPIDRight(height);
        });

    }


    public Command setRightSpeed(double speed){
        return run(()-> {
            rightClimberMotor.set(speed);
        });
    }


    public Command setLeftSpeed(double speed){
        return run(()-> {
            leftClimberMotor.set(speed);
        });
    }

    @Override
    public void periodic()
    {
        //System.out.println("This is the height of the climber: " + targetHeight);
    }
}

