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

    private double targetHeight;

    private final SparkPIDController PIDController;

    private RelativeEncoder       rightEncoder;
    private RelativeEncoder       leftEncoder;
    private final DigitalInput lowerLimitSwitch;


    public ClimberSubsystem() {
        leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.leftClimberMotorID, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.rightClimberMotorID, MotorType.kBrushless);
        leftClimberMotor.restoreFactoryDefaults();
        rightClimberMotor.restoreFactoryDefaults();
        PIDController = rightClimberMotor.getPIDController();
        leftClimberMotor.setInverted(true);
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
        rightEncoder = rightClimberMotor.getEncoder();
        leftEncoder = leftClimberMotor.getEncoder();
        lowerLimitSwitch = new DigitalInput(Constants.ClimberConstants.lowerID);
    }

    public void runRightMotor(double power) {
        rightClimberMotor.set(power);
    }

    public void runLeftMotor(double power) {
        leftClimberMotor.set(power);
    }

    public void run(double power){
        rightClimberMotor.set(power);
        leftClimberMotor.set(power);
    }

    public void stopRightMotor() {
        rightClimberMotor.set(0);
    }

    public void stopLeftMotor() {
        leftClimberMotor.set(0);
    }

    public void stop(){
        rightClimberMotor.set(0);
        leftClimberMotor.set(0);
    }

    // Adds getter methods for the encoders
    public double getLeftEncoderPosition(){
        return leftEncoder.getPosition();
    }

    public double getRightEncoderPosition(){
        return rightEncoder.getPosition();
    }

    public double getHeight(){
        return rightEncoder.getPosition();
    }

    public enum ClimberState {
        RETRACTED(0),
        EXTENDED(100);

        public double height;

        private ClimberState(double height){
            this.height = height;
        }


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
        targetHeight = targetPosition;
        PIDController.setReference(targetPosition, CANSparkBase.ControlType.kPosition);
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

    public Command setHeight(double height){
        return run(() -> {
            runPID(height);
        });
    }


    @Override
    public void periodic()
    {
        //System.out.println("This is the height of the climber: " + targetHeight);
    }
}

