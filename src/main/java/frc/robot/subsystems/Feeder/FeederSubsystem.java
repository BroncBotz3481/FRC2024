package frc.robot.subsystems.Feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

    private final CANSparkMax feederMotor;

    private final DigitalInput limitSwitchBeamBrake;


    public FeederSubsystem() {
        feederMotor = new CANSparkMax(Constants.FeederConstants.feederMotorID, CANSparkLowLevel.MotorType.kBrushless);
        feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        limitSwitchBeamBrake = new DigitalInput(Constants.FeederConstants.limitSwitchBeanBrakeChannel);
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


    public void setSpeed(double fPower) {
        feederMotor.set(fPower);
    }

    public void stopFeeder() {
        feederMotor.set(0);
    }

    public enum FeederState {

        FORWARD(1),
        OFF(0),
        REVERSE(-1);

       public double power;

       private FeederState(double power){
           this.power = power;
       }

    }

    public double getSpeed(){
        return feederMotor.get();
    }


    public boolean getBeamBrakeState(){
        return limitSwitchBeamBrake.get();
    }


    public Command runFeeder(double targetSpeed){
        return run(()-> {
            setSpeed(targetSpeed);
        });
    }

    @Override
    public void periodic() {
        System.out.println("This the power of the feeder: " + getSpeed());
        System.out.println("This is the state of the beam brake: " + getBeamBrakeState());
    }

}
