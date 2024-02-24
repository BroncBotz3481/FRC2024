package frc.robot.subsystems.Feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

    private final CANSparkMax feederMotor;

    //private final DigitalInput beamBrake;


    public FeederSubsystem() {
        feederMotor = new CANSparkMax(Constants.FeederConstants.feederMotorID, CANSparkLowLevel.MotorType.kBrushless);
        feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //beamBrake = new DigitalInput(Constants.FeederConstants.beamBrakeChannel);
    }

    public void setSpeed(double fPower) {
        feederMotor.set(fPower);
    }

    public void stopFeeder() {
        feederMotor.set(0);
    }

    public enum FeederState {

        FORWARD(0.5),
        OFF(0),
        REVERSE(-0.5);

       public double power;

       private FeederState(double power){
           this.power = power;
       }

    }

    public double getSpeed(){
        return feederMotor.get();
    }


    public boolean getBeamBrakeState(){
        return true;
        //return beamBrake.get();
    }


    public Command runFeeder(double targetSpeed){
        return run(()-> {
            setSpeed(targetSpeed);
        });
    }

    @Override
    public void periodic() {
        //System.out.println("This the power of the feeder: " + getSpeed());
        //System.out.println("This is the state of the beam brake: " + getBeamBrakeState());
    }

}

