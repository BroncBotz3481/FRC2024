package frc.robot.subsystems.Feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

    private final CANSparkMax feederMotor;

    private final DigitalInput beamBrake;

    private final CANSparkMax intakeMotor;


    public FeederSubsystem() {
        feederMotor = new CANSparkMax(Constants.FeederConstants.feederMotorID, CANSparkLowLevel.MotorType.kBrushless);
        feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        beamBrake = new DigitalInput(Constants.FeederConstants.beamBrakeChannel);

        intakeMotor = new CANSparkMax(Constants.FeederConstants.intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeMotor.setInverted(false);
    }

    public void setSpeed(double fPower, double iPower) {
        feederMotor.set(fPower);
        intakeMotor.set(iPower);
    }

    public void stopFeeder() {
        feederMotor.set(0);
        intakeMotor.set(0);
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
        return !beamBrake.get();
    }


    public Command runFeeder(double fTargetSpeed, double iTargetSpeed, boolean ignoreBeambreak){
         // This assumes that beambreak == true when note is present. If beambreak == false when note is present, add a !
        return run(() -> {
            if(ignoreBeambreak || !getBeamBrakeState())  // This assumes that beambreak == true when note is present. If beambreak == false when note is present, add a !
                setSpeed(fTargetSpeed, iTargetSpeed);
            else 
                stopFeeder();   
            }); 
    }

    public Command runFeederCommand(double fTargetSpeed, double iTargetSpeed)
    {
       return run(() -> {setSpeed(fTargetSpeed, iTargetSpeed);}).until(this::getBeamBrakeState).andThen(runOnce(this::stopFeeder));
    }

    // public void runFeederAuto(double fTargetSpeed, double iTargetSpeed, boolean ignoreBeambreak){
    //         if(ignoreBeambreak || !getBeamBrakeState()) { // This assumes that beambreak == true when note is present. If beambreak == false when note is present, add a !
    //             setSpeed(fTargetSpeed, iTargetSpeed);
    //         } else {
    //             stopFeeder();
    //         }
    //     }

    @Override
    public void periodic() {
        //System.out.println("This the power of the feeder: " + getSpeed());
        // System.out.println("This is the state of the beam brake: " + getBeamBrakeState());
    }

}

