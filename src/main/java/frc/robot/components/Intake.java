package frc.robot.components;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ControlMode;

public class Intake extends SubsystemBase {
    public static enum FingerState{
        DEPLOYED, RETRACTED
    };

    public static enum JayState{
        DEPLOYED, RETRACTED
    };

    private final WPI_TalonSRX rollers;

    private final Solenoid jay;
    private final DoubleSolenoid finger;

    private ControlMode controlMode = ControlMode.MANUAL;

    private JayState jayState = JayState.DEPLOYED;
    private FingerState fingerState = FingerState.DEPLOYED;
    private double rollerSpeed = 0.0;

    public Intake(){
        rollers = new WPI_TalonSRX(Constants.INTAKE_ROLLER_ID);
        rollers.setInverted(Constants.INTAKE_ROLLER_INVERTED);
        rollers.setNeutralMode(NeutralMode.Brake);

        finger = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_FINGER_FORWARD_ID, Constants.INTAKE_FINGER_REVERSE_ID);

        jay = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_JAY_ID);
    }

    @Override
    public void periodic(){
        rollers.set(rollerSpeed);
        
        finger.set(fingerState == FingerState.DEPLOYED ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
        System.out.println(finger.get());

        jay.set(jayState == JayState.RETRACTED);
    }

    public void roll(final double val){
        rollerSpeed = val;
    }

    public void finger(final FingerState val){
        fingerState = val;
    }

    public void jay(final JayState val){
        if(val == JayState.RETRACTED){
            finger(FingerState.RETRACTED);
        }
        jayState = val;
    }

    public JayState getJayState(){
        return jayState;
    }

    public FingerState getFingerState(){
        return fingerState;
    }

    public boolean isBallIn(){
        return rollers.getSupplyCurrent() >= Constants.ROLLER_BALL_AMPS;
    }

    public boolean isRolling(){
        return rollerSpeed != 0;
    }

    public boolean canShootHatch(){
        return true;
    }

    public ControlMode getControlMode(){
        return controlMode;
    }

    public void setControlMode(final ControlMode mode){
        controlMode = mode;
    }
}