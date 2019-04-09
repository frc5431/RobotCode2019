package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.robot.util.Titan;

public class Intake extends Titan.Component<Robot>{
    public static enum FingerState{
        DEPLOYED, RETRACTED
    };

    public static enum JayState{
        DEPLOYED, RETRACTED
    };

    private final CANSparkMax rollers;

    private final Titan.Solenoid jay;
    private final Titan.DoubleSolenoid finger;

    private ControlMode controlMode = ControlMode.MANUAL;

    private JayState jayState = JayState.DEPLOYED;
    private FingerState fingerState = FingerState.DEPLOYED;
    private double rollerSpeed = 0.0;

    public Intake(){
        rollers = new CANSparkMax(Constants.INTAKE_ROLLER_ID, MotorType.kBrushed);
        rollers.setInverted(Constants.INTAKE_ROLLER_INVERTED);
        rollers.setIdleMode(IdleMode.kBrake);

        finger = new Titan.DoubleSolenoid(Constants.INTAKE_FINGER_PCM_ID, Constants.INTAKE_FINGER_FORWARD_ID, Constants.INTAKE_FINGER_REVERSE_ID);

        jay = new Titan.Solenoid(Constants.INTAKE_JAY_PCM_ID, Constants.INTAKE_JAY_ID);
    }

    @Override
    public void init(final Robot robot){
        
    }

    @Override
    public void periodic(final Robot robot){
        rollers.set(rollerSpeed);
        
        finger.set(fingerState == FingerState.DEPLOYED ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);

        jay.set(jayState == JayState.RETRACTED);
    }

    @Override
    public void disabled(final Robot robot){
        
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
        return rollers.getOutputCurrent() >= Constants.ROLLER_BALL_AMPS;
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