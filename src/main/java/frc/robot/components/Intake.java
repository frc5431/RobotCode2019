package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.robot.util.Component;
import frc.robot.util.Testable;

public class Intake extends Component{
    public static enum FingerState{
        DEPLOYED, RETRACTED
    };

    public static enum JayState{
        DEPLOYED, RETRACTED
    };

    public static enum HatchState{
        UP, DOWN
    };

    private final WPI_TalonSRX rollers;

    //private final Solenoid hatchLeft, hatchRight, finger;
    private final Solenoid hatch, jay;
    private final DoubleSolenoid finger;

    private ControlMode controlMode = ControlMode.MANUAL;

    private HatchState hatchState = HatchState.DOWN;
    private JayState jayState = JayState.RETRACTED;
    private FingerState fingerState = FingerState.DEPLOYED;
    private double rollerSpeed = 0.0;

    public Intake(){
        rollers = new WPI_TalonSRX(Constants.INTAKE_ROLLER_ID);
        rollers.setInverted(Constants.INTAKE_ROLLER_INVERTED);
        rollers.setNeutralMode(NeutralMode.Brake);

        // hatchLeft = new Solenoid(Constants.INTAKE_HATCH_LEFT_PCM_ID, Constants.INTAKE_HATCH_LEFT_ID);
        // hatchRight = new Solenoid(Constants.INTAKE_HATCH_RIGHT_PCM_ID, Constants.INTAKE_HATCH_RIGHT_ID);
        //finger = new Solenoid(Constants.INTAKE_FINGER_PCM_ID, Constants.INTAKE_FINGER_ID);
        finger = new DoubleSolenoid(Constants.INTAKE_FINGER_PCM_ID, Constants.INTAKE_FINGER_ID, 7);

        hatch = new Solenoid(Constants.INTAKE_HATCH_PCM_ID, Constants.INTAKE_HATCH_ID);
        jay = new Solenoid(Constants.INTAKE_JAY_PCM_ID, Constants.INTAKE_JAY_ID);
    }

    @Override
    public void init(final Robot robot){
        
    }

    @Override
    public void periodic(final Robot robot){
        // hatchLeft.set(!isHatching);
        // hatchRight.set(!isHatching);

        rollers.set(rollerSpeed);

        jay.set(jayState == JayState.DEPLOYED);

        hatch.set(hatchState == HatchState.UP);

        finger.set(fingerState == FingerState.RETRACTED ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }

    @Override
    public void disabled(final Robot robot){
        
    }

    public void roll(final double val){
        rollerSpeed = val;
    }

    public void actuateHatch(final HatchState state){
        hatchState = state;
    }

    public void finger(final FingerState val){
        fingerState = val;
    }

    public void jay(final JayState val){
        jayState = val;
    }

    public JayState getJayState(){
        return jayState;
    }

    public FingerState getFingerState(){
        return fingerState;
    }

    public HatchState getHatchState(){
        return hatchState;
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

    @Override
    public String getTestResult(){
        return Testable.SUCCESS;
    }
}