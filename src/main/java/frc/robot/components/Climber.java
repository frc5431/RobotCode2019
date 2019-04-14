package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Titan;

public class Climber extends Titan.Component<Robot>{
    public static enum ForkState{
        DEPLOYED, RETRACTED
    }

    private final CANSparkMax left, right;

    private final Titan.Solenoid forks;

    private double climbSpeed = 0.0;

    private ForkState forkState = ForkState.RETRACTED;
    
    public Climber(){
        left = new CANSparkMax(Constants.CLIMBER_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        left.setInverted(Constants.CLIMBER_LEFT_INVERTED);
        left.setIdleMode(IdleMode.kBrake);
        left.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        left.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        left.burnFlash();

        right = new CANSparkMax(Constants.CLIMBER_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        right.setInverted(Constants.CLIMBER_RIGHT_INVERTED);
        right.follow(left, Constants.CLIMBER_RIGHT_INVERTED);
        right.setIdleMode(IdleMode.kBrake);
        right.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        right.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        right.burnFlash();

        forks = new Titan.Solenoid(Constants.CLIMBER_FORK_PCM_ID, Constants.CLIMBER_FORK_ID);
    }
    
    @Override
    public void init(final Robot robot){
        
    }

    @Override
    public void periodic(final Robot robot){
        if(right.getFault(FaultID.kHasReset)){
            right.follow(right, Constants.CLIMBER_RIGHT_INVERTED);
            right.clearFaults();
        }

        left.set(climbSpeed);

        forks.set(forkState == ForkState.DEPLOYED);
    }
    
    @Override
    public void disabled(final Robot robot){
        
    }
    
    public void climb(final double val){
        climbSpeed = val;
        //right.set(0.0);
    }

    public void fork(final ForkState state){
        forkState = state;
    }

    public ForkState getForkState(){
        return forkState;
    }
}