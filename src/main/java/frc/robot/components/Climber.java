package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Titan;

public class Climber extends Titan.Component<Robot>{
    public static enum ForkState{
        DEPLOYED, RETRACTED
    }

    private final CANSparkMax left, right;

    private final PWMVictorSPX leftWinch, rightWinch;

    private final SpeedControllerGroup winches;

    private final Titan.Solenoid forks;

    private double climbSpeed = 0.0, winchSpeed = 0.0;

    private ForkState forkState = ForkState.RETRACTED;
    
    public Climber(){
        left = new CANSparkMax(Constants.CLIMBER_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        left.setInverted(Constants.CLIMBER_LEFT_INVERTED);
        left.setIdleMode(IdleMode.kBrake);
        left.burnFlash();

        right = new CANSparkMax(Constants.CLIMBER_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        right.setInverted(Constants.CLIMBER_RIGHT_INVERTED);
        right.follow(left, Constants.CLIMBER_RIGHT_INVERTED);
        right.setIdleMode(IdleMode.kBrake);
        right.burnFlash();

        leftWinch = new PWMVictorSPX(Constants.CLIMBER_WINCH_LEFT_ID);
        leftWinch.setInverted(Constants.CLIMBER_WINCH_LEFT_INVERTED);
        
        rightWinch = new PWMVictorSPX(Constants.CLIMBER_WINCH_RIGHT_ID);
        rightWinch.setInverted(Constants.CLIMBER_WINCH_RIGHT_INVERTED);

        winches = new SpeedControllerGroup(leftWinch, rightWinch);

        forks = new Titan.Solenoid(Constants.CLIMBER_FORK_PCM_ID, Constants.CLIMBER_FORK_ID);
    }
    
    @Override
    public void init(final Robot robot){
        
    }

    @Override
    public void periodic(final Robot robot){
        left.set(climbSpeed);

        winches.set(winchSpeed);

        forks.set(forkState == ForkState.DEPLOYED);
    }
    
    @Override
    public void disabled(final Robot robot){
        
    }
    
    public void climb(final double val){
        climbSpeed = val;
        //right.set(0.0);
    }

    public void winch(final double val){
        winchSpeed = val;
    }

    public void fork(final ForkState state){
        forkState = state;
    }

    public ForkState getForkState(){
        return forkState;
    }
}