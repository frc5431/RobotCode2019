package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.wpilib.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public static enum ForkState{
        DEPLOYED, RETRACTED
    }

    private final CANSparkMax left, right;

    private final Solenoid forks;

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

        forks = new Solenoid(Constants.CLIMBER_FORK_ID);
    }

    @Override
    public void periodic(){
        if(right.getFault(FaultID.kHasReset)){
            right.follow(right, Constants.CLIMBER_RIGHT_INVERTED);
            right.clearFaults();
        }

        left.set(climbSpeed);

        forks.set(forkState == ForkState.DEPLOYED);
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