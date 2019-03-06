package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Component;
import frc.robot.util.Testable;

public class Climber extends Component{
    private final CANSparkMax left, right;
    
    public Climber(){
        left = new CANSparkMax(Constants.CLIMBER_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        left.setInverted(Constants.CLIMBER_LEFT_INVERTED);
        left.setIdleMode(IdleMode.kBrake);

        right = new CANSparkMax(Constants.CLIMBER_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        right.setInverted(Constants.CLIMBER_RIGHT_INVERTED);
        right.follow(left, Constants.CLIMBER_RIGHT_INVERTED);
        right.setIdleMode(IdleMode.kBrake);
    }
    
    @Override
    public void init(final Robot robot){
        
    }

    @Override
    public void periodic(final Robot robot){
    }
    
    @Override
    public void disabled(final Robot robot){
        
    }
    
    public void climb(final double val){
        left.set(val);
        //right.set(0.0);
    }

    @Override
    public String getTestResult(){
        return Testable.SUCCESS;
    }
}