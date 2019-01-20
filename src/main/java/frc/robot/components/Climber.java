package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.robot.Constants;

public class Climber {
    private final CANSparkMax left, right;
    
    public Climber(){
        left = new CANSparkMax(Constants.CLIMBER_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        left.setInverted(Constants.CLIMBER_LEFT_INVERTED);

        right = new CANSparkMax(Constants.CLIMBER_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        right.setInverted(Constants.CLIMBER_RIGHT_INVERTED);
    }

    public void climb(final double val){
        left.set(val);
        right.set(val);
    }
}