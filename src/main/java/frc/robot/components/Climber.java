package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;

import frc.robot.Constants;

public class Climber {
    private final CANSparkMax left, right;

    private final AnalogInput encoder;
    
    public Climber(){
        left = new CANSparkMax(Constants.CLIMBER_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        left.setInverted(Constants.CLIMBER_LEFT_INVERTED);
        left.setIdleMode(IdleMode.kBrake);

        right = new CANSparkMax(Constants.CLIMBER_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        right.setInverted(Constants.CLIMBER_RIGHT_INVERTED);
        right.follow(left, Constants.CLIMBER_RIGHT_INVERTED);
        right.setIdleMode(IdleMode.kBrake);
        
        encoder = new AnalogInput(Constants.CLIMBER_ENCODER_PORT);
    }

    public void climb(final double val){
        left.set(val);
        //right.set(0.0);
    }

    public double getEncoderPosition(){
        return (encoder.getAverageVoltage() / 5.0) * 360.0;
    }
}