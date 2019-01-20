package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class Arm{
    final CANSparkMax pivot;
    final Solenoid brakePad;

    public Arm(){
        pivot = new CANSparkMax(Constants.ARM_PIVOT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivot.setInverted(Constants.ARM_PIVOT_INVERTED);
    
        brakePad = new Solenoid(Constants.ARM_BRAKE_ID);
    }

    public void pivot(final double val){
        pivot.set(val);
    }

    public void brake(final boolean val){
        brakePad.set(val);
    }
}