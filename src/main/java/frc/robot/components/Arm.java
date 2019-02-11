package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class Arm{
    final CANSparkMax pivot;
    final Solenoid brakePad;

    final DoubleSolenoid wristLeft, wristRight;

    final AnalogInput wristEncoder;

    public Arm(){
        pivot = new CANSparkMax(Constants.ARM_PIVOT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivot.setInverted(Constants.ARM_PIVOT_INVERTED);
    
        brakePad = new Solenoid(Constants.ARM_BRAKE_PCM_ID, Constants.ARM_BRAKE_ID);

        wristLeft = new DoubleSolenoid(Constants.ARM_WRIST_LEFT_PCM_ID, Constants.ARM_WRIST_LEFT_FORWARD_ID, Constants.ARM_WRIST_LEFT_REVERSE_ID);
        wristRight = new DoubleSolenoid(Constants.ARM_WRIST_RIGHT_PCM_ID, Constants.ARM_WRIST_RIGHT_FORWARD_ID, Constants.ARM_WRIST_RIGHT_REVERSE_ID);
    
        wristEncoder = new AnalogInput(Constants.WRIST_ENCODER_PORT);
    }

    public void pivot(final double val){
        //if the value of the pivot is 0 (so stopped), automatically break
        brake(val == 0);
        pivot.set(val);
    }

    public void brake(final boolean val){
        brakePad.set(val);
    }

    public void wrist(final boolean val){
        wristLeft.set(val ? Value.kForward : Value.kReverse);
        wristRight.set(val ? Value.kForward : Value.kReverse);
    }

    public double getWristPosition(){
        return (wristEncoder.getAverageVoltage() / 5.0) * 360.0;
    }
}