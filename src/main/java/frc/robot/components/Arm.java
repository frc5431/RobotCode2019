package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.ControlMode;
import frc.robot.Robot;

public class Arm{
    final CANSparkMax pivot;
    final Solenoid brakePad;

    final Solenoid wrist;

    final AnalogInput armEncoder;

    private ControlMode controlMode = ControlMode.MANUAL;

    private boolean isWristing = true, isBraking = true;

    public Arm(){
        pivot = new CANSparkMax(Constants.ARM_PIVOT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivot.setInverted(Constants.ARM_PIVOT_INVERTED);
        pivot.setIdleMode(IdleMode.kBrake);
    
        brakePad = new Solenoid(Constants.ARM_BRAKE_PCM_ID, Constants.ARM_BRAKE_ID);

        wrist = new Solenoid(Constants.ARM_WRIST_PCM_ID, Constants.ARM_WRIST_ID);
    
        armEncoder = new AnalogInput(Constants.ARM_ENCODER_PORT);
    }

    public void periodic(final Robot robot){
        wrist.set(!isWristing);

        //solenoid is inverted
        brakePad.set(!isBraking);
    }

    public void pivot(final double in){
        double val = in;
        if(getArmAngle() > 250 && in > 0){
            val = 0;
        }
        //if the value of the pivot is 0 (so stopped), automatically break
        brake(val == 0);
        // System.out.println((0.3 * Math.cos(Math.toRadians(getWristPosition() - 90))));
        // USE THIS FOR BETTER EQUATION:
        // pivot.set(val + (0.3 * (Math.signum(val) * Math.cos(Math.toRadians(getArmAngle() - 90)))));
        pivot.set(Math.signum(val) * (Math.abs(val) + (0.3 * Math.abs(Math.cos(Math.toRadians(getArmAngle() - 90))))));
    }

    public void brake(final boolean val){
        isBraking = val;
    }

    public void wrist(final boolean val){
        isWristing = val;
    }

    public ControlMode getControlMode(){
        return controlMode;
    }

    public void setControlMode(final ControlMode mode){
        controlMode = mode;
    }

    public double getArmAngle(){
        final double position = (((armEncoder.getAverageVoltage() / 5.0) * 360.0) - Constants.ARM_ENCODER_CALIBRATION_OFFSET) % 360;
        if(position < 0){
            return 360 + position;
        }
        return position;
    }

    public boolean isWristing(){
        return isWristing;
    }
}