package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.ControlMode;
import frc.robot.Robot;

public class Arm{
    final CANSparkMax pivot;
    final Solenoid brakePad;

    final DoubleSolenoid wristLeft, wristRight;

    final AnalogInput wristEncoder;

    private ControlMode controlMode = ControlMode.MANUAL;

    private boolean isWristing = true;

    public Arm(){
        pivot = new CANSparkMax(Constants.ARM_PIVOT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivot.setInverted(Constants.ARM_PIVOT_INVERTED);
        pivot.setIdleMode(IdleMode.kBrake);
    
        brakePad = new Solenoid(Constants.ARM_BRAKE_PCM_ID, Constants.ARM_BRAKE_ID);

        wristLeft = new DoubleSolenoid(Constants.ARM_WRIST_LEFT_PCM_ID, Constants.ARM_WRIST_LEFT_FORWARD_ID, Constants.ARM_WRIST_LEFT_REVERSE_ID);
        wristRight = new DoubleSolenoid(Constants.ARM_WRIST_RIGHT_PCM_ID, Constants.ARM_WRIST_RIGHT_FORWARD_ID, Constants.ARM_WRIST_RIGHT_REVERSE_ID);
    
        wristEncoder = new AnalogInput(Constants.WRIST_ENCODER_PORT);
    }

    public void periodic(final Robot robot){
        wristLeft.set(isWristing ? Value.kForward : Value.kReverse);
        wristRight.set(isWristing ? Value.kForward : Value.kReverse);
    }

    public void pivot(final double val){
        //if the value of the pivot is 0 (so stopped), automatically break
        brake(val == 0);
        pivot.set(val /*+ (Math.signum(val) * 0.3 * Math.cos(Math.signum(val) * Math.toRadians(getWristPosition() - 90)))*/);
    }

    public void brake(final boolean val){
        brakePad.set(!val);
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

    public double getWristPosition(){
        return (((wristEncoder.getAverageVoltage() / 5.0) * 360.0) - 74) % 360;
    }

    public boolean isWristing(){
        return isWristing;
    }
}