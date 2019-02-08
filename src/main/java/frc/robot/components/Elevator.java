package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class Elevator{
    private final WPI_TalonSRX left, right;

    private final DoubleSolenoid brakePad;

    private final AnalogInput encoder;

    public Elevator(){
        left = new WPI_TalonSRX(Constants.ELEVATOR_LEFT_ID);
        left.setInverted(Constants.ELEVATOR_LEFT_INVERTED);
        
        right = new WPI_TalonSRX(Constants.ELEVATOR_RIGHT_ID);
        right.setInverted(Constants.ELEVATOR_RIGHT_INVERTED);

        brakePad = new DoubleSolenoid(Constants.ELEVATOR_BRAKE_FORWARD_ID, Constants.ELEVATOR_BRAKE_REVERSE_ID);

        encoder = new AnalogInput(Constants.ELEVATOR_ENCODER_PORT);
    }

    public void elevate(final double val){
        left.set(val);
        right.set(val);
    }

    public void setBrake(final boolean braked){
        brakePad.set(braked ? Value.kForward : Value.kReverse);
    }

    public double getEncoderPosition(){
        return (encoder.getAverageVoltage() / 5.0) * 360.0;
    }
}