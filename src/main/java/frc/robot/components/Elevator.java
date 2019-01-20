package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class Elevator{
    private final WPI_TalonSRX frontLeft, frontRight, backLeft, backRight;

    private final DoubleSolenoid brakePad;

    public Elevator(){
        frontLeft = new WPI_TalonSRX(Constants.ELEVATOR_FRONT_LEFT_ID);
        frontLeft.setInverted(Constants.ELEVATOR_FRONT_LEFT_INVERTED);
        
        frontRight = new WPI_TalonSRX(Constants.ELEVATOR_FRONT_LEFT_ID);
        frontRight.setInverted(Constants.ELEVATOR_FRONT_LEFT_INVERTED);
        
        backLeft = new WPI_TalonSRX(Constants.ELEVATOR_BACK_LEFT_ID);
        backLeft.setInverted(Constants.ELEVATOR_BACK_LEFT_INVERTED);

        backRight = new WPI_TalonSRX(Constants.ELEVATOR_BACK_RIGHT_ID);
        backRight.setInverted(Constants.ELEVATOR_BACK_RIGHT_INVERTED);

        brakePad = new DoubleSolenoid(Constants.ELEVATOR_BRAKE_FORWARD_ID, Constants.ELEVATOR_BRAKE_REVERSE_ID);
    }

    public void elevate(final double val){
        frontLeft.set(val);
        frontRight.set(val);
        backLeft.set(val);
        backRight.set(val);
    }

    public void setBrake(final boolean braked){
        brakePad.set(braked ? Value.kForward : Value.kReverse);
    }
}