package frc.robot.components;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Elevator{
    private final WPI_TalonSRX bottom, top;

    private final DoubleSolenoid brakePad;

    private final DigitalInput carriageUp;

    public Elevator(){
        bottom = new WPI_TalonSRX(Constants.ELEVATOR_BOTTOM_ID);
        bottom.setInverted(Constants.ELEVATOR_BOTTOM_INVERTED);
        bottom.configClearPositionOnLimitR(true, 0);
        bottom.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        bottom.configForwardSoftLimitEnable(false);
        bottom.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        bottom.configReverseSoftLimitEnable(false);

        top = new WPI_TalonSRX(Constants.ELEVATOR_TOP_ID);
        top.setInverted(Constants.ELEVATOR_TOP_INVERTED);
        top.set(ControlMode.Follower, bottom.getDeviceID());

        brakePad = new DoubleSolenoid(Constants.ELEVATOR_BRAKE_PCM_ID, Constants.ELEVATOR_BRAKE_FORWARD_ID, Constants.ELEVATOR_BRAKE_REVERSE_ID);
    
        carriageUp = new DigitalInput(Constants.ELEVATOR_CARRIAGE_UP_PORT);
    }

    public void elevate(final double val){
        brake(val == 0);
        bottom.set(val);
        //right.set(val);
    }

    public void brake(final boolean braked){
        brakePad.set(braked ? Value.kForward : Value.kReverse);
    }

    public int getEncoderPosition(){
        return bottom.getSensorCollection().getQuadraturePosition();
    }

    public boolean isUp(){
        return bottom.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean isDown(){
        return bottom.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean isCarriageUp(){
        return carriageUp.get();
    }
}