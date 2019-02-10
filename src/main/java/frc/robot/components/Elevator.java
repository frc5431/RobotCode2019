package frc.robot.components;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Elevator{
    private final WPI_TalonSRX left, right;

    private final DoubleSolenoid brakePad;

    private final DigitalInput carriageUp;

    public Elevator(){
        left = new WPI_TalonSRX(Constants.ELEVATOR_LEFT_ID);
        left.setInverted(Constants.ELEVATOR_LEFT_INVERTED);
        left.configClearPositionOnLimitR(true, 0);
        left.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        left.configForwardSoftLimitEnable(true);
        left.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        left.configReverseSoftLimitEnable(true);
        
        right = new WPI_TalonSRX(Constants.ELEVATOR_RIGHT_ID);
        right.setInverted(Constants.ELEVATOR_RIGHT_INVERTED);
        right.set(ControlMode.Follower, left.getDeviceID());

        brakePad = new DoubleSolenoid(Constants.ELEVATOR_BRAKE_PCM_ID, Constants.ELEVATOR_BRAKE_FORWARD_ID, Constants.ELEVATOR_BRAKE_REVERSE_ID);
    
        carriageUp = new DigitalInput(Constants.ELEVATOR_CARRIAGE_UP_PORT);
    }

    public void elevate(final double val){
        brake(val == 0);
        left.set(val);
        //right.set(val);
    }

    public void brake(final boolean braked){
        brakePad.set(braked ? Value.kForward : Value.kReverse);
    }

    public int getEncoderPosition(){
        return left.getSensorCollection().getQuadraturePosition();
    }

    public boolean isUp(){
        return left.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean isDown(){
        return left.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean isCarriageUp(){
        return carriageUp.get();
    }
}