package frc.robot.components;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;

public class Elevator{
    private final WPI_TalonSRX bottom, top;

    private final Solenoid brakePad;

    private final DigitalInput carriageUp;
    private final DigitalInput carriageDown1;
    private final DigitalInput elevatorDown1;

    private long lastBrake = -1;

    private frc.robot.ControlMode controlMode = frc.robot.ControlMode.MANUAL;

    private boolean isBraking = true;

    public Elevator(){
        bottom = new WPI_TalonSRX(Constants.ELEVATOR_BOTTOM_ID);
        bottom.setInverted(Constants.ELEVATOR_BOTTOM_INVERTED);
        bottom.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        bottom.configForwardSoftLimitEnable(false);
        bottom.configReverseSoftLimitEnable(false);
        bottom.setNeutralMode(NeutralMode.Brake);
        
        bottom.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        final double peakSensorVelocity = 9000;
        //to calculate kF, move elevator to second stage, run at 100%, and print getSelectedSensorVelocity

        // bottom.config_kP(0, Constants.kGains_Distanc.kP, 00);
		// bottom.config_kI(0, Constants.kGains_Distanc.kI, 0);
		// bottom.config_kD(0, Constants.kGains_Distanc.kD, 0);
		// bottom.config_kF(0, 0, 0);
		// bottom.config_IntegralZone(0, Constants.kGains_Distanc.kIzone, 0);
        // bottom.configClosedLoopPeakOutput(0, Constants.kGains_Distanc.kPeakOutput, 0);

        top = new WPI_TalonSRX(Constants.ELEVATOR_TOP_ID);
        top.setInverted(Constants.ELEVATOR_TOP_INVERTED);
        top.set(ControlMode.Follower, bottom.getDeviceID());
        top.setNeutralMode(NeutralMode.Brake);

        brakePad = new Solenoid(Constants.ELEVATOR_BRAKE_PCM_ID, Constants.ELEVATOR_BRAKE_ID);

        carriageUp = new DigitalInput(Constants.ELEVATOR_CARRIAGE_UP_PORT);

        elevatorDown1 = new DigitalInput(Constants.ELEVATOR_DOWN_1_PORT);

        carriageDown1 = new DigitalInput(Constants.ELEVATOR_CARRIAGE_DOWN_1_PORT);

        //hi tauseef
    }

    public void periodic(final Robot robot){
        if(isCarriageUp() && isElevatorDown()){
            bottom.getSensorCollection().setQuadraturePosition(27000, 0);
        }else if(isCarriageDown()){
            bottom.getSensorCollection().setQuadraturePosition(0, 0);
        }

        if(!isBraking){
            if(lastBrake < 0){
                lastBrake = System.currentTimeMillis();
            }
        }else{
            lastBrake = -1;
        }
        brakePad.set(!isBraking);
    }

    public void elevate(double val){
        if(val == 0 /*|| (val < 0 && Titan.approxEquals(getEncoderPosition(), 0, 3)) || (val > 0 && isUp())*/){
            bottom.set(0);
            brake(true);
        }else{
            //in order to fix the brake issue, move the error checkng to outside the if statement
            if(System.currentTimeMillis() >= lastBrake + Constants.ELEVATOR_BRAKE_TIME){
                if((isCarriageDown() && val < 0) || (getEncoderPosition() > Constants.ELEVATOR_TOP_LIMIT && val > 0) || (getEncoderPosition() <= Constants.ELEVATOR_BOTTOM_LIMIT && val < 0)){
                    val = 0;
                    brake(true);
                }else{
                    brake(false);
                }
                bottom.set(val < 0 ? val * Constants.ELEVATOR_DOWN_MULTIPLIER : val);
            }else{
                brake(false);
                if(val < 0){
                    bottom.set(Constants.ELEVATOR_BRAKE_UP_SPEED);
                }else{
                    bottom.set(Constants.ELEVATOR_BRAKE_DOWN_SPEED);
                }
            }
        }
        //right.set(val);
    }

    public void brake(final boolean braked){
        isBraking = braked;
    }

    public frc.robot.ControlMode getControlMode(){
        return controlMode;
    }

    public void setControlMode(final frc.robot.ControlMode mode){
        controlMode = mode;
    }

    public int getEncoderPosition(){
        return bottom.getSensorCollection().getQuadraturePosition();
    }

    public boolean isElevatorDown(){
        return !elevatorDown1.get();
    }

    public boolean isCarriageUp(){
        return !carriageUp.get();
    }

    public boolean isCarriageDown(){
        return !carriageDown1.get();
    }
}