package frc.robot.components;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Component;
import frc.robot.util.Testable;

public class Elevator extends Component{
    public static enum BrakeState{
        ENGAGED, DISENGAGED
    };

    private final WPI_TalonSRX bottom, top;

    private final Solenoid brakePad;

    //private final DigitalInput carriageUp;
    private final DigitalInput carriageDown1;
    private final DigitalInput elevatorDown1;

    private long lastBrake = -1;

    //having to use the fully qualified name as it would conflict with the TalonSRX ControlMode otherwise
    private frc.robot.util.ControlMode controlMode = frc.robot.util.ControlMode.MANUAL;

    private BrakeState brakeState = BrakeState.ENGAGED;

    private double elevPower = 0.0;
    private int targetPosition = -1;

    private int lastEncoderPosition = 0;

    public Elevator(){
        bottom = new WPI_TalonSRX(Constants.ELEVATOR_BOTTOM_ID);
        bottom.setInverted(Constants.ELEVATOR_BOTTOM_INVERTED);
        bottom.configForwardSoftLimitEnable(false);
        bottom.configReverseSoftLimitEnable(false);
        bottom.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        bottom.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        bottom.configClearPositionOnLimitF(false, 0);
        bottom.configClearPositionOnLimitR(false, 0);
        bottom.setNeutralMode(NeutralMode.Brake);
        
        bottom.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        final double peakSensorVelocity = 5000;
        //to calculate kF, move elevator to second stage, run at 100%, and print getSelectedSensorVelocity

        bottom.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
        bottom.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

        bottom.selectProfileSlot(0, 0);
        bottom.setSensorPhase(false);
        bottom.config_kP(0, 4.4, 0);//0.05
		bottom.config_kI(0, 0.004, 0);
		bottom.config_kD(0, 40, 0);
        bottom.config_kF(0, 1023.0 / peakSensorVelocity, 0);
        bottom.configAllowableClosedloopError(0, 0/*Constants.ELEVATOR_POSITION_TOLERANCE*/, 0);
		bottom.config_IntegralZone(0, 200, 0);
        bottom.configMotionAcceleration((int)(1.5 * peakSensorVelocity));
        bottom.configMotionCruiseVelocity((int)(1.0 * peakSensorVelocity));
        bottom.configClosedLoopPeakOutput(0, 1);
        //bottom.configClosedLoopPeakOutput(0, Constants.kGains_Distanc.kPeakOutput, 0);
        
        // bottom.disableNominalClosedLoopVoltage();
        // bottom.setNominalClosedLoopVoltage(12.0);


        top = new WPI_TalonSRX(Constants.ELEVATOR_TOP_ID);
        top.setInverted(Constants.ELEVATOR_TOP_INVERTED);
        top.set(ControlMode.Follower, bottom.getDeviceID());
        top.setNeutralMode(NeutralMode.Brake);

        brakePad = new Solenoid(Constants.ELEVATOR_BRAKE_PCM_ID, Constants.ELEVATOR_BRAKE_ID);

        //carriageUp = new DigitalInput(Constants.ELEVATOR_CARRIAGE_UP_PORT);

        elevatorDown1 = new DigitalInput(Constants.ELEVATOR_DOWN_1_PORT);

        carriageDown1 = new DigitalInput(Constants.ELEVATOR_CARRIAGE_DOWN_1_PORT);

        //hi tauseef
    }

    @Override
    public void init(final Robot robot){
        
    }

    @Override
    public void periodic(final Robot robot){
        //in case of an encoder brownout, restore encoder position
        if(bottom.hasResetOccurred()){
            bottom.getSensorCollection().setQuadraturePosition(lastEncoderPosition, 0);
        }

        /*if(isCarriageUp() && isElevatorDown()){
            bottom.getSensorCollection().setQuadraturePosition(27000, 0);
        }else*/if(isCarriageDown() && isElevatorDown()){
            bottom.getSensorCollection().setQuadraturePosition(0, 0);
        }

        lastEncoderPosition = getEncoderPosition();

        if(targetPosition >= 0){
            elevPower = bottom.get();
        }

        System.out.println(elevPower + ", " + targetPosition + ", " + bottom.getSelectedSensorVelocity());

        if(targetPosition < 0 && elevPower == 0 /*|| (val < 0 && Titan.approxEquals(getEncoderPosition(), 0, 3)) || (val > 0 && isUp())*/){
            bottom.set(0);
            brake(isCarriageDown() ? BrakeState.DISENGAGED : BrakeState.ENGAGED);
        }else{
            //in order to fix the brake issue, move the error checkng to outside the if statement
            //if the elevator is going up, or the elevator is going down and it has waited long enough for the break to disengage
            if(System.currentTimeMillis() >= lastBrake + Constants.ELEVATOR_BRAKE_TIME){
                if(isCarriageDown() && elevPower < 0){
                    elevPower = 0;
                    brake(BrakeState.DISENGAGED);
                }else if((getEncoderPosition() > Constants.ELEVATOR_TOP_LIMIT && elevPower > 0)){
                    elevPower = 0;
                    brake(BrakeState.ENGAGED);
                }else{
                    brake(BrakeState.DISENGAGED);
                }
                if(targetPosition >= 0){
                    bottom.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, 0.0);//0.35
                }else{
                    bottom.set(ControlMode.PercentOutput, elevPower < 0 ? elevPower * Constants.ELEVATOR_DOWN_MULTIPLIER : elevPower);
                }
            }else{
                //at this point, the elevater wants to move down but it hasn't disengaged the break yet
                brake(BrakeState.DISENGAGED);
                //have to move the elevator slightly up to engage the break
                bottom.set(Constants.ELEVATOR_BRAKE_UP_SPEED);
            }
        }

        if(brakeState == BrakeState.DISENGAGED){
            if(lastBrake < 0){
                lastBrake = System.currentTimeMillis();
            }
        }else{
            lastBrake = -1;
        }
        brakePad.set(brakeState == BrakeState.DISENGAGED);
    }

    @Override
    public void disabled(final Robot robot){
        
    }

    public void elevate(final double val){
        elevPower = val;
        targetPosition = -1;
        //right.set(val);
    }

    public void elevateTo(final int pos){
        elevPower = 0;
        targetPosition = pos;
    }

    public void brake(final BrakeState state){
        brakeState = state;
    }

    public frc.robot.util.ControlMode getControlMode(){
        return controlMode;
    }

    public BrakeState getBrakeState(){
        return brakeState;
    }

    public void setControlMode(final frc.robot.util.ControlMode mode){
        controlMode = mode;
    }

    public int getEncoderPosition(){
        return bottom.getSensorCollection().getQuadraturePosition();
    }

    public int getEncoderVelocity(){
        return bottom.getSensorCollection().getQuadratureVelocity();
    }

    public boolean isElevatorDown(){
        return !elevatorDown1.get();
    }

    public boolean isCarriageUp(){
        return /*!carriageUp.get() || */getEncoderPosition() > Constants.ELEVATOR_FIRST_STAGE_LIMIT;
    }

    public boolean isCarriageDown(){
        return !carriageDown1.get();
    }

    @Override
    public String getTestResult(){
        if(isCarriageDown()&&isCarriageUp()){
            return "Carriage sensor mismatch";
        }else if(isCarriageDown() && getEncoderPosition() != 0){
            return "Bottom limit switch logic failure";
        }
        return Testable.SUCCESS;
    }
}