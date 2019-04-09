package frc.robot.components;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.util.ControlMode;
import frc.robot.Robot;
import frc.robot.util.Titan;

public class Arm extends Titan.Component<Robot>{
    public static enum BrakeState{
        ENGAGED, DISENGAGED
    };

    public static enum BrakeMode{
        BREAK, COAST
    };

    final WPI_TalonSRX pivot;
    final Titan.Solenoid brakePad;

    final AnalogInput armEncoder;

    private ControlMode controlMode = ControlMode.MANUAL;

    private BrakeState brakeState = BrakeState.ENGAGED;

    private double armPower = 0.0;

    public Arm(){
        pivot = new WPI_TalonSRX(Constants.ARM_PIVOT_ID);
        pivot.setInverted(Constants.ARM_PIVOT_INVERTED);
        pivot.configForwardSoftLimitEnable(false);
        pivot.configReverseSoftLimitEnable(false);
        pivot.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        pivot.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        pivot.configClearPositionOnLimitF(false, 0);
        pivot.configClearPositionOnLimitR(false, 0);
        
        pivot.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

        pivot.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
        pivot.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

        pivot.selectProfileSlot(0, 0);
        pivot.setSensorPhase(Constants.ELEVATOR_ENCODER_SENSOR_PHASE);
        pivot.config_kP(0, Constants.ARM_MM_P, 0);//7.5
		pivot.config_kI(0, Constants.ARM_MM_I, 0);//0.004
		pivot.config_kD(0, Constants.ARM_MM_D, 0);//40
        pivot.config_kF(0, 1023.0 / Constants.ARM_MM_PEAK_SENSOR_VELOCITY, 0);
        pivot.configAllowableClosedloopError(0, 0/*Constants.ELEVATOR_POSITION_TOLERANCE*/, 0);
		pivot.config_IntegralZone(0, 300, 0);
        pivot.configMotionAcceleration((int)(Constants.ARM_MM_ACCELERATION * Constants.ARM_MM_PEAK_SENSOR_VELOCITY));
        pivot.configMotionCruiseVelocity((int)(Constants.ARM_MM_CRUISE_VELOCITY * Constants.ARM_MM_PEAK_SENSOR_VELOCITY));
        pivot.configClosedLoopPeakOutput(0, 1);
        
        setBrakeMode(BrakeMode.BREAK);
    
        brakePad = new Titan.Solenoid(Constants.ARM_BRAKE_PCM_ID, Constants.ARM_BRAKE_ID);
    
        armEncoder = new AnalogInput(Constants.ARM_ENCODER_PORT);
    }

    @Override
    public void init(final Robot robot){
    }

    @Override
    public void periodic(final Robot robot){
        if(getControlMode() == ControlMode.MANUAL){
            setBrakeMode(BrakeMode.BREAK);
        }
        // if(robot.getTeleop().getOperator().getRawButton(Titan.LogitechExtreme3D.Button.ELEVEN)){
        //     pivot.getEncoder().setPosition(0);
        // }

        //System.out.println(pivot.getIdleMode().name());

        brakePad.set(brakeState == BrakeState.DISENGAGED);

        final double power = Math.signum(armPower) * (Math.abs(armPower) + (0.16 * Math.abs(Math.cos(Math.toRadians(getArmAngle() - 90)))));
        //System.out.println(power);
        // System.out.println((0.3 * Math.cos(Math.toRadians(getWristPosition() - 90))));
        // USE THIS FOR BETTER EQUATION:
        //pivot.set(armPower + (0.2 * (Math.signum(armPower) * Math.cos(Math.toRadians(getArmAngle() - 90)))));
        pivot.set(power);
    }
    
    @Override
    public void disabled(final Robot robot){
        setBrakeMode(BrakeMode.BREAK);
    }

    public void pivot(final double in){
        double val = in;
        // if(getArmAngle() > Constants.ARM_MAX_ANGLE && val > 0){
        //     val = 0;
        // }

        pivot(in, val == 0 ? BrakeState.ENGAGED : BrakeState.DISENGAGED);
    }

    public void pivot(final double in, final BrakeState state){
        double val = in;
        // if(getArmAngle() > Constants.ARM_MAX_ANGLE && val > 0){
        //     val = 0;
        // }

        armPower = val;
        //if the value of the pivot is 0 (so stopped), automatically break
        brake(state);
    }

    public void brake(final BrakeState state){
        brakeState = state;
    }

    public ControlMode getControlMode(){
        return controlMode;
    }

    public void setControlMode(final ControlMode mode){
        controlMode = mode;
    }

    public double getArmAngle(){
        //return pivot.getEncoder().getPosition() / 237.5;
        //System.out.println(pivot.getEncoder().getPosition());
        final double position = (((5.0 - (armEncoder.getAverageVoltage() / 5.0)) * 360.0) - Constants.ARM_ENCODER_CALIBRATION_OFFSET) % 360;
        //final double position = (360.0 * (pivot.getEncoder().getPosition() / 237.5) + Constants.ARM_STOW_ANGLE) % 360;
        if(position < 0){
            return 360 + position;
        }
        return position;
    }

    public double getEncoderPosition(){
        return pivot.getSelectedSensorPosition();
    }

    public double getEncoderVelocity(){
        return pivot.getSelectedSensorVelocity();
    }

    public void setBrakeMode(final BrakeMode mode){
        pivot.setNeutralMode(mode == BrakeMode.BREAK ? NeutralMode.Brake : NeutralMode.Coast);
    }

}