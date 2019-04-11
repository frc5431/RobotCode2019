package frc.robot.components;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.util.ControlMode;
import frc.robot.util.PIDConstants;
import frc.robot.Robot;
import frc.robot.util.Titan;

public class Arm extends Titan.Component<Robot>{
    public static enum BrakeState{
        ENGAGED, DISENGAGED
    };

    public static enum BrakeMode{
        BREAK, COAST
    };

    final CANSparkMax pivot;
    final CANEncoder pivotEncoder;
    final CANPIDController pivotPid;
    final Titan.Solenoid brakePad;

    final AnalogInput armEncoder;

    private ControlMode controlMode = ControlMode.MANUAL;

    private BrakeState brakeState = BrakeState.ENGAGED;

    private double armPower = 0.0;
    private double targetPosition = -1;

    public Arm(){
        pivot = new CANSparkMax(Constants.ARM_PIVOT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivot.setInverted(Constants.ARM_PIVOT_INVERTED);

        pivotEncoder = pivot.getEncoder();
        pivotEncoder.setPosition(0);
        pivotEncoder.setPositionConversionFactor(360.0 / 237.5);
        pivotEncoder.setVelocityConversionFactor(360.0 / 237.5);

        pivotPid = pivot.getPIDController();

        final PIDConstants pid = Constants.ARM_SM_PID;
        pivotPid.setP(pid.getP());
        pivotPid.setI(pid.getI());
        pivotPid.setD(pid.getD());

        final double peakSensorVelocity = Constants.ARM_SM_PEAK_SENSOR_VELOCITY;
        pivotPid.setSmartMotionMaxVelocity(Constants.ARM_SM_CRUISE_VELOCITY * peakSensorVelocity, 0);
        pivotPid.setSmartMotionMaxAccel(Constants.ARM_SM_ACCELERATION * peakSensorVelocity, 0);

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

        pivotEncoder.setPosition(getArmAngle());
        // if(robot.getTeleop().getOperator().getRawButton(Titan.LogitechExtreme3D.Button.ELEVEN)){
        //     pivot.getEncoder().setPosition(0);
        // }

        //System.out.println(pivot.getIdleMode().name());

        brakePad.set(brakeState == BrakeState.DISENGAGED);

        final double ff = 0.16 * Math.cos(Math.toRadians(getArmAngle() - 90));
        if(targetPosition >= 0){
            pivotPid.setReference(targetPosition, ControlType.kSmartMotion, 0, ff);
        }else{
            final double power = Math.signum(armPower) * (Math.abs(armPower) + Math.abs(ff));
            pivotPid.setReference(power, ControlType.kDutyCycle, 0, 0);
        }

        //pivot.set(0.3);
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
        targetPosition = -1;
        //if the value of the pivot is 0 (so stopped), automatically break
        brake(state);
    }

    public void pivotTo(final double pos){
        armPower = 0;
        targetPosition = pos;

        brake(BrakeState.DISENGAGED);
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
        return pivotEncoder.getPosition();
    }

    public double getEncoderVelocity(){
        return pivotEncoder.getVelocity();
    }

    public void setBrakeMode(final BrakeMode mode){
        pivot.setIdleMode(mode == BrakeMode.BREAK ? IdleMode.kBrake : IdleMode.kCoast);
    }

}