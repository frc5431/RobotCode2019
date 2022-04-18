package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.wpilib.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ControlMode;
import frc.robot.util.PIDConstants;

public class Arm extends SubsystemBase {
    public static enum BrakeState{
        ENGAGED, DISENGAGED
    };

    public static enum BrakeMode{
        BREAK, COAST
    };

    final CANSparkMax pivot;
    final RelativeEncoder pivotEncoder;
    final SparkMaxPIDController pivotPid;
    final Solenoid brakePad;

    final AnalogInput armEncoder;

    private ControlMode controlMode = ControlMode.MANUAL;

    private BrakeState brakeState = BrakeState.ENGAGED;

    private double armPower = 0.0;
    private double targetPosition = -1;

    public Arm(){
        pivot = new CANSparkMax(Constants.ARM_PIVOT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivot.restoreFactoryDefaults();
        pivot.setInverted(Constants.ARM_PIVOT_INVERTED);
        pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);

        pivotEncoder = pivot.getEncoder();
        //pivotEncoder.setPosition(0);
        pivotEncoder.setPositionConversionFactor(360.0 / 152);
        pivotEncoder.setVelocityConversionFactor(1.0);
        //pivotEncoder.setVelocityConversionFactor(360.0 / 237.5);

        pivotPid = pivot.getPIDController();

        final double peakSensorVelocity = Constants.ARM_SM_PEAK_SENSOR_VELOCITY;
        final PIDConstants pid = Constants.ARM_SM_PID;
        pivotPid.setP(pid.getP(), 0);
        pivotPid.setI(pid.getI(), 0);
        pivotPid.setD(pid.getD(), 0);
        //pivotPid.setFF(0);
        pivotPid.setFF(1.0 / peakSensorVelocity, 0);
        pivotPid.setSmartMotionMaxVelocity(Constants.ARM_SM_CRUISE_VELOCITY * peakSensorVelocity, 0);
        pivotPid.setSmartMotionMaxAccel(Constants.ARM_SM_ACCELERATION * peakSensorVelocity, 0);
        pivotPid.setOutputRange(-1, 1, 0);
        pivotPid.setSmartMotionAllowedClosedLoopError(1, 0);
        pivot.burnFlash();

        setBrakeMode(BrakeMode.BREAK);
    
        brakePad = new Solenoid(Constants.ARM_BRAKE_ID);
    
        armEncoder = new AnalogInput(Constants.ARM_ENCODER_PORT);

        new Thread(()->{
            while(true){
                try{
                    Thread.sleep(1);
                }catch(final InterruptedException e){
                    e.printStackTrace();
                }
                pivotEncoder.setPosition(getAbsoluteAngle());
            }
        }).start();
    }

    @Override
    public void periodic(){
        if(getControlMode() == ControlMode.MANUAL){
            setBrakeMode(BrakeMode.BREAK);
        }

        //pivotEncoder.setPosition(getArmAngle());
        // if(robot.getTeleop().getOperator().getRawButton(Titan.LogitechExtreme3D.Button.ELEVEN)){
        //     pivot.getEncoder().setPosition(0);
        // }

        //System.out.println(pivot.getIdleMode().name());

        //if(Titan.approxEquals(getAbsoluteAngle(), Constants.ARM_STOW_ANGLE, Constants.ARM_ANGLE_TOLERANCE)){
        //pivotEncoder.setPosition(getAbsoluteAngle());
        //}

        brakePad.set(brakeState == BrakeState.DISENGAGED);

        final double ff = 0.05 * Math.cos(Math.toRadians(getArmAngle() - 90));
        if(targetPosition >= 0){
            pivotPid.setReference(targetPosition, ControlType.kSmartMotion, 0, ff * RobotController.getBatteryVoltage());
        }else{
            final double power = Math.signum(armPower) * (Math.abs(armPower) + Math.abs(ff));
            pivotPid.setReference(power, ControlType.kDutyCycle, 0, 0);
        }
        //pivot.set(0.3);
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

    public double getAbsoluteAngle(){
        //return pivot.getEncoder().getPosition() / 237.5;
        //System.out.println(pivot.getEncoder().getPosition());
        final double position = (((5.0 - (armEncoder.getAverageVoltage() / 5.0)) * 360.0) - Constants.ARM_ENCODER_CALIBRATION_OFFSET) % 360;
        //final double position = (360.0 * (pivot.getEncoder().getPosition() / 237.5) + Constants.ARM_STOW_ANGLE) % 360;
        if(position < 0){
            return 360 + position;
        }
        return position;
    }

    public double getArmAngle(){
        return getEncoderPosition();
    }

    public double getEncoderPosition(){
        return pivotEncoder.getPosition();
    }

    public double getEncoderVelocity(){
        return pivotEncoder.getVelocity();
    }

    public double getOutputPower(){
        return pivot.getAppliedOutput();
    }

    public void setBrakeMode(final BrakeMode mode){
        pivot.setIdleMode(mode == BrakeMode.BREAK ? IdleMode.kBrake : IdleMode.kCoast);
    }

}