package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.util.ControlMode;
import frc.robot.util.Component;
import frc.robot.util.Testable;
import frc.robot.Robot;

public class Arm extends Component{
    public static enum BrakeState{
        ENGAGED, DISENGAGED
    };

    final CANSparkMax pivot;
    final Solenoid brakePad;

    final AnalogInput armEncoder;

    private ControlMode controlMode = ControlMode.MANUAL;

    private BrakeState brakeState = BrakeState.ENGAGED;

    private double armPower = 0.0;

    public Arm(){
        pivot = new CANSparkMax(Constants.ARM_PIVOT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivot.setInverted(Constants.ARM_PIVOT_INVERTED);
        pivot.setIdleMode(IdleMode.kBrake);
    
        brakePad = new Solenoid(Constants.ARM_BRAKE_PCM_ID, Constants.ARM_BRAKE_ID);
    
        armEncoder = new AnalogInput(Constants.ARM_ENCODER_PORT);
    }

    @Override
    public void init(final Robot robot){
        
    }

    @Override
    public void periodic(final Robot robot){
        brakePad.set(brakeState == BrakeState.DISENGAGED);
        
        // System.out.println((0.3 * Math.cos(Math.toRadians(getWristPosition() - 90))));
        // USE THIS FOR BETTER EQUATION:
        //pivot.set(armPower + (0.2 * (Math.signum(armPower) * Math.cos(Math.toRadians(getArmAngle() - 90)))));
        pivot.set(Math.signum(armPower) * (Math.abs(armPower) + (0.3 * Math.abs(Math.cos(Math.toRadians(getArmAngle() - 90))))));
    }
    
    @Override
    public void disabled(final Robot robot){
        
    }

    public void pivot(final double in){
        double val = in;
        if(getArmAngle() > 250 && val > 0){
            val = 0;
        }

        armPower = val;
        //if the value of the pivot is 0 (so stopped), automatically break
        brake(val == 0 ? BrakeState.ENGAGED : BrakeState.DISENGAGED);
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
        final double position = (((armEncoder.getAverageVoltage() / 5.0) * 360.0) - Constants.ARM_ENCODER_CALIBRATION_OFFSET) % 360;
        if(position < 0){
            return 360 + position;
        }
        return position;
    }

    @Override
    public String getTestResult(){
        return Testable.SUCCESS;
    }
}