package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class Intake{
    private final WPI_TalonSRX rollers;

    private final DoubleSolenoid hatchLeft, hatchRight, finger;

    public Intake(){
        rollers = new WPI_TalonSRX(Constants.INTAKE_ROLLER_ID);
        rollers.setInverted(Constants.INTAKE_ROLLER_INVERTED);

        hatchLeft = new DoubleSolenoid(Constants.INTAKE_HATCH_LEFT_PCM_ID, Constants.INTAKE_HATCH_LEFT_FORWARD_ID, Constants.INTAKE_HATCH_LEFT_REVERSE_ID);
        hatchRight = new DoubleSolenoid(Constants.INTAKE_HATCH_RIGHT_PCM_ID, Constants.INTAKE_HATCH_RIGHT_FORWARD_ID, Constants.INTAKE_HATCH_RIGHT_REVERSE_ID);
        finger = new DoubleSolenoid(Constants.INTAKE_FINGER_PCM_ID, Constants.INTAKE_FINGER_FORWARD_ID, Constants.INTAKE_FINGER_REVERSE_ID);
    }

    public void roll(final double val){
        rollers.set(val);
    }

    public void actuateHatch(final boolean val){
        hatchLeft.set(val ? Value.kForward : Value.kReverse);
        hatchRight.set(val ? Value.kForward : Value.kReverse);
    }

    public void finger(final boolean val){
        finger.set(val ? Value.kForward : Value.kReverse);
    }
}