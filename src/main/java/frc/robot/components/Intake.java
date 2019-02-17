package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake{
    private final WPI_TalonSRX rollers;

    private final DoubleSolenoid hatchLeft, hatchRight, finger;

    private boolean isHatching = true, isFingering = true;

    public Intake(){
        rollers = new WPI_TalonSRX(Constants.INTAKE_ROLLER_ID);
        rollers.setInverted(Constants.INTAKE_ROLLER_INVERTED);
        rollers.setNeutralMode(NeutralMode.Brake);

        hatchLeft = new DoubleSolenoid(Constants.INTAKE_HATCH_LEFT_PCM_ID, Constants.INTAKE_HATCH_LEFT_FORWARD_ID, Constants.INTAKE_HATCH_LEFT_REVERSE_ID);
        hatchRight = new DoubleSolenoid(Constants.INTAKE_HATCH_RIGHT_PCM_ID, Constants.INTAKE_HATCH_RIGHT_FORWARD_ID, Constants.INTAKE_HATCH_RIGHT_REVERSE_ID);
        finger = new DoubleSolenoid(Constants.INTAKE_FINGER_PCM_ID, Constants.INTAKE_FINGER_FORWARD_ID, Constants.INTAKE_FINGER_REVERSE_ID);
    }

    public void periodic(final Robot robot){
        hatchLeft.set(isHatching ? Value.kForward : Value.kReverse);
        hatchRight.set(isHatching ? Value.kForward : Value.kReverse);

        finger.set(isFingering ? Value.kForward : Value.kReverse);
    }

    public void roll(final double val){
        rollers.set(val);
    }

    public void actuateHatch(final boolean val){
        isHatching = val;
    }

    public void finger(final boolean val){
        isFingering = val;
    }

    public boolean isFingering(){
        return isFingering;
    }

    public boolean isHatchOuttaking(){
        return isHatching;
    }
}