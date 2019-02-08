package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class Intake{
    private final WPI_VictorSPX rollers;

    private final DoubleSolenoid hatchMechanism;

    public Intake(){
        rollers = new WPI_VictorSPX(Constants.INTAKE_ROLLER_ID);
        rollers.setInverted(Constants.INTAKE_ROLLER_INVERTED);

        hatchMechanism = new DoubleSolenoid(Constants.INTAKE_HATCH_FORWARD_ID, Constants.INTAKE_HATCH_REVERSE_ID);
    }

    public void roll(final double val){
        rollers.set(val);
    }

    public void actuateHatch(final boolean val){
        hatchMechanism.set(val ? Value.kForward : Value.kReverse);
    }
}