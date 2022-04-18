package frc.robot.wpilib;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class DoubleSolenoid extends edu.wpi.first.wpilibj.DoubleSolenoid {
    private Value currentState = Value.kOff;

    public DoubleSolenoid(final int forwardChannel, final int reverseChannel){
        super(PneumaticsModuleType.CTREPCM, forwardChannel, reverseChannel);
    }

    public DoubleSolenoid(final int moduleNumber, final int forwardChannel, final int reverseChannel){
        super(moduleNumber, PneumaticsModuleType.CTREPCM, forwardChannel, reverseChannel);
    }

    @Override
    public void set(final Value newState){
        if(currentState != newState){
            currentState = newState;
            super.set(newState);
        }
    }
}
