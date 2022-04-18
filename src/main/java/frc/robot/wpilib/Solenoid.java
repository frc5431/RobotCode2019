package frc.robot.wpilib;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Solenoid extends edu.wpi.first.wpilibj.Solenoid {
    private boolean currentState = false;

    public Solenoid(final int channel){
        super(PneumaticsModuleType.CTREPCM, channel);
    }

    public Solenoid(final int moduleNumber, final int channel) {
        super(moduleNumber, PneumaticsModuleType.CTREPCM, channel);
    }

    @Override
    public void set(final boolean newState){
        if(currentState != newState){
            currentState = newState;
            super.set(newState);
        }
    }
}
