package frc.robot.components;

import edu.wpi.first.wpilibj.Compressor;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Titan;

public class Pneumatics extends Titan.Component<Robot>{
    private final Compressor compressor;

    public Pneumatics(){
        compressor = new Compressor(Constants.COMPRESSOR_PCM_ID);
        compressor.start();
        compressor.setClosedLoopControl(true);
    }

    @Override
    public void init(final Robot robot){
        switch(robot.getMode()){
        case AUTO:
        case TEST:
            compressor.setClosedLoopControl(false);
            compressor.stop();
            break;
        case TELEOP:
        default:
            compressor.setClosedLoopControl(true);
        }
    }

    @Override
    public void periodic(final Robot robot){
    }

    @Override
    public void disabled(final Robot robot){
    }
}