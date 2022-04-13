package frc.robot.components;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Pneumatics extends SubsystemBase {
    private final Compressor compressor;

    public Pneumatics(){
        compressor = new Compressor(Constants.COMPRESSOR_PCM_ID, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        // compressor.setClosedLoopControl(true);

        switch(Robot.getRobot().getMode()){
            case AUTO:
            case TEST:
                // compressor.setClosedLoopControl(false);
                compressor.disable();
                break;
            case TELEOP:
            default:
                // compressor.setClosedLoopControl(true);
        }
    }
}