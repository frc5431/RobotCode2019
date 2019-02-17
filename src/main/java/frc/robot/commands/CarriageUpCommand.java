package frc.robot.commands;

import frc.robot.Titan;
import frc.robot.Robot;
import frc.robot.ControlMode;

public class CarriageUpCommand extends Titan.Command<Robot> {
    private final double speed;

    public CarriageUpCommand(final double speed) {
        this.speed = speed;

        name = "CarriageUpCommand";
        properties = "Move the carriage all the way up : Speed " + speed;
    }

    @Override
    public CommandResult update(final Robot robot) {
        if(robot.getElevator().getControlMode() == ControlMode.MANUAL){
			return CommandResult.CLEAR_QUEUE;
		}

        if(robot.getElevator().isCarriageUp() && robot.getElevator().getEncoderPosition() > 29000){
            robot.getElevator().elevate(0.0);
            return CommandResult.COMPLETE;
        }

        robot.getElevator().elevate(speed);
        
        return CommandResult.IN_PROGRESS;
    }

    @Override
    public void init(final Robot robot) {
        robot.getElevator().setControlMode(ControlMode.AUTO);
    }

    @Override
    public void done(final Robot robot) {
    }
}