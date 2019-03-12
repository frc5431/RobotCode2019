package frc.robot.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;

public class RollerCommand extends Titan.Command<Robot> {
    private final double power;
    private final long time;

	public RollerCommand(final double power, final long time) {
		name = "RollerCommand";

        this.power = power;
        this.time = time;

		properties = "Power: " + power + " for " + time + " MS";
	}
	
	@Override
	public void init(final Robot robot) {
		//if(time >= 0){
			robot.getIntake().setControlMode(ControlMode.AUTO);
		//}
    }

	@Override
	public CommandResult update(final Robot robot) {
		if(time >= 0 && robot.getArm().getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

        robot.getIntake().roll(power);
		
		if(time < 0){
			return CommandResult.COMPLETE;
		}else if(System.currentTimeMillis() > startTime + time) {
			robot.getIntake().roll(0);
			return CommandResult.COMPLETE;
		}

		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void done(final Robot robot) {
		if(time >= 0){
			robot.getIntake().roll(0);
		}
	}
}