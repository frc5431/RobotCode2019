package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;
import frc.robot.components.Intake;

public class RollerCommand extends Titan.Command<Robot> {
    private final double power;
    private final long time;

	public RollerCommand(final double power, final long time) {
		name = "RollerCommand";

        this.power = power;
		this.time = time;
		
		properties = String.format("Power: %f; Time: %d", power, time);
	}
	
	@Override
	public void init(final Robot robot) {
		robot.getIntake().setControlMode(ControlMode.AUTO);
    }

	@Override
	public CommandResult update(final Robot robot) {
		final Intake intake = robot.getIntake();
		if(time >= 0 && intake.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

    	intake.roll(power);
		
		if(time < 0){
			return CommandResult.COMPLETE;
		}else if(System.currentTimeMillis() > startTime + time) {
			intake.roll(0);
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