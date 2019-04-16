package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.Robot;

public class WaitForTargetCommand extends Titan.Command<Robot> {
	private long foundTarget = -1;

	public WaitForTargetCommand() {
		name = "WaitForTargetCommand";
	}

	@Override
	public void init(final Robot robot) {
		// drivebase.enableDistancePID();
		// drivebase.setDistancePIDTarget(leftDistance, rightDistance);
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getVision().getTargetInfo().exists()){
			if(foundTarget < 0){
				foundTarget = System.currentTimeMillis();
			}
		}else{
			foundTarget = -1;
		}
		if(foundTarget >= 0 && System.currentTimeMillis() >= foundTarget + 100){
			return CommandResult.COMPLETE;
		}

		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void done(final Robot robot) {
	}
}