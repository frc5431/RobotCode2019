package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class WaitForTargetCommand extends CommandBase {
	private long foundTarget = -1;

	public WaitForTargetCommand() {
		// name = "WaitForTargetCommand";
	}

	@Override
	public void initialize() {
		// drivebase.enableDistancePID();
		// drivebase.setDistancePIDTarget(leftDistance, rightDistance);
	}

	@Override
	public boolean isFinished() {
		if(Robot.getRobot().getVision().getTargetInfo().exists()){
			if(foundTarget < 0){
				foundTarget = System.currentTimeMillis();
			}
		}else{
			foundTarget = -1;
		}
		if(foundTarget >= 0 && System.currentTimeMillis() >= foundTarget + 100){
			return true;
		}

		return false;
	}
}