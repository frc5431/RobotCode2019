package frc.robot.commands;

import frc.robot.Titan;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ControlMode;

public class GrabBallCommand extends Titan.Command<Robot>{
	public GrabBallCommand() {
		name = "GrabBallCommand";
		properties = "Grabs a ball by running the rollers until it gets a ball";
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getArm().getControlMode() == ControlMode.MANUAL){
			return CommandResult.CLEAR_QUEUE;
		}

		if(robot.getIntake().isBallIn()){
			robot.getIntake().roll(0.0);
			return CommandResult.COMPLETE;
		}


		// if((position > 180 && currentPos > 180) || (position < 180 && currentPos < 180)){
		// 	robot.getElevator().intakeFlipping = false;
		// }

		// if(robot.getElevator().intakeFlipping){
		// 	robot.getElevator().elevate(0.3);
		// }

		robot.getIntake().roll(Constants.INTAKE_ROLLER_SPEED);

		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void init(final Robot robot) {
		robot.getArm().setControlMode(ControlMode.AUTO);
	}

	@Override
	public void done(final Robot robot) {
	}
}