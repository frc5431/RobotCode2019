package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.components.Intake;
import frc.robot.util.ControlMode;

public class GrabBallCommand extends Titan.Command<Robot>{
	public GrabBallCommand() {
		name = "GrabBallCommand";
		properties = "Grabs a ball by running the rollers until it gets a ball";
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Intake intake = robot.getIntake();
		if(intake.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		if(intake.isBallIn()){
			intake.roll(0.0);
			return CommandResult.COMPLETE;
		}

		intake.roll(Constants.INTAKE_ROLLER_SPEED);

		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void init(final Robot robot) {
		robot.getIntake().setControlMode(ControlMode.AUTO);
	}

	@Override
	public void done(final Robot robot) {
	}
}