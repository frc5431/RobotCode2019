package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.components.Intake;
import frc.robot.util.ControlMode;

public class GrabBallCommand extends CommandBase{
	public GrabBallCommand() {
		// name = "GrabBallCommand";
		// properties = "Grabs a ball by running the rollers until it gets a ball";
	}

	@Override
	public boolean isFinished() {
		Robot robot = Robot.getRobot();
		final Intake intake = robot.getIntake();
		if(intake.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return true;
		}

		if(intake.isBallIn()){
			intake.roll(0.0);
			return true;
		}

		intake.roll(Constants.INTAKE_ROLLER_SPEED);

		return false;
	}

	@Override
	public void initialize() {
		Robot.getRobot().getIntake().setControlMode(ControlMode.AUTO);
	}
}