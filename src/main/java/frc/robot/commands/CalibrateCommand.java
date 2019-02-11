package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Titan;
import frc.robot.components.Arm;
import frc.robot.components.Intake;

public class CalibrateCommand extends Titan.Command<Robot> {
	public CalibrateCommand() {
		name = "CalibrateCommand";
		properties = "Intake and Gamedata calibration";
	}

	@Override
	public CommandResult update(final Robot robot) {
		return CommandResult.COMPLETE;
	}

	@Override
	public void init(final Robot robot) {
		robot.getDrivebase().drive(0.0, 0.0);
		robot.getElevator().elevate(0.0);
        
        final Intake intake = robot.getIntake();
        intake.actuateHatch(false);
        intake.finger(false);
        intake.roll(0.0);

        final Arm arm = robot.getArm();
        arm.pivot(0.0);
        arm.wrist(false);
	}

	@Override
	public void done(final Robot robot) {
		robot.getDrivebase().setHome();
	}
}