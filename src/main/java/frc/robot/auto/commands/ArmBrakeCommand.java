package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.robot.components.Arm;
import frc.robot.components.Arm.BrakeMode;
import frc.robot.components.Arm.BrakeState;

public class ArmBrakeCommand extends Titan.Command<Robot>{
	private final boolean willBrake;

	public ArmBrakeCommand(final boolean brake) {
		this.willBrake = brake;

		name = "ArmMoveCommand";
		properties = String.format("Brake: %b", willBrake);
	}

	@Override
	public CommandResult update(final Robot robot) {
		return CommandResult.COMPLETE;
	}

	@Override
	public void init(final Robot robot) {
		final Arm arm = robot.getArm();

		arm.setControlMode(ControlMode.AUTO);

		arm.pivot(0.0, willBrake ? BrakeState.ENGAGED : BrakeState.DISENGAGED);
		arm.setBrakeMode(willBrake ? BrakeMode.BREAK : BrakeMode.COAST);
	}

	@Override
	public void done(final Robot robot) {
	}
}