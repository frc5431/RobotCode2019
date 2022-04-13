package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.robot.components.Arm;
import frc.robot.components.Arm.BrakeMode;
import frc.robot.components.Arm.BrakeState;

public class ArmBrakeCommand extends CommandBase {
	private final boolean willBrake;

	public ArmBrakeCommand(final boolean brake) {
		this.willBrake = brake;

		// name = "ArmMoveCommand";
		// properties = String.format("Brake: %b", willBrake);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void initialize() {
		final Arm arm = Robot.getRobot().getArm();

		arm.setControlMode(ControlMode.AUTO);

		arm.pivot(0.0, willBrake ? BrakeState.ENGAGED : BrakeState.DISENGAGED);
		arm.setBrakeMode(willBrake ? BrakeMode.BREAK : BrakeMode.COAST);
	}
}