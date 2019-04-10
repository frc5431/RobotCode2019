package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.robot.components.Arm;
import frc.robot.components.Arm.BrakeMode;
import frc.robot.components.Arm.BrakeState;

public class ArmMoveToCommand extends Titan.Command<Robot>{
	private final double position;

	private final boolean willBrake;

	public ArmMoveToCommand(final double position){
		this(position, true);
	}

	public ArmMoveToCommand(final double position, final boolean brake) {
        this.position = position;
		this.willBrake = brake;

		name = "ArmMoveCommand";
		properties = "Position: " + position + "; Will brake: " + brake;
	}

	private double getArmSpeed(final double armAngle){
		// currentPosError = ((startPos + (directionSignum * profile.getDistAtTime(time))) % 360) - robot.getArm().getArmAngle();
		// //System.out.println((startPos + (directionSignum * profile.getDistAtTime(time))) % 360 + ", " + (directionSignum * profile.getDistAtTime(time)));
		// return (directionSignum * profile.getVelocityAtTime(time) / Constants.ARM_MP_PEAK_SENSOR_VELOCITY) - distancePid.get();

		final double error = Math.abs(position - armAngle);
		final double speedOffset = Constants.AUTO_ARM_ACCELERATION * (Math.min(Constants.AUTO_ARM_ACCELERATION_MAX_ERROR, error) / Constants.AUTO_ARM_ACCELERATION_MAX_ERROR);

		if(armAngle > position){
			return -(Constants.AUTO_ARM_SPEED + speedOffset) * Constants.AUTO_ARM_DOWN_MULITPLIER;
		}else{
			return Constants.AUTO_ARM_SPEED + speedOffset;
		}
	}

	private boolean isComplete(final Robot robot){
		return Titan.approxEquals(robot.getArm().getArmAngle(), position, Constants.ARM_ANGLE_TOLERANCE);
	}

	private void runArm(final Arm arm){
		arm.pivot(getArmSpeed(arm.getArmAngle()));
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Arm arm = robot.getArm();

		if(arm.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		if (isComplete(robot)) {
			arm.pivot(0.0, willBrake ? BrakeState.ENGAGED : BrakeState.DISENGAGED);
			arm.setBrakeMode(willBrake ? BrakeMode.BREAK : BrakeMode.COAST);
			return CommandResult.COMPLETE;
		}else{
			runArm(arm);
		}

		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void init(final Robot robot) {
		final Arm arm = robot.getArm();

		arm.setControlMode(ControlMode.AUTO);

		if(!isComplete(robot)){
			runArm(arm);
		}
	}

	@Override
	public void done(final Robot robot) {
	}
}