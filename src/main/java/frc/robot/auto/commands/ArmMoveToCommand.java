package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.team5431.titan.core.misc.Calc;
import frc.robot.components.Arm;
import frc.robot.components.Arm.BrakeMode;
import frc.robot.components.Arm.BrakeState;

public class ArmMoveToCommand extends CommandBase{
	public static enum CompletionCondition{
		SETPOINT, TRAVELLED, TRAVELLED_NO_BRAKE;
	}

	private final double position;
	private double startPosition;

	private final CompletionCondition condition;

	public ArmMoveToCommand(final double position){
		this(position, CompletionCondition.SETPOINT);
	}

	public ArmMoveToCommand(final double position, final CompletionCondition cond) {
        this.position = position;
		this.condition = cond;

		// name = "ArmMoveCommand";
		// properties = String.format("Position: %f; CompletionCondition: %s", position, condition.name());
	}

	// private double getArmSpeed(final double armAngle){
	// 	// currentPosError = ((startPos + (directionSignum * profile.getDistAtTime(time))) % 360) - robot.getArm().getArmAngle();
	// 	// //System.out.println((startPos + (directionSignum * profile.getDistAtTime(time))) % 360 + ", " + (directionSignum * profile.getDistAtTime(time)));
	// 	// return (directionSignum * profile.getVelocityAtTime(time) / Constants.ARM_MP_PEAK_SENSOR_VELOCITY) - distancePid.get();

	// 	final double error = Math.abs(position - armAngle);
	// 	final double speedOffset = Constants.AUTO_ARM_ACCELERATION * (Math.min(Constants.AUTO_ARM_ACCELERATION_MAX_ERROR, error) / Constants.AUTO_ARM_ACCELERATION_MAX_ERROR);

	// 	if(armAngle > position){
	// 		return -(Constants.AUTO_ARM_SPEED + speedOffset) * Constants.AUTO_ARM_DOWN_MULITPLIER;
	// 	}else{
	// 		return Constants.AUTO_ARM_SPEED + speedOffset;
	// 	}
	// }

	private boolean willBrake(){
		return condition != CompletionCondition.TRAVELLED_NO_BRAKE;
	}

	private boolean isComplete(final Arm arm){
		switch(condition){
		case TRAVELLED:
		case TRAVELLED_NO_BRAKE:
			if(startPosition > position){
				return arm.getArmAngle() <= position;
			}else{
				return arm.getArmAngle() >= position;
			}
		case SETPOINT:
		default:
			return Calc.approxEquals(arm.getArmAngle(), position, Constants.ARM_ANGLE_TOLERANCE);
		}
	}

	private void runArm(final Arm arm){
		arm.pivotTo(position);
		//arm.pivot(getArmSpeed(arm.getArmAngle()));
	}

	@Override
	public boolean isFinished() {
		final Arm arm = Robot.getRobot().getArm();

		if(arm.getControlMode() == ControlMode.MANUAL){
			Robot.getRobot().getAuton().abort(Robot.getRobot());
			return true;
		}

		if (isComplete(arm)) {
			arm.pivot(0.0, willBrake() ? BrakeState.ENGAGED : BrakeState.DISENGAGED);
			arm.setBrakeMode(willBrake() ? BrakeMode.BREAK : BrakeMode.COAST);
			return true;
		}else{
			runArm(arm);
		}

		return false;
	}

	@Override
	public void initialize() {
		final Arm arm = Robot.getRobot().getArm();

		arm.setControlMode(ControlMode.AUTO);

		startPosition = arm.getArmAngle();

		if(!isComplete(arm)){
			runArm(arm);
		}
	}
}