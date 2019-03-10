package frc.robot.commands;

import frc.robot.util.Titan;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.robot.components.Arm.BrakeMode;
//import frc.robot.components.Arm;
import frc.robot.components.Arm.BrakeState;

public class ArmMoveToCommand extends Titan.Command<Robot>{
	private final double position, speed;

	//private long finished = -1;

	private final boolean willBrake;
	//private PIDController pid = null;
	//private double speedOffset = 0.0;

	public ArmMoveToCommand(final double position, final double spd){
		this(position, spd, true);
	}

	public ArmMoveToCommand(final double position, final double spd, final boolean brake) {
        this.position = position;
        /* sped */
		this.speed = spd;
		this.willBrake = brake;

		name = "ArmMoveCommand";
		properties = String.format("Position %.2f : Speed %.2f", position, speed);
	}

	private double getArmSpeed(final Robot robot){
		final double error = Math.abs(position - robot.getArm().getArmAngle());
		final double speedOffset = Constants.AUTO_ARM_ACCELERATION * (Math.min(Constants.AUTO_ARM_ACCELERATION_MAX_ERROR, error) / Constants.AUTO_ARM_ACCELERATION_MAX_ERROR);

		if(robot.getArm().getArmAngle() > position){
			return -(speed + speedOffset) * Constants.AUTO_ARM_DOWN_MULITPLIER;
		}else{
			return speed + speedOffset;
		}
	}

	private boolean isComplete(final Robot robot){
		return Titan.approxEquals(robot.getArm().getArmAngle(), position, Constants.ARM_ANGLE_TOLERANCE);
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getArm().getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		// if(pid == null){
		// 	return CommandResult.RESTART_COMMAND;
		// }

		if (isComplete(robot)) {
			// if(finished < 0){
			// 	finished = System.currentTimeMillis();
			// }
			// if(!willBrake){
			// 	if(System.currentTimeMillis() > finished + 1000){
			// 		robot.getArm().pivot(0.0);
			// 		return CommandResult.COMPLETE;
			// 	}else{
			// 		robot.getArm().pivot(0.0, Arm.BrakeState.DISENGAGED);
			// 		return CommandResult.IN_PROGRESS;
			// 	}
			// }else{
				robot.getArm().pivot(0.0, willBrake ? BrakeState.ENGAGED : BrakeState.DISENGAGED);
				robot.getArm().setBrakeMode(willBrake ? BrakeMode.BREAK : BrakeMode.COAST);
				return CommandResult.COMPLETE;
			//}
		}

		robot.getArm().pivot(getArmSpeed(robot));

		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void init(final Robot robot) {
		// pid  = new PIDController(Constants.ARM_PID_P, Constants.ARM_PID_I, Constants.ARM_PID_D, new PIDSource(){
		
		// 	@Override
		// 	public void setPIDSourceType(final PIDSourceType pidSource) {
		// 		//do nothing
		// 	}
		
		// 	@Override
		// 	public double pidGet() {
		// 		return robot.getArm().getWristPosition();
		// 	}
		
		// 	@Override
		// 	public PIDSourceType getPIDSourceType() {
		// 		return PIDSourceType.kDisplacement;
		// 	}
		// }, new PIDOutput(){
		
		// 	@Override
		// 	public void pidWrite(final double output) {
		// 		speedOffset = output;
		// 	}
		// });
		// pid.enable();

		// pid.setInputRange(0.0, 360);
		// pid.setOutputRange(-1.0, 1.0);
		// pid.setSetpoint(position);

		robot.getArm().setControlMode(ControlMode.AUTO);

		if(!isComplete(robot)){
			robot.getArm().pivot(getArmSpeed(robot));
		}
	}

	@Override
	public void done(final Robot robot) {
		// if(pid != null){
		// 	pid.disable();
		// }
	}
}