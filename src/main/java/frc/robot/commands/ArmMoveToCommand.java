package frc.robot.commands;

import frc.robot.Titan;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ControlMode;

public class ArmMoveToCommand extends Titan.Command<Robot>{
	private final double position, speed;
	//private PIDController pid = null;
	//private double speedOffset = 0.0;

	public ArmMoveToCommand(final double position, final double spd) {
        this.position = position;
        /* sped */
        this.speed = spd;

		name = "ArmMoveCommand";
		properties = String.format("Position %.2f : Speed %.2f", position, speed);
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getArm().getControlMode() == ControlMode.MANUAL){
			return CommandResult.CLEAR_QUEUE;
		}

		// if(pid == null){
		// 	return CommandResult.RESTART_COMMAND;
		// }

		final double currentPos = robot.getArm().getArmAngle();
		if (Titan.approxEquals(currentPos, position, Constants.ARM_ANGLE_TOLERANCE)) {
			robot.getArm().pivot(0.0);
			return CommandResult.COMPLETE;
		}

		// if((position > 180 && currentPos > 180) || (position < 180 && currentPos < 180)){
		// 	robot.getElevator().intakeFlipping = false;
		// }

		// if(robot.getElevator().intakeFlipping){
		// 	robot.getElevator().elevate(0.3);
		// }

		final double error = Math.abs(position - robot.getArm().getArmAngle());
		final double speedOffset = Constants.AUTO_ARM_ACCELERATION * (Math.min(Constants.AUTO_ARM_ACCELERATION_MAX_ERROR, error) / Constants.AUTO_ARM_ACCELERATION_MAX_ERROR);

		if(robot.getArm().getArmAngle() > position){
			robot.getArm().pivot(-(speed + speedOffset));
		}else{
			robot.getArm().pivot(speed + speedOffset);
		}

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
		//robot.getElevator().intakeFlipping = position > 180;
	}

	@Override
	public void done(final Robot robot) {
		// if(pid != null){
		// 	pid.disable();
		// }
	}
}