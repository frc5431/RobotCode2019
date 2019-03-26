package frc.robot.commands;

import frc.robot.util.Titan;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.robot.components.Arm.BrakeMode;
import frc.robot.components.Arm.BrakeState;
import frc.robot.util.motionprofile.Profile;

public class ArmMoveToCommand extends Titan.Command<Robot>{
	private Profile profile = null;

	private final double position;

	private final boolean willBrake;

	private int time = 0;

	private double currentPosError = 0.0;

	private final PIDController distancePid = new PIDController(Constants.ARM_MP_P, Constants.ARM_MP_I, Constants.ARM_MP_D, new PIDSource(){
	
		@Override
		public void setPIDSourceType(final PIDSourceType pidSource) {
			//do nothing
		}
	
		@Override
		public double pidGet() {
			return currentPosError;
		}
	
		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
	}, new PIDOutput(){
		@Override
		public void pidWrite(final double val){
		}
	}, 0.02);

	public ArmMoveToCommand(final double position){
		this(position, true);
	}

	public ArmMoveToCommand(final double position, final boolean brake) {
        this.position = position;
		this.willBrake = brake;

		name = "ArmMoveCommand";
		properties = "Position: " + position + "; Will brake: " + brake;
	}

	private double getArmSpeed(final Robot robot){
		// currentPosError = profile.getDistAtTime(time) - robot.getArm().getArmAngle();
		// return (profile.getVelocityAtTime(time) / Constants.ARM_MP_PEAK_SENSOR_VELOCITY) + distancePid.get();

		final double error = Math.abs(position - robot.getArm().getArmAngle());
		final double speedOffset = Constants.AUTO_ARM_ACCELERATION * (Math.min(Constants.AUTO_ARM_ACCELERATION_MAX_ERROR, error) / Constants.AUTO_ARM_ACCELERATION_MAX_ERROR);

		if(robot.getArm().getArmAngle() > position){
			return -(Constants.AUTO_ARM_SPEED + speedOffset) * Constants.AUTO_ARM_DOWN_MULITPLIER;
		}else{
			return Constants.AUTO_ARM_SPEED + speedOffset;
		}
	}

	private boolean isComplete(final Robot robot){
		return Titan.approxEquals(robot.getArm().getArmAngle(), position, Constants.ARM_ANGLE_TOLERANCE);
	}

	private void runArm(final Robot robot){
		robot.getArm().pivot(getArmSpeed(robot));
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getArm().getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		if (isComplete(robot)) {
			robot.getArm().pivot(0.0, willBrake ? BrakeState.ENGAGED : BrakeState.DISENGAGED);
			robot.getArm().setBrakeMode(willBrake ? BrakeMode.BREAK : BrakeMode.COAST);
			return CommandResult.COMPLETE;
		}

		++time;

		runArm(robot);

		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void init(final Robot robot) {
		robot.getArm().setControlMode(ControlMode.AUTO);

		if(!isComplete(robot)){
			profile = Profile.getVelProfile(Constants.ARM_MP_CRUISE_VELOCITY * Constants.ARM_MP_PEAK_SENSOR_VELOCITY, Constants.ARM_MP_ACCELERATION * Constants.ARM_MP_PEAK_SENSOR_VELOCITY, position - robot.getArm().getArmAngle(), robot.getArm().getArmVelocity(), 0);

			distancePid.setInputRange(-360, 360);
			distancePid.setOutputRange(-1.0, 1.0);
			distancePid.setContinuous(false);
			distancePid.setSetpoint(0.0);
			distancePid.enable();

			runArm(robot);
		}
	}

	@Override
	public void done(final Robot robot) {
		distancePid.disable();
	}
}