package frc.robot.auto.commands;

import frc.robot.util.ControlMode;
import frc.team5431.titan.core.misc.Calc;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.auto.vision.TargetInfo;
import frc.robot.auto.vision.TargetType;
import frc.robot.components.Drivebase;
import frc.robot.components.Vision;
import frc.robot.components.Drivebase.AutoType;

public class TurnToTargetCommand extends CommandBase {
	private final double speed;
	private final TargetType type;

	private long hitTarget = -1;

	public TurnToTargetCommand(final double speed, final TargetType ttype) {
		// name = "TurnToTargetCommand";

		this.type = ttype;
		this.speed = speed;

		// properties = String.format("Speed: %f; Target: %s", speed, ttype);
	}

	public TurnToTargetCommand(final TargetType ttype){
		this(Constants.AUTO_TURN_MIN_SPEED, ttype);
	}

	@Override
	public void initialize() {
		final Drivebase drivebase = Robot.getRobot().getDrivebase();
		drivebase.prepareForAutoControl(AutoType.POINT_TURN);

		drivebase.enableAnglePID();
		drivebase.setAnglePIDTarget(0);

		Robot.getRobot().getVision().setTargetType(type);

		// drivebase.enableDistancePID();
		// drivebase.setDistancePIDTarget(leftDistance, rightDistance);
	}

	@Override
	public boolean isFinished() {
		Robot robot = Robot.getRobot();
		final Drivebase drivebase = robot.getDrivebase();
		final Vision vision = robot.getVision();
		if(drivebase.getControlMode() == ControlMode.MANUAL){
			vision.setLEDState(Vision.LEDState.OFF);
			robot.getAuton().abort(robot);
			return true;
		}

		vision.setLEDState(Vision.LEDState.ON);

		final double currentAngle = drivebase.getAngle();

		final TargetInfo target = robot.getVision().getTargetInfo();
		final double angle = currentAngle + target.getXAngle();
		drivebase.setAnglePIDTarget(angle);


		final double targetSignum;
		switch(type){
		case FRONT_RIGHT:
			targetSignum = -1;
			break;
		case FRONT_LEFT:
		default:
			targetSignum = 1;
		}

		drivebase.drive(speed * targetSignum * Math.signum(angle - currentAngle), -speed * targetSignum * Math.signum(angle - currentAngle));
		
		if(Calc.approxEquals(currentAngle, angle, Constants.DRIVEBASE_ANGLE_TOLERANCE)){
			if(hitTarget <= 0){
				hitTarget = System.currentTimeMillis();
			}
		}else{
			hitTarget = -1;
		}

		if(/*drivebase.hasTurned(angle)*/hitTarget > 0 && System.currentTimeMillis() >= hitTarget + 100){
			drivebase.disableAutoControl();
			//vision.setLEDState(Vision.LEDState.OFF);
			return true;
		}else{
			return false;
		}
	}
}