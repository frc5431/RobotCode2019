package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.team5431.titan.core.joysticks.Xbox;
import frc.team5431.titan.core.misc.Calc;
import frc.robot.util.ControlMode;
import frc.robot.components.Vision;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.AutoType;
import frc.robot.auto.vision.TargetInfo;
import frc.robot.auto.vision.TargetType;

public class DriveToTargetCommand extends CommandBase {
	private final TargetType ttype;

	private double lastDistance = 0;
	private long lastDistanceChange = 0;

	private double lastErrorAngle = 0;


	public DriveToTargetCommand(){
		this(TargetType.FRONT_RIGHT);
	}

	public DriveToTargetCommand(final TargetType type){
		this.ttype = type;

		// name = "Drive to vision target";
		// properties = String.format("Type: %s", type.name());
	}

    @Override
	public void initialize() {
		final Drivebase drivebase = Robot.getRobot().getDrivebase();
		drivebase.prepareForAutoControl(AutoType.VISION);
	
		Robot.getRobot().getVision().setTargetType(ttype);

		lastDistanceChange = System.currentTimeMillis();
	}

	@Override
	public boolean isFinished() {
		Robot robot = Robot.getRobot();
		final Drivebase drivebase = robot.getDrivebase();
		final Vision vision = robot.getVision();

		if(drivebase.getControlMode() == ControlMode.MANUAL){
			if(!robot.getTeleop().getDriver().getRawButton(Xbox.Button.BUMPER_R) && !robot.getTeleop().getDriver().getRawButton(Xbox.Button.BUMPER_L)){
				vision.setLEDState(Vision.LEDState.OFF);
				robot.getAuton().abort(robot);
				return true;
			}else{
				drivebase.setControlMode(ControlMode.AUTO);
			}
		}

		vision.setLEDState(Vision.LEDState.ON);

		/*
		When the elevator is running, to avoid breaking the carriage, we want to slow down.
		*/
		final boolean isRunningElevator = !Calc.approxEquals(robot.getElevator().getEncoderVelocity(), 0, 200);

		final double currentDistance = (drivebase.getLeftDistance() + drivebase.getRightDistance()) / 2.0;
		if(currentDistance != lastDistance || !isRunningElevator){
			lastDistance = currentDistance;
			lastDistanceChange = System.currentTimeMillis();
		}

		final TargetInfo target = vision.getTargetInfo();
		/*
		When in reverse, the angles are flipped
		*/
		//double angleError = directionSignum * target.getXAngle();
		double angleError;
		if(target.exists()){
			//if(ttype.getLimelight().isCentered()){
				angleError = target.getXAngle();
			// }else{
			// 	angleError = (target.getXAngle() / target.getYAngle()) - 1.33;//1.18
			// }
			lastErrorAngle = angleError;
		}else{
			angleError = lastErrorAngle;
		}
		//angleError *= directionSignum;
		if(Calc.approxEquals(angleError, 1, 0.1)){
			angleError = 0;
		}
		final double distanceError = target.getYAngle();

		final boolean atTarget = target.getArea() > 12;

		// you are allowed to be too close, as the intake will just ram the hatch into the rocket
		if((target.exists() && atTarget) || (!isRunningElevator && System.currentTimeMillis() > lastDistanceChange + 500)){
			drivebase.disableAutoControl();

			vision.setLEDState(Vision.LEDState.OFF);
			return true;
		}

		double rawPower = (isRunningElevator ? 0.0 : (Constants.AUTO_AIM_DISTANCE_MIN + (Constants.AUTO_AIM_DISTANCE_P * distanceError)));
		final double angleP = Constants.AUTO_AIM_ANGLE_UNCENTERED_P;
		//rawPower *= (5.0 - Math.min(Math.abs(angleError), 5)) / 5.0;
		final double angleAdjust = (isRunningElevator ? 0.0 : (angleP * angleError)) + (Math.signum(angleError) * Constants.AUTO_AIM_ANGLE_MIN);

		drivebase.drive(rawPower + angleAdjust, rawPower - angleAdjust);
        //drivebase.drive(rawPower, rawPower);
		return false;
	}
}