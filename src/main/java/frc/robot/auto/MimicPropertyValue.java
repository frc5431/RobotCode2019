package frc.robot.auto;

import java.util.function.Function;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import frc.robot.util.Titan;

public enum MimicPropertyValue implements Titan.Mimic.PropertyValue<Robot>{
        LEFT_DISTANCE(Titan.Mimic.PropertyType.DOUBLE, (robot)->robot.getDrivebase().getLeftDistance()),
		RIGHT_DISTANCE(Titan.Mimic.PropertyType.DOUBLE, (robot)->robot.getDrivebase().getRightDistance()),
		ANGLE(Titan.Mimic.PropertyType.DOUBLE, (robot)->0/*robot.getDrivebase().getNavx().getAngle()*/),
		LEFT_POWER(Titan.Mimic.PropertyType.DOUBLE, (robot)->robot.getDrivebase().getLeftPower()),
		RIGHT_POWER(Titan.Mimic.PropertyType.DOUBLE, (robot)->robot.getDrivebase().getRightPower()),
		HOME(Titan.Mimic.PropertyType.BOOLEAN, (robot)->robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.START)),
		BATTERY(Titan.Mimic.PropertyType.DOUBLE, (robot)->RobotController.getBatteryVoltage()),
		USING_VISION(Titan.Mimic.PropertyType.BOOLEAN, (robot)->robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.BACK)),
		SEQUENCE_TYPE(Titan.Mimic.PropertyType.INTEGER, (robot)->robot.getAuton().getCurrentSequenceType().ordinal()),
		RUNNING_SEQUENCE(Titan.Mimic.PropertyType.INTEGER, (robot)->{
			final Sequence runningSequence = robot.getAuton().getRunningSequence();
			if(runningSequence == null){
				return -1;
			}else{
				return runningSequence.ordinal();
			}
		});

		private final Titan.Mimic.PropertyType type;
		private final Function<Robot, Object> getter;

		private MimicPropertyValue(final Titan.Mimic.PropertyType type, final Function<Robot, Object> getter){
			this.type = type;
			this.getter = getter;
			
		}

		public Titan.Mimic.PropertyType getType(){
			return type;
		}

		public Object get(final Robot robot){
			return getter.apply(robot);
		}


}