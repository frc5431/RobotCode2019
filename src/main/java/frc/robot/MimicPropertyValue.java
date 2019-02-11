package frc.robot;

import java.util.function.Function;

import frc.robot.Robot;
import frc.robot.Titan;

public enum MimicPropertyValue implements Titan.Mimic.PropertyValue<Robot>{
        LEFT_DISTANCE(Titan.Mimic.PropertyType.DOUBLE, (robot)->robot.getDrivebase().getLeftDistance()),
		RIGHT_DISTANCE(Titan.Mimic.PropertyType.DOUBLE, (robot)->robot.getDrivebase().getRightDistance()),
		ANGLE(Titan.Mimic.PropertyType.DOUBLE, (robot)->robot.getDrivebase().getNavx().getAngle()),
		LEFT_POWER(Titan.Mimic.PropertyType.DOUBLE, (robot)->robot.getTeleop().getDriver().getRawAxis(Titan.Xbox.Axis.LEFT_Y)),
		RIGHT_POWER(Titan.Mimic.PropertyType.DOUBLE, (robot)->robot.getTeleop().getDriver().getRawAxis(Titan.Xbox.Axis.RIGHT_Y)),
		HOME(Titan.Mimic.PropertyType.BOOLEAN, (robot)->robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.START));

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