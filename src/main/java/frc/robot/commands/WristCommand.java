package frc.robot.commands;

import frc.robot.Titan;
import frc.robot.Robot;

public class WristCommand extends Titan.Command<Robot> {
        private final boolean wristed;

        public WristCommand(final boolean wristed) {
            this.wristed = wristed;

            name = "WristCommand";
            properties = "Whether it will wrist or not";
        }
    
        @Override
        public CommandResult update(final Robot robot) {
            return CommandResult.COMPLETE;
        }
    
        @Override
        public void init(final Robot robot) {
            robot.getArm().wrist(wristed);
        }
    
        @Override
        public void done(final Robot robot) {
        }
}