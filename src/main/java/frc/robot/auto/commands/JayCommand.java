package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.Robot;
import frc.robot.components.Intake.JayState;

public class JayCommand extends Titan.Command<Robot> {
        private final JayState jayed;

        public JayCommand(final JayState jayed) {
            this.jayed = jayed;

            name = "JayCommand";
            properties = String.format("State: %s", jayed);
        }
    
        @Override
        public CommandResult update(final Robot robot) {
            return CommandResult.COMPLETE;
        }
    
        @Override
        public void init(final Robot robot) {
            robot.getIntake().jay(jayed);
        }
    
        @Override
        public void done(final Robot robot) {
        }
}