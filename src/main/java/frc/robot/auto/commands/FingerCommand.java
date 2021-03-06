package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.Robot;
import frc.robot.components.Intake.FingerState;

public class FingerCommand extends Titan.Command<Robot> {
        private final FingerState fingered;

        public FingerCommand(final FingerState fingered) {
            this.fingered = fingered;

            name = "FingerCommand";
            properties = String.format("State: %s", fingered);
        }
    
        @Override
        public CommandResult update(final Robot robot) {
            return CommandResult.COMPLETE;
        }
    
        @Override
        public void init(final Robot robot) {
            robot.getIntake().finger(fingered);
        }
    
        @Override
        public void done(final Robot robot) {
        }
}