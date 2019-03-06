package frc.robot.commands;

import frc.robot.util.Titan;
import frc.robot.Robot;
import frc.robot.components.Intake.HatchState;

public class HatchOuttakeCommand extends Titan.Command<Robot> {
        private final boolean outtaking;

        public HatchOuttakeCommand(final boolean outtaking) {
            this.outtaking = outtaking;

            name = "HatchOuttakeCommand";
            properties = "Whether it will outtake the hatch or not";
        }
    
        @Override
        public CommandResult update(final Robot robot) {
            return CommandResult.COMPLETE;
        }
    
        @Override
        public void init(final Robot robot) {
            robot.getIntake().actuateHatch(outtaking ? HatchState.DOWN : HatchState.UP);
        }
    
        @Override
        public void done(final Robot robot) {
        }
}