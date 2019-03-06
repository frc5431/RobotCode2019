package frc.robot.commands;

import frc.robot.util.Titan;
import frc.robot.Robot;

public class JayCommand extends Titan.Command<Robot> {
        private final boolean jayed;
        //private boolean wasFingering = false;

        public JayCommand(final boolean jayed) {
            this.jayed = jayed;

            name = "JayCommand";
            properties = "Whether it will jay or not";
        }
    
        @Override
        public CommandResult update(final Robot robot) {
            // if(jayed == false && !wasFingering){
            //     robot.getIntake().finger(false);
            //     if(System.currentTimeMillis() > startTime + 100){
            //         return CommandResult.COMPLETE;
            //     }else{
            //         return CommandResult.IN_PROGRESS;
            //     }
            // }else{
                return CommandResult.COMPLETE;
            //}
        }
    
        @Override
        public void init(final Robot robot) {
            robot.getIntake().jay(jayed);

            //wasFingering = robot.getIntake().isFingering();
        }
    
        @Override
        public void done(final Robot robot) {
        }
}