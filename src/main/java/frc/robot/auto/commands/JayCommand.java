package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.components.Intake.JayState;

public class JayCommand extends CommandBase {
    private final JayState jayed;

    public JayCommand(final JayState jayed) {
        this.jayed = jayed;

        // name = "JayCommand";
        // properties = String.format("State: %s", jayed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        Robot.getRobot().getIntake().jay(jayed);
    }
}