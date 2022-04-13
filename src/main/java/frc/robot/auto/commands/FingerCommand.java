package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.components.Intake.FingerState;

public class FingerCommand extends CommandBase {
    private final FingerState fingered;

    public FingerCommand(final FingerState fingered) {
        this.fingered = fingered;

        // name = "FingerCommand";
        // properties = String.format("State: %s", fingered);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        Robot.getRobot().getIntake().finger(fingered);
    }
}