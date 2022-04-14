package frc.robot.auto.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;

public class CommandQueue extends CommandGroupBase {
    private final List<Command> m_commands = new ArrayList<>();
    private int m_currentCommandIndex = -1;
    private boolean m_runWhenDisabled = true;

    public CommandQueue(Command... commands) {
        addCommands(commands);
    }

    public boolean isEmpty() {
        return m_commands.isEmpty();
    }

    public void clear() {
        m_commands.clear();
    }

    public void add(Command command) {
        addCommands(command);
    }

    public void addAll(CommandQueue queue) {
        addAll(queue.getCommands());
    }

    public void addAll(List<? extends Command> commands) {
        addCommands(commands.toArray(new Command[0]));
    }

    public List<Command> getCommands() {
        return m_commands;
    }

    @Override
    public final void addCommands(Command... commands) {
        requireUngrouped(commands);

        if (m_currentCommandIndex != -1) {
            throw new IllegalStateException(
                "Commands cannot be added to a CommandGroup while the group is running");
        }

        for (Command command : commands) {
            m_commands.add(command);
            m_requirements.addAll(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
        }
    }

    @Override
    public void initialize() {
        m_currentCommandIndex = 0;

        if (!m_commands.isEmpty()) {
            m_commands.get(0).initialize();
        }
    }

    @Override
    public void execute() {
        if (m_commands.isEmpty()) {
            return;
        }

        Command currentCommand = m_commands.get(m_currentCommandIndex);

        currentCommand.execute();
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            m_currentCommandIndex++;
            if (m_currentCommandIndex < m_commands.size()) {
                m_commands.get(m_currentCommandIndex).initialize();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted
                && !m_commands.isEmpty()
                && m_currentCommandIndex > -1
                && m_currentCommandIndex < m_commands.size()) {
            m_commands.get(m_currentCommandIndex).end(true);
        }
        m_currentCommandIndex = -1;
    }

    @Override
    public boolean isFinished() {
        return m_currentCommandIndex == m_commands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }
}
