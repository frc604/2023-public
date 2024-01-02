package frc.quixlib.wpilib;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;

/**
 * A command composition that runs a list of commands in sequence.
 *
 * <p>Similar to WPILib's SequentialCommandGroup, but doesn't require all subsystems at the
 * beginning, but rather only on use.
 */
public class LazySequentialCommandGroup extends Command {
  private final List<Command> m_commands = new ArrayList<>();
  private int m_commandIndex = 0;
  private int m_lastScheduledIndex = -1;

  public LazySequentialCommandGroup(Command... commands) {
    for (Command command : commands) {
      m_commands.add(command);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_commandIndex = 0;
    m_lastScheduledIndex = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Command command = m_commands.get(m_commandIndex);
    if (m_lastScheduledIndex != m_commandIndex) {
      command.schedule();
      m_lastScheduledIndex = m_commandIndex;
    } else if (m_lastScheduledIndex == m_commandIndex && command.isFinished()) {
      m_commandIndex++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_commandIndex >= m_commands.size();
  }

  @Override
  public void cancel() {
    super.cancel();
    for (Command command : m_commands) {
      command.cancel();
    }
  }
}
