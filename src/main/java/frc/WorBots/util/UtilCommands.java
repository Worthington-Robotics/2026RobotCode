// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class UtilCommands {
  /**
   * Returns a command with a SmartDashboard entry that displays whether it is running or not
   *
   * @param command The command to decorate
   * @param key The SmartDashboard key to use
   * @return The command to run
   */
  public static Command named(Command command, String key) {
    return Commands.startEnd(
            () -> SmartDashboard.putBoolean(key, true), () -> SmartDashboard.putBoolean(key, false))
        .raceWith(command);
  }

  /**
   * Returns a command sequence with a SmartDashboard entry that displays progress
   *
   * @param key The SmartDashboard key to use
   * @param commands The commands to run
   * @return The command
   */
  public static Command namedSequence(String key, Command... commands) {
    return namedSequence(key, List.of(commands));
  }

  /**
   * Returns a command sequence with a SmartDashboard entry that displays progress
   *
   * @param key The SmartDashboard key to use
   * @param commands The commands to run
   * @return The command
   */
  public static Command namedSequence(String key, List<Command> commands) {
    final ArrayList<Command> newCommands = new ArrayList<>(commands.size() + 1);
    newCommands.add(Commands.runOnce(() -> SmartDashboard.putNumber(key, 0)));
    for (int i = 0; i < commands.size(); i++) {
      final int finalI = i;
      newCommands.add(
          optimalSequence(
              Commands.runOnce(() -> SmartDashboard.putNumber(key, finalI)), commands.get(i)));
    }
    return optimalSequence(newCommands);
  }

  /**
   * Creates a command that waits for Button 1 on the Driverstation to be pressed
   *
   * @return The command to run
   */
  public static Command waitForDriverstationButton() {
    return optimalSequence(
        Commands.waitUntil(() -> SmartDashboard.getBoolean("DB/Button 1", false)),
        Commands.runOnce(() -> SmartDashboard.putBoolean("DB/Button 1", false)),
        Commands.waitUntil(() -> !SmartDashboard.getBoolean("DB/Button 1", true)));
  }

  /**
   * Returns a command that times how long the command inside it takes
   *
   * @param name The key to put the time in SmartDashboard
   * @param command The command to time
   * @return The wrapped command
   */
  public static Command timer(String name, Command command) {
    return new TimedCommand(command, name);
  }

  /**
   * Returns a command that runs a sequence of commands optimally with no delay
   *
   * @param commands The commands to run in the sequence
   * @return The command
   */
  public static Command optimalSequence(Command... commands) {
    return new OptimalSequentialCommandGroup(commands);
  }

  /**
   * Returns a command that runs a sequence of commands optimally with no delay
   *
   * @param commands The commands to run in the sequence
   * @return The command
   */
  public static Command optimalSequence(List<Command> commands) {
    return new OptimalSequentialCommandGroup(commands);
  }

  /*
   * Returns a ConditionalCommand that only has the requirements of whatever
   * command is selected. Use wisely.
   */
  public static Command eitherNoRequire(
      Command whenTrue, Command whenFalse, BooleanSupplier condition) {
    return new ConditionalCommandNoRequire(whenTrue, whenFalse, condition);
  }

  /** A command wrapped with a timer that will be shown on SmartDashboard */
  public static class TimedCommand extends Command {
    private Command command;
    private final Timer timer = new Timer();
    private final String name;

    public TimedCommand(Command command, String name) {
      this.command = command;
      this.name = name;
    }

    @Override
    public void initialize() {
      timer.restart();
      if (command != null) {
        command.schedule();
      }
    }

    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        command.cancel();
      }
      SmartDashboard.putNumber(name, timer.get());
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
      return command == null || !command.isScheduled();
    }

    @Override
    public boolean runsWhenDisabled() {
      return command.runsWhenDisabled();
    }
  }

  /**
   * A command composition that runs a list of commands in sequence, with optimal execution. Unlike
   * the normal SequentialCommandGroup, when this command runs a subcommand that is already
   * finished, it will immediately start the next one in the sequence without the usual
   * CommandScheduler overhead.
   */
  public static class OptimalSequentialCommandGroup extends Command {
    private final List<Command> commands = new ArrayList<>();
    private int currentCommandIndex = -1;
    private boolean runWhenDisabled = true;
    private InterruptionBehavior interruptBehavior = InterruptionBehavior.kCancelIncoming;

    /**
     * Creates a new SequentialCommandGroup. The given commands will be run sequentially, with the
     * composition finishing when the last command finishes.
     *
     * @param commands the commands to include in this composition.
     */
    public OptimalSequentialCommandGroup(Command... commands) {
      addCommands(List.of(commands));
    }

    /**
     * Creates a new SequentialCommandGroup. The given commands will be run sequentially, with the
     * composition finishing when the last command finishes.
     *
     * @param commands the commands to include in this composition.
     */
    public OptimalSequentialCommandGroup(List<Command> commands) {
      addCommands(commands);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add, in order of execution.
     */
    public final void addCommands(List<Command> commands) {
      if (currentCommandIndex != -1) {
        throw new IllegalStateException(
            "Commands cannot be added to a composition while it's running");
      }

      for (var command : commands) {
        CommandScheduler.getInstance().registerComposedCommands(command);
      }

      for (Command command : commands) {
        this.commands.add(command);
        addRequirements(command.getRequirements());
        runWhenDisabled &= command.runsWhenDisabled();
        if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
          interruptBehavior = InterruptionBehavior.kCancelSelf;
        }
      }
    }

    @Override
    public final void initialize() {
      currentCommandIndex = 0;

      if (!commands.isEmpty()) {
        commands.get(0).initialize();
      }
    }

    @Override
    public final void execute() {
      if (commands.isEmpty()) {
        return;
      }

      while (true) {
        if (currentCommandIndex >= commands.size()) {
          break;
        }

        final Command currentCommand = commands.get(currentCommandIndex);

        currentCommand.execute();
        if (currentCommand.isFinished()) {
          currentCommand.end(false);
          currentCommandIndex++;
          if (currentCommandIndex < commands.size()) {
            commands.get(currentCommandIndex).initialize();
          }
        } else {
          break;
        }
      }
    }

    @Override
    public final void end(boolean interrupted) {
      if (interrupted
          && !commands.isEmpty()
          && currentCommandIndex > -1
          && currentCommandIndex < commands.size()) {
        commands.get(currentCommandIndex).end(true);
      }
      currentCommandIndex = -1;
    }

    @Override
    public final boolean isFinished() {
      return currentCommandIndex == commands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
      return runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
      return interruptBehavior;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);

      builder.addIntegerProperty("index", () -> currentCommandIndex, null);
    }
  }

  /*
   * A conditional command that only has the requirements of whatever command is
   * selected. Use wisely.
   */
  public static class ConditionalCommandNoRequire extends Command {
    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;
    private Command selectedCommand;
    private boolean finished = true;

    /**
     * Creates a new ConditionalCommand.
     *
     * @param onTrue the command to run if the condition is true
     * @param onFalse the command to run if the condition is false
     * @param condition the condition to determine which command to run
     */
    @SuppressWarnings("this-escape")
    public ConditionalCommandNoRequire(Command onTrue, Command onFalse, BooleanSupplier condition) {
      this.onTrue = requireNonNullParam(onTrue, "onTrue", "ConditionalCommand");
      this.onFalse = requireNonNullParam(onFalse, "onFalse", "ConditionalCommand");
      this.condition = requireNonNullParam(condition, "condition", "ConditionalCommand");

      // CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);
    }

    // We have to do some mumbo-jumbo with dynamic requirements for this to work
    // @Override
    // public Set<Subsystem> getRequirements() {
    // if (condition.getAsBoolean()) {
    // return onTrue.getRequirements();
    // } else {
    // return onFalse.getRequirements();
    // }
    // }

    @Override
    public void initialize() {
      if (condition.getAsBoolean()) {
        selectedCommand = onTrue;
      } else {
        selectedCommand = onFalse;
      }
      if (finished) {
        selectedCommand.schedule();
      }
      finished = false;
    }

    @Override
    public void execute() {
      // selectedCommand.execute();
      if (selectedCommand.isFinished()) {
        finished = true;
      }
    }

    @Override
    public void end(boolean interrupted) {
      selectedCommand.cancel();
    }

    @Override
    public boolean isFinished() {
      return finished;
    }

    @Override
    public boolean runsWhenDisabled() {
      return onTrue.runsWhenDisabled() && onFalse.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
      if (onTrue.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf
          || onFalse.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        return InterruptionBehavior.kCancelSelf;
      } else {
        return InterruptionBehavior.kCancelIncoming;
      }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addStringProperty("onTrue", onTrue::getName, null);
      builder.addStringProperty("onFalse", onFalse::getName, null);
      builder.addStringProperty(
          "selected",
          () -> {
            if (selectedCommand == null) {
              return "null";
            } else {
              return selectedCommand.getName();
            }
          },
          null);
    }
  }
}
