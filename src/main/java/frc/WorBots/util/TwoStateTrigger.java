// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class TwoStateTrigger {
  final Command whenTrue;
  final Command whenFalse;
  final BooleanSupplier triggerCondition;
  final BooleanSupplier stateCondition;
  final EventLoop eventLoop;

  public TwoStateTrigger(
      Command whenTrue, Command whenFalse, Trigger trigger, BooleanSupplier stateCondition) {
    this(
        whenTrue,
        whenFalse,
        () -> trigger.getAsBoolean(),
        stateCondition,
        CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public TwoStateTrigger(
      Command whenTrue,
      Command whenFalse,
      BooleanSupplier triggerCondition,
      BooleanSupplier stateCondition) {
    this(
        whenTrue,
        whenFalse,
        triggerCondition,
        stateCondition,
        CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public TwoStateTrigger(
      Command whenTrue,
      Command whenFalse,
      BooleanSupplier triggerCondition,
      BooleanSupplier stateCondition,
      EventLoop eventLoop) {
    this.whenTrue = whenTrue;
    this.whenFalse = whenFalse;
    this.triggerCondition = triggerCondition;
    this.stateCondition = stateCondition;
    this.eventLoop = eventLoop;
  }

  private void addBinding(BindingBody body) {
    eventLoop.bind(
        new Runnable() {
          private boolean m_previous = triggerCondition.getAsBoolean();

          @Override
          public void run() {
            boolean current = triggerCondition.getAsBoolean();

            body.run(m_previous, current);

            m_previous = current;
          }
        });
  }

  private interface BindingBody {
    /**
     * Executes the body of the binding.
     *
     * @param previous The previous state of the condition.
     * @param current The current state of the condition.
     */
    void run(boolean previous, boolean current);
  }

  private void runCommand() {
    cancelCommand();
    if (stateCondition.getAsBoolean()) {
      whenTrue.schedule();
    } else {
      whenFalse.schedule();
    }
  }

  private void cancelCommand() {
    whenTrue.cancel();
    whenFalse.cancel();
  }

  /**
   * Starts the given command whenever the condition changes from `false` to `true`.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  public TwoStateTrigger onTrue() {
    addBinding(
        (previous, current) -> {
          if (!previous && current) {
            runCommand();
          }
        });

    return this;
  }

  public TwoStateTrigger whileTrue() {
    addBinding(
        (previous, current) -> {
          if (!previous && current) {
            runCommand();
          } else if (previous && !current) {
            cancelCommand();
          }
        });
    return this;
  }

  public TwoStateTrigger toggleOnTrue() {
    addBinding(
        (previous, current) -> {
          if (!previous && current) {
            runCommand();
          }
        });
    return this;
  }
}
