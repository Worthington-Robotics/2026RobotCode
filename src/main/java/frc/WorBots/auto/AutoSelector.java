// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.auto;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.debug.SwitchableChooser;
import java.util.ArrayList;
import java.util.List;

/**
 * Subsystem for running a persistent auto selector on the dashboard. Can either display multiple
 * dropdown choosers on Shuffleboard or one dropdown on the dashboard
 */
public class AutoSelector extends SubsystemBase {
  /** The maximum number of questions */
  public static final int MAX_QUESTIONS = 4;

  /** Whether to use the LabVIEW dashboard instead of multiple question choosers */
  private static final boolean useDriverStation = true;

  private static final AutoRoutine defaultRoutine =
      new AutoRoutine("Do Nothing", Commands.none());

  private SwitchableChooser routineChooser;
  private List<StringPublisher> questionPublishers;
  private List<SwitchableChooser> questionChoosers;

  private AutoRoutine lastRoutine;
  private List<String> names = new ArrayList<>();
  private List<AutoRoutine> routines = new ArrayList<>();
  private List<String> answers = new ArrayList<>();

  /**
   * The auto selector is logged onto SmartDashboard and allows for the drivers to set the desired
   * auto before a match.
   *
   * @param key The SmartDashboard table to be logged under.
   */
  public AutoSelector(String key) {
    lastRoutine = defaultRoutine;

    if (!useDriverStation) {
      routineChooser = new SwitchableChooser(key);
      questionPublishers = new ArrayList<>();
      questionChoosers = new ArrayList<>();
      for (int i = 0; i < MAX_QUESTIONS; i++) {
        var publisher =
            NetworkTableInstance.getDefault()
                .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
                .publish();
        publisher.set("NA");
        questionPublishers.add(publisher);
        questionChoosers.add(
            new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
      }
    }
  }

  /**
   * Adds an auto to the auto selector
   *
   * @param name The name of the auto
   * @param questions The auto questions to be asked
   * @param command The command that needs to be ran
   */
  public void addRoutine(String name, Command command) {
    addRoutine(new AutoRoutine(name, command));
  }

  /**
   * Adds a routine to the auto selector
   *
   * @param routine The routine to add
   */
  public void addRoutine(AutoRoutine routine) {
    names.add(routine.name);
    routines.add(
        new AutoRoutine(routine.name, createRoutineCommand(routine.command)));
    if (useDriverStation) {
      createAnswerList();
    } else {
      routineChooser.setOptions(names.toArray(new String[0]));
    }
  }

  /**
   * Gets a routine from it's name
   *
   * @param name The name of the routine
   * @return The routine
   */
  public AutoRoutine getRoutineFromName(String name) {
    AutoRoutine returnRoutine = null;
    for (AutoRoutine routine : routines) {
      if (routine.name.equalsIgnoreCase(name)) {
        returnRoutine = routine;
      }
    }
    return returnRoutine;
  }

  /** Creates the answer list for the Dashboard chooser */
  private void createAnswerList() {
    answers.clear();
    for (AutoRoutine routine : routines) {
        answers.add(routine.name);
    }
  }

  public void periodic() {
    // Don't run expensive periodic code if we are enabled
    if (DriverStation.isAutonomousEnabled()
        || DriverStation.isTeleopEnabled() && lastRoutine != null) {
      return;
    }
    if (useDriverStation) {
      SmartDashboard.putStringArray("Auto List", answers.toArray(new String[0]));
      String answer = SmartDashboard.getString("Auto Selector", "Do Nothing");
      if (answer == null) {
        return;
      }
      String[] items = answer.split("; ");
      var selectedRoutine = getRoutineFromName(items[0]);
      if (selectedRoutine == null) {
        return;
      }

      lastRoutine = selectedRoutine;
      SmartDashboard.putString("Actually Selected Auto", lastRoutine.name);
      //StatusPage.reportStatus(StatusPage.ALL_AUTO_QUESTIONS, true);
    } else {
      routineChooser.periodic();

      var selectedRoutine = getRoutineFromName(routineChooser.get());
      if (selectedRoutine == null) {
        return;
      }
      for (SwitchableChooser chooser : questionChoosers) {
        chooser.periodic();
      }
    }
      //StatusPage.reportStatus(StatusPage.ALL_AUTO_QUESTIONS, allQuestionsChosen);
    //StatusPage.reportStatus(StatusPage.AUTO_CHOSEN, lastRoutine != null);
  }

  /**
   * Return the selected auto command.
   *
   * @return The command.
   */
  public Command getCommand() {
    StringBuilder attempted = new StringBuilder(lastRoutine.name);
    SmartDashboard.putString("Auto Attempted", attempted.toString());
    return lastRoutine.command();
  }

  /**
   * We have to do this once at the beginning when the command is registered otherwise WPILib gives
   * us errors
   */
  private static Command createRoutineCommand(Command command) {
    return command;
  }

  /** A named auto routine */
  private static final record AutoRoutine(
      String name, Command command) {}
}
