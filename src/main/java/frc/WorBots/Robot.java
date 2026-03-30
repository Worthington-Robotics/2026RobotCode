// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.WorBots;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.MatchTime;
import frc.WorBots.util.OdometryThread;
import frc.WorBots.util.cache.Cache.TimeCache;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.math.GeneralMath;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private boolean useDebugBindings = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
    // Set robot period
    SmartDashboard.putBoolean("DebugBindingsEnabed", false);
    this.addPeriodic(this::realRobotPeriodic, Constants.RobotConstants.ROBOT_PERIOD);

    // Silences Joystick warning in SIM
    if (Constants.getSim()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    } else {
      DriverStation.silenceJoystickConnectionWarning(false);
    }

    OdometryThread.getInstance();
    Lights.getInstance();
  }

  public void realRobotPeriodic() {
    boolean temp = SmartDashboard.getBoolean("DebugBindingsEnabed", false);
    if (temp != useDebugBindings) {
      if (temp) {
      } else {
      }
    }
    CommandScheduler.getInstance().run();
    Lights.getInstance().periodic();
    TimeCache.getInstance().update();
    StatusPage.periodic();
    m_robotContainer.updatePowerLogs();
  }

  @Override
  public void robotInit(){
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    super.robotInit();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.ranAuto();
    m_robotContainer.disableSubsystems();
    MatchTime.getInstance().startAuto();
    TimeCache.getInstance().update();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.disableSubsystems();
  }

  @Override
  public void teleopInit() {
    m_robotContainer.disableSubsystems();
    m_robotContainer.teleopInitSubsystems();
    MatchTime.getInstance().startTeleop();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    StatusPage.reportStatus("Climbing", false);

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void robotPeriodic(){}
}
