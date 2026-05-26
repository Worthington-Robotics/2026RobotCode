// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightEffects;
import frc.WorBots.subsystems.lights.Lights.LightsTarget;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.util.debug.StatusPage;

public class FireController {
  private Superstructure superstructure;
  private static FireController instance;
  private Drive drive;
  private Spindexer spindexer;

  public FireController(Superstructure superstructure, Drive drive, Spindexer spindexer) {
    this.spindexer = spindexer;
    this.superstructure = superstructure;
    this.drive = drive;
    instance = this;
    StatusPage.reportStatus(StatusPage.FIRE_CONTROL, true);
  }

  public static FireController getInstance() {
    return instance;
  }

  public boolean shouldAgitate(){
    return !superstructure.isPassing() && spindexer.getSpinGoalVelocity() > 0.0;
  }

  /**
   * Most basic check for if the robot is ready to fire
   * 
   * @return if the subsystems are at goals, does not acount for time or
   *         constraints from the F.A.S.
   */
  private boolean subsystemsAtGoals() {
    return superstructure.readyToShoot();
  }

  /**
   * More advanced check that predicts if the ball will arrive at a active hub,
   * does not account for hub deactivating
   * 
   * @return are the subsystems at their goals, and the ball that will fire will
   *         arrive to a active hub
   */
  public boolean timeAcceptable() {
    return true; //Constants.TurretShooterConstants.PREFIRE_LIMIT > utils.timeToHubActive() || (utils.timeToAcivationSwitch() > 24 && !utils.isHubActive());
  }

  /**
   * Checks if the subsystems are actually at the goal the shot calc wants or if
   * they are being constrained for saftey
   * 
   * @return if the robot is ready to fire
   */
  public boolean constrained() {
    boolean output = false;
    if (drive.nearTrench()) {
      output = false;
    } else {
      output = true;
    }
    return output;
  }

  /**
   * Combines all the internal checks to figure out if we are cleared to shoot (Time, trench, and System status)
   * @return if we are cleared for firing
   */
  public boolean readyToFire() {
    boolean output;
    if (StatusPage.getStatus(StatusPage.TAG_VISION_SUBSUBSYSTEM)) {
      output = subsystemsAtGoals();
    } else {
      output = subsystemsAtGoals();
    }

    if (DriverStation.getAlliance().isPresent() && ((DriverStation.getAlliance().get() == Alliance.Red && drive.inRedZone())
        || (DriverStation.getAlliance().get() == Alliance.Blue && drive.inBlueZone()))) {
      Lights.getInstance().setTarget(LightsTarget.Hub);
    } else {
      Lights.getInstance().setTarget(LightsTarget.Pass);
    }

    Lights.getInstance().runEffect(LightEffects.invalidShotFlash);
    return output;
  }
}
