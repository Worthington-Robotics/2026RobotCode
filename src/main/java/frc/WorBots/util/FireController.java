package frc.WorBots.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightEffects;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.util.debug.StatusPage;

public class FireController {
  private Superstructure superstructure;
  private RebuiltUtils utils;
  private static FireController instance;
  private Drive drive;

  public FireController(Superstructure superstructure, Drive drive) {
    this.superstructure = superstructure;
    this.drive = drive;
    instance = this;
    StatusPage.reportStatus(StatusPage.FIRE_CONTROL, true);
  }

  public static FireController getInstance() {
    return instance;
  }

  public boolean shouldAgitate(){
    return !superstructure.isPassing() && Math.abs(superstructure.getDesiredTurretPose()) > Units.degreesToRadians(90);
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

    if ((DriverStation.getAlliance().get() == Alliance.Red && drive.inRedZone())
        || (DriverStation.getAlliance().get() == Alliance.Blue && drive.inBlueZone())) {
      output = output && timeAcceptable();
    }

    Lights.getInstance().runEffect(LightEffects.invalidShotFlash);
    return output;
  }
}
