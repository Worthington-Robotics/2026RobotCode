package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.superstructure.ShotCalculator.ShootingParams;
import frc.WorBots.subsystems.superstructure.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.shooter.ShooterIO;
import frc.WorBots.subsystems.superstructure.shooter.Shooter.ControlMode;
import frc.WorBots.subsystems.superstructure.turret.Turret;
import frc.WorBots.subsystems.superstructure.turret.TurretIO;
import frc.WorBots.subsystems.superstructure.turret.Turret.TurretControlMode;
import frc.WorBots.util.FireController;
import frc.WorBots.util.math.AllianceFlipUtil;

/**
 * A class to represent the robot's superstructure, which is composed of a shooter (flywheel and hood) and a turret
*/
public class Superstructure extends SubsystemBase {
  // Subsystems
  public Shooter shooter;
  public Turret turret;
  private Drive 
  drive;

  private ShootingParams currentShootingParams = new ShootingParams(false, new Rotation2d(), 0, 0,0);
  private boolean shotValid = false;

  public enum SuperstructureControlMode {
    Disabled,
    ManualShot,
    Voltage,
    AutomaticShot,
    Debug
  }

  // Publishers
  private final NetworkTable superstructure = NetworkTableInstance.getDefault().getTable("Superstructure");
  private final StringPublisher controlModePub = superstructure.getStringTopic("Control Mode").publish();

  private SuperstructureControlMode controlMode = SuperstructureControlMode.Disabled;

  private double flywheelVolts = 0.0;
  private double turretVolts = 0.0;
  private double hoodVolts = 0.0;

  private boolean doAutoPassing = false;
  private boolean hoodDown = false;
  private boolean isPassing = false;

  /**
   * Creates a shooter object
   * @param shooterIO The ShooterIO to use
   * @param turretIO The TurretIO to use
   * @param drive The Drive to use
   */
  public Superstructure(ShooterIO shooterIO, TurretIO turretIO, Drive drive) {
    shooter = new Shooter(shooterIO, drive);
    turret = new Turret(turretIO);
    this.drive = drive;
  }

  public void periodic() {
    //Publish values
    controlModePub.set(controlMode.toString());
    if (controlMode == SuperstructureControlMode.Disabled) {
      turret.setTurretMode(TurretControlMode.Disabled);
      shooter.setShooterMode(ControlMode.Disabled);
    } else if (controlMode == SuperstructureControlMode.AutomaticShot) {
      //Calculate new params
      if (drive.inNeutralZone() && (!DriverStation.isAutonomous() || doAutoPassing)) {
        currentShootingParams = ShotCalculator.getPassParams(drive.getPose(), drive.getFieldrelativeMeasuredSpeeds());
        isPassing = true;
      } else {
        currentShootingParams = ShotCalculator.getHubParams(drive.getPose(), drive.getFieldrelativeMeasuredSpeeds());
        isPassing = false;
      }
      //Apply params
      shotValid = currentShootingParams.isValid();
      shooter.setFlywheelSpeed(currentShootingParams.flywheelspeed());
      shooter.setHoodPose(currentShootingParams.hoodAngle());
      turret.setPosition(currentShootingParams, drive.getPose());
    } else if (controlMode == SuperstructureControlMode.ManualShot) {
      //Apply params
      shotValid = currentShootingParams.isValid();
      shooter.setFlywheelSpeed(currentShootingParams.flywheelspeed());
      shooter.setHoodPose(currentShootingParams.hoodAngle());
      turret.setPosition(currentShootingParams, drive.getPose());
    } else if (controlMode == SuperstructureControlMode.Voltage) {
      //Apply volts
      shooter.setHoodVolts(hoodVolts);
      shooter.setFlywheelVolts(flywheelVolts);
      turret.setVoltage(turretVolts);
    }
    if(hoodDown){
      shooter.setHoodPose(0);
    }
    shooter.periodic();
    turret.periodic();
    Lights.getInstance().addShotStatus(readyToShoot() && FireController.getInstance().readyToFire());
  }

  /**
   * Sets the control mode of the superstructure
   * @param mode The control mode to set
   */
  public void setControlMode(SuperstructureControlMode mode) {
    controlMode = mode;
    if (controlMode == SuperstructureControlMode.AutomaticShot) {
      if (drive.inNeutralZone() && (!DriverStation.isAutonomous() || doAutoPassing)) {
        currentShootingParams = ShotCalculator.getPassParams(drive.getPose(), drive.getFieldrelativeMeasuredSpeeds());
      } else {
        currentShootingParams = ShotCalculator.getHubParams(drive.getPose(), drive.getFieldrelativeMeasuredSpeeds());
      }
      shotValid = currentShootingParams.isValid();
      shooter.setFlywheelSpeed(currentShootingParams.flywheelspeed());
      shooter.setHoodPose(currentShootingParams.hoodAngle());
    }
  }

  /**
   * Runs a manual shot.
   * @param params The shooting params representing the shot to be used
   * @apiNote Sets the superstructure to manual shot mode
   */
  public void runShot(ShootingParams params) {
    controlMode = SuperstructureControlMode.ManualShot;
    currentShootingParams = new ShootingParams(params.isValid(), AllianceFlipUtil.apply(params.turretAngle()), params.hoodAngle(), params.flywheelspeed(), params.turretSpeed());
  }

  public void enableAutoAiming() {
    controlMode = SuperstructureControlMode.AutomaticShot;
  }

  public void setFlyWheelSpeed(double speed) {
    controlMode = SuperstructureControlMode.Debug;
    shooter.setFlywheelSpeed(speed);
  }

  public void setHoodPose(double pose) {
    controlMode = SuperstructureControlMode.Debug;
    shooter.setHoodPose(pose);
  }

  public void setTurretPose(double pose) {
    controlMode = SuperstructureControlMode.Debug;
    turret.setPosition(pose);
  }

  public void setFlyWheelVolts(double volts) {
    shooter.setFlywheelVolts(volts);
  }

  public void setHoodVolts(double volts) {
    shooter.setFlywheelVolts(volts);
  }

  public void setTurretVolts(double volts) {
    turret.setVoltage(volts);
  }

  public double getDesiredHoodPose() {
    return shooter.getDesiredHoodPosition();
  }

  public double getHoodPose() {
    return shooter.getHoodPose();
  }

  public double getDesiredShooterSpeed() {
    return shooter.getDesiredLeaderVelocity();
  }

  public double getShooterSpeed() {
    return shooter.getFlyWheelSpeed();
  }

  public double getDesiredTurretPose() {
    return turret.getDesiredAngle();
  }

  public double getTurretPose() {
    return turret.getPosition();
  }

  public void disable() {
    controlMode = SuperstructureControlMode.Disabled;
  }

  public boolean readyToShoot() {
    return turret.readyToShoot() && shooter.readyToShoot();
  }

  public boolean shotValid() {
    return shotValid;
  }

  public boolean hoodInPosition(){
    return shooter.hoodInPosition();
  }

  public boolean flywheelAtSpeed(){
    return shooter.flywheelAtSpeed();
  }

  public boolean turretReady(){
    return turret.readyToShoot();
  }

  public void setAutoPassing(boolean autoPassing){
    doAutoPassing = autoPassing;
  }

  public void setHoodDown(boolean hoodDown){
    this.hoodDown = hoodDown;
  }

  public boolean isPassing(){
    return isPassing;
  }
}
