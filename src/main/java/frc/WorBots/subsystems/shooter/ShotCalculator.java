package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.WorBots.Constants;

/* Taken and modified from team 6328. */

/***
 * A class that can be used to calclate the required parameters for the robot's
 * next shot
 */
public class ShotCalculator {
  private static ShotCalculator instance;

  public static ShotCalculator getInstance() {
    if (instance == null)
      instance = new ShotCalculator();
    return instance;
  }

  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;

  /***
   * A class to represent all aspects of a shot
   */
  public record ShootingParams(
      boolean isValid,
      Rotation2d turretAngle,
      double hoodAngle,
      double flywheelspeed) {
  }

  ShootingParams latestParams = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  // Set up interpolating tree tables
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = new InterpolatingTreeMap<>(
      InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlyWheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    minDistance = 0.0; // TODO set this //Thee minimum distance the robot can shoot
    maxDistance = 10.0; // TODO set this //The maximum distance the robot can shoot
    phaseDelay = 0.03; // TODO set this

    shotHoodAngleMap.put(0.0, Rotation2d.fromDegrees(0.0)); // TODO add actual values

    shotFlyWheelSpeedMap.put(0.0, 0.0); // TODO add actual values

    timeOfFlightMap.put(0.0, 0.0); // TODO add actual values
  }

  /***
   * Gets the parameters required to make a shot into the Hub
   * 
   * @param pose          The current field relative robot position
   * @param robotVelocity The current field relative robot velocity
   * @return The parameters required to make a shot into the hub with the current
   *         robot position and velocity
   */
  public ShootingParams getParams(Pose2d pose, ChassisSpeeds robotVelocity) {
    // Calculate the estimated robot pose when this method is done running
    Pose2d estimatedPose = pose
        .exp(ChassisSpeeds.fromFieldRelativeSpeeds(robotVelocity, pose.getRotation()).toTwist2d(phaseDelay));
    // Calculate the distance from the turret to the target
    Translation2d target = new Translation2d(); // TODO set this
    Pose2d turretPosition = estimatedPose.transformBy(Constants.ROBOT_TO_TURRET);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX = robotVelocity.vxMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
            * (Constants.ROBOT_TO_TURRET.getY() * Math.cos(robotAngle)
                - Constants.ROBOT_TO_TURRET.getX() * Math.sin(robotAngle));
    double turretVelocityY = robotVelocity.vyMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
            * (Constants.ROBOT_TO_TURRET.getX() * Math.cos(robotAngle)
                - Constants.ROBOT_TO_TURRET.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot to offset
    double timeOfFlight;
    Pose2d lookAheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookAheadPose = new Pose2d(
          turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
          turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookAheadPose.getTranslation());
    }

    // Calculate params
    turretAngle = target.minus(lookAheadPose.getTranslation()).getAngle();
    hoodAngle = shotHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    latestParams = new ShootingParams(lookaheadTurretToTargetDistance >= minDistance
        && lookaheadTurretToTargetDistance <= maxDistance,
        turretAngle,
        hoodAngle,
        shotFlyWheelSpeedMap.get(lookaheadTurretToTargetDistance));
    return latestParams;
  }
}
