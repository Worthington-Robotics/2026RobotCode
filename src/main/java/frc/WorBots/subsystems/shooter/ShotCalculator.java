package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.WorBots.Constants;
import frc.WorBots.FieldConstants;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.GeomUtil;

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
  // Set up interpolating tree maps for scoring
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = new InterpolatingTreeMap<>(
      InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  //Set up interpolating tree maps for passing
  private static final InterpolatingTreeMap<Double, Rotation2d> passHoodAngleMap = new InterpolatingTreeMap<>(
      InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passTimeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    minDistance = 0.0; // TODO set this //Thee minimum distance the robot can shoot
    maxDistance = 10.0; // TODO set this //The maximum distance the robot can shoot
    phaseDelay = 0.03; // TODO set this

    //Add values to scoring interpolating tree maps
    shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0)); //TODO set real values
    shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    shotFlywheelSpeedMap.put(1.34, 210.0); //TODO set real values
    shotFlywheelSpeedMap.put(1.78, 220.0);
    shotFlywheelSpeedMap.put(2.17, 220.0);
    shotFlywheelSpeedMap.put(2.81, 230.0);
    shotFlywheelSpeedMap.put(3.82, 250.0);
    shotFlywheelSpeedMap.put(4.09, 255.0);
    shotFlywheelSpeedMap.put(4.40, 260.0);
    shotFlywheelSpeedMap.put(4.77, 265.0);
    shotFlywheelSpeedMap.put(5.57, 275.0);
    shotFlywheelSpeedMap.put(5.60, 290.0);

    timeOfFlightMap.put(5.68, 1.16); //TODO set real values
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);

    
    // Add values to Passing interpolating tree maps
    passHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0)); //TODO set real values
    passHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    passHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    passHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    passHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    passHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    passHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    passHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    passHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    passHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    passFlywheelSpeedMap.put(1.34, 210.0); //TODO set real values
    passFlywheelSpeedMap.put(1.78, 220.0);
    passFlywheelSpeedMap.put(2.17, 220.0);
    passFlywheelSpeedMap.put(2.81, 230.0);
    passFlywheelSpeedMap.put(3.82, 250.0);
    passFlywheelSpeedMap.put(4.09, 255.0);
    passFlywheelSpeedMap.put(4.40, 260.0);
    passFlywheelSpeedMap.put(4.77, 265.0);
    passFlywheelSpeedMap.put(5.57, 275.0);
    passFlywheelSpeedMap.put(5.60, 290.0);

    passTimeOfFlightMap.put(5.68, 1.16); //TODO set real values
    passTimeOfFlightMap.put(4.55, 1.12);
    passTimeOfFlightMap.put(3.15, 1.11);
    passTimeOfFlightMap.put(1.88, 1.09);
    passTimeOfFlightMap.put(1.38, 0.90);
  }

  /***
   * Gets the parameters required to make a shot into the Hub
   * 
   * @param pose          The current field relative robot position
   * @param robotVelocity The current field relative robot velocity
   * @param doOverRide Whether to override the controls to prevent the robot form illegally shooting
   * @return The parameters required to make a shot into the hub with the current
   *         robot position and velocity
   */
  public ShootingParams getParamsToHub(Pose2d pose, ChassisSpeeds robotVelocity) {
    Translation2d target = AllianceFlipUtil.apply(FieldConstants.hubPosition);
    return getParams(pose, robotVelocity, target, passHoodAngleMap, passFlywheelSpeedMap, passTimeOfFlightMap, true);
  }

  /*** Gets the shooter parameters in order to pass 
   * @param Pose The robot's position
   * @param robotVelocity The robot's velocity
  */
  public ShootingParams getPassParams(Pose2d pose, ChassisSpeeds robotVelocity){
    boolean isValid = true;
    Translation2d turretPose = pose.getTranslation(); //TODO add translating from robot to turret
    Translation2d targetPose;
    if(GeomUtil.translation2dInBoundingBox(turretPose, null)){ //TODO add bounds
      targetPose = AllianceFlipUtil.apply(new Translation2d()); //TODO add a real value
    } else if (GeomUtil.translation2dInBoundingBox(turretPose, null)) {
      targetPose = AllianceFlipUtil.apply(new Translation2d()); //TODO add a real value
    } else {
      return new ShootingParams(false, new Rotation2d(), 0, 0);
    }
    Translation2d[] shotPath = {turretPose, targetPose};
    if(GeomUtil.doesLinePassThroughArea(shotPath, AllianceFlipUtil.apply(FieldConstants.passExclusionZone))){ //TODO make sure alliance flip for arrays is working correctly 
      isValid=false;
    }
    return getParams(pose, robotVelocity, targetPose, passHoodAngleMap, passFlywheelSpeedMap, passTimeOfFlightMap, isValid);
  }

  /*** An internal method used for getting the shot parameters to a pose 
   * @param robotPose The position of the robot
   * @param robotVelocity The robot's velocity
   * @param target The target of the shot
   * @param hoodAngleMap The interpolating tree map to use for calculating shooter angle
   * @param flywheelSpeedMap The interpolating tree map to use for calculating the flywheel speed
   * @param timeOfFlightMap The interpolating tree map to use for calculating the time of flight of a shot
   * @param isValid If the shot is a valid shot
  */
  private ShootingParams getParams(Pose2d robotPose, ChassisSpeeds robotVelocity, Translation2d target, InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap, InterpolatingDoubleTreeMap flywheelSpeedMap, InterpolatingDoubleTreeMap timeOfFlightMap, boolean isValid){
    // Calculate the estimated robot pose when this method is done running
    Pose2d estimatedPose = robotPose
        .exp(ChassisSpeeds.fromFieldRelativeSpeeds(robotVelocity, robotPose.getRotation()).toTwist2d(phaseDelay));
    // Calculate the distance from the turret to the target
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
    double lastOffsetX = 0.0;
    double lastOffsetY = 0.0;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight - lastOffsetX;
      double offsetY = turretVelocityY * timeOfFlight - lastOffsetY;
      lastOffsetX = offsetX;
      lastOffsetY = offsetY;
      lookAheadPose = new Pose2d(
          turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
          turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookAheadPose.getTranslation());
    }

    // Calculate params
    turretAngle = target.minus(lookAheadPose.getTranslation()).getAngle();
    hoodAngle = hoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    latestParams = new ShootingParams(lookaheadTurretToTargetDistance >= minDistance
        && lookaheadTurretToTargetDistance <= maxDistance && isValid,
        turretAngle,
        hoodAngle,
        flywheelSpeedMap.get(lookaheadTurretToTargetDistance));
    return latestParams;
}

}
