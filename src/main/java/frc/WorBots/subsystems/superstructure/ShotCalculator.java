package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.Constants;
import frc.WorBots.FieldConstants;
import frc.WorBots.util.debug.TunableDouble;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.GeomUtil;

/* Taken and modified from team 6328. */

/***
 * A class that can be used to calclate the required parameters for the robot's
 * next shot
 */
public class ShotCalculator {
  static TunableDouble paraTuning = new TunableDouble("Shooter", "Tuning", "Para Tuning");
  /***
   * A class to represent all aspects of a shot
   */
  public record ShootingParams(
      boolean isValid,
      Rotation2d turretAngle,
      double hoodAngle,
      double flywheelspeed,
      double turretSpeed) {
  }

  // Set up interpolating tree maps for scoring
  private static final InterpolatingTreeMap<Double, Rotation2d> hubHoodAngleMap = new InterpolatingTreeMap<>(
      InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap hubFlywheelSpeedMap = new InterpolatingDoubleTreeMap();

  // Set up interpolating tree maps for passing
  private static final InterpolatingTreeMap<Double, Rotation2d> passHoodAngleMap = new InterpolatingTreeMap<>(
      InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
  static {
    // Stored as distance (m), hood angle (radians), flywheel speed (Rads/sec), time of
    // flight (sec)
    double[][] scoringData = {
      { Units.inchesToMeters(55), 0, 146, 1},
      { 1.528, 0.1542, 133.65, 1},
      { 2.067517874264592, 0.226, 134, 1},
      { 2.981151807004248, 0.265, 149, 1},
      { 3.8951452800012, 0.375, 163, 1},
      { 4.809293308638241, 0.384, 185.3, 1},
      };

    // Stored as distance (m), hood angle (radians), flywheel speed (Rads/sec), time of
    // flight (sec)
    //TODO actually find this, just using hub right now
    double[][] passingData = { 
      { Units.inchesToMeters(55), 0, 124, 1},
      { 2.067517874264592, 0.226, 100, 1},
      { 2.981151807004248, 0.265, 115, 1},
      { 3.8951452800012, 0.375, 124, 1},
      { 4.809293308638241, 0.384, 156.3, 1},
      { 5.551, 0.404, 201.3, 1},
      };

    for (double[] i : scoringData) {
      hubHoodAngleMap.put(i[0], Rotation2d.fromRadians(i[1]));
      hubFlywheelSpeedMap.put(i[0], i[2]);
    }

    for (double[] i : passingData) {
      passHoodAngleMap.put(i[0], Rotation2d.fromRadians(i[1]));
      passFlywheelSpeedMap.put(i[0], i[2]);
    }

  }

  /***
   * Gets the parameters required to make a shot into the Hub
   * 
   * @param pose          The current field relative robot position
   * @param robotVelocity The current field relative robot velocity
   * @param doOverRide    Whether to override the controls to prevent the robot
   *                      form illegally shooting
   * @return The parameters required to make a shot into the hub with the current
   *         robot position and velocity
   */
  public static ShootingParams getHubParams(Pose2d pose, ChassisSpeeds robotVelocity) {
    Translation2d target = AllianceFlipUtil.apply(FieldConstants.hubPosition);
    return getParams(pose, robotVelocity, target, true, true);
  }

  /***
   * Gets the shooter parameters in order to pass
   * 
   * @param Pose          The robot's position
   * @param robotVelocity The robot's velocity
   */
  public static ShootingParams getPassParams(Pose2d pose, ChassisSpeeds robotVelocity) {
    boolean isValid = true;
    Translation2d turretPose = pose.getTranslation(); // TODO add translating from robot to turret
    Translation2d targetPose;
    if (GeomUtil.translation2dInBoundingBox(turretPose, AllianceFlipUtil.apply(FieldConstants.allianceZone))) {
      return new ShootingParams(false, new Rotation2d(), 0, 0, 0);
    }
    if (pose.getY() > AllianceFlipUtil.apply(FieldConstants.hubPosition).getY() && !AllianceFlipUtil.shouldFlip() || pose.getY() < AllianceFlipUtil.apply(FieldConstants.hubPosition).getY() && AllianceFlipUtil.shouldFlip()){
      targetPose = AllianceFlipUtil.apply(new Translation2d(FieldConstants.passTarget.getX(), (FieldConstants.fieldWidth - FieldConstants.passTarget.getY())));
    } else {
      targetPose = AllianceFlipUtil.apply(FieldConstants.passTarget);
    }
    Translation2d[] shotPath = {turretPose, targetPose };
    if (GeomUtil.doesLinePassThroughArea(shotPath, AllianceFlipUtil.apply(FieldConstants.passExclusionZone))) { // TODO make sure alliance flip for arrays is working correctly
      isValid = false;
    }
    return getParams(pose, robotVelocity, targetPose, false, isValid);
  }

  /***
   * An internal method used for getting the shot parameters to a pose
   * 
   * @param robotPose        The position of the robot
   * @param robotVelocity    The robot's velocity
   * @param target           The target of the shot
   * @param isValid          If the shot is a valid shot
   */
  private static ShootingParams getParams(Pose2d robotPose, ChassisSpeeds robotVelocity, Translation2d target, boolean isHub, boolean isValid) {
    // Calculate the distance from the turret to the target
    Pose2d turretPosition = robotPose.transformBy(Constants.TurretShooterConstants.ROBOT_TO_TURRET);
    turretPosition.rotateAround(robotPose.getTranslation(), robotPose.getRotation());

    //Calculate turret velocity
    double dx = target.getX() - turretPosition.getX(); // Change in x to target
    double dy = target.getY() - turretPosition.getY(); // Change in y to target

    //Factor in rotational velocity
    double cos = Math.cos(robotPose.getRotation().getRadians());
    double sin = Math.sin(robotPose.getRotation().getRadians());
    double turretVelocityX = robotVelocity.vxMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
            * (Constants.TurretShooterConstants.ROBOT_TO_TURRET.getY() * cos
                - Constants.TurretShooterConstants.ROBOT_TO_TURRET.getX() * sin);
    double turretVelocityY = robotVelocity.vyMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
            * (Constants.TurretShooterConstants.ROBOT_TO_TURRET.getX() * cos
                - Constants.TurretShooterConstants.ROBOT_TO_TURRET.getY() * sin);
    Translation2d robotVelocityVector = new Translation2d(turretVelocityX, turretVelocityY);
    Translation2d aimVector = new Translation2d(dx, dy);
    double dist_to_target = aimVector.getNorm();

    // Do the dot product
    double para_vel = GeomUtil.dot(robotVelocityVector, aimVector) / dist_to_target;
    Translation2d perpVector = new Translation2d(-dy, dx);
    double perp_vel = GeomUtil.dot(robotVelocityVector, perpVector) / dist_to_target;

    // Calculate params
    //turretAngle = lookAheadPose.getTranslation().getAngle();
    //Outputs the angle off by 90 degrees
    SmartDashboard.putNumber("ShotCalc/Range", dist_to_target);
    double angle_adj = perp_vel * -0.86 / (dist_to_target);
    Rotation2d turretAngle = new Rotation2d(Math.atan2(dy, dx)).rotateBy(new Rotation2d(angle_adj));
    double range_adj = (para_vel > 0 ? Constants.TurretShooterConstants.SHOT_CALC_PARA_VEL_GAIN_TOWARDS : Constants.TurretShooterConstants.SHOT_CALC_PARA_VEL_GAIN_AWAY) * (para_vel);
    dist_to_target += range_adj;//* TurretShooterConstants.SHOT_CALC_PARA_VEL_GAIN);
    dist_to_target = Math.max(1.528, dist_to_target); // Do not allow interpolation within the hub space, it doesn't make sense
    SmartDashboard.putNumber("ShotCalc/Range_ADJ", range_adj);
    SmartDashboard.putNumber("ShotCalc/Angle_ADJ", angle_adj);
    SmartDashboard.putNumber("ShotCalc/ParaVel", para_vel);
    SmartDashboard.putNumber("ShotCalc/PerpVel", perp_vel);
    SmartDashboard.putNumber("ShotCalc/dx", dx);
    SmartDashboard.putNumber("ShotCalc/dy", dy);
    SmartDashboard.putNumber("ShotCalc/xVelocity", turretVelocityX);
    SmartDashboard.putNumber("ShotCalc/yVelocity", turretVelocityY);
    turretAngle = new Rotation2d(MathUtil.angleModulus(turretAngle.getRadians()));

    //Calculate turret velocity
    double vx = turretVelocityX;
    double vy = turretVelocityY;
    double turretSpeed = -(dx * vy - dy * vx) / (dx * dx + dy * dy);
    return new ShootingParams(isValid,
        turretAngle,
        (isHub ? hubHoodAngleMap.get(dist_to_target).getRadians() : passHoodAngleMap.get(dist_to_target).getRadians()),
        (isHub ? hubFlywheelSpeedMap.get(dist_to_target) : passFlywheelSpeedMap.get(dist_to_target)), turretSpeed);
  }

}
