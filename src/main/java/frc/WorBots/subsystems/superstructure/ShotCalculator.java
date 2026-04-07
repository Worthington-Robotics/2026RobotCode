package frc.WorBots.subsystems.superstructure;

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
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.GeomUtil;

//Based on 6328's 2026 shot calculator

/***
 * A class that can be used to calclate the required parameters for the robot's
 * next shot
 */
public class ShotCalculator {
  // Variables to alter functionality
  private final static boolean REMOVE_TRIG = false;
  private final static int ITERATIONS = 10;
  private final static boolean USE_TOF_TABLES = true;
  private final static double MANUAL_TOF_FACTOR = 1.0;

  private final static double UNIVERSAL_FLYWHEEL_MOD = -0.5;
  
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

  private static double minScoreDistance;
  private static double maxScoreDistance;
  private static double minPassDistance;
  private static double maxPassDistance;
  private static double phaseDelay;
  private static double thetaExtraDelay = 0.1;
  // Set up interpolating tree maps for scoring
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = new InterpolatingTreeMap<>(
      InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  // Set up interpolating tree maps for passing
  private static final InterpolatingTreeMap<Double, Rotation2d> passHoodAngleMap = new InterpolatingTreeMap<>(
      InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passTimeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    // Stored as distance (m), hood angle (radians), flywheel speed (m/sec), time of
    // flight (sec)
    double[][] scoringData = {
      // { Units.inchesToMeters(55), 0, 146, 1},
      { 1.341, 0.0, 167.65 + UNIVERSAL_FLYWHEEL_MOD, 1.08}, //new TIF 
      {1.64, 0.020, 154.3 + UNIVERSAL_FLYWHEEL_MOD, 1.05}, //Newer
      {1.828, 0.0355, 158 + UNIVERSAL_FLYWHEEL_MOD, 1.10},
      { 2.046, 0.0534, 167.95 + UNIVERSAL_FLYWHEEL_MOD, 1.231}, //New TIF
      { 2.371, 0.0534, 168 + UNIVERSAL_FLYWHEEL_MOD, 1.23}, //Newer TIF 
      { 2.62, 0.0932, 178.3 + UNIVERSAL_FLYWHEEL_MOD, 1.25}, //Newer
      { 2.972, 0.114028, 179 + UNIVERSAL_FLYWHEEL_MOD, 1.23}, //New TIF
      { 3.29, 0.1227, 184.7 + UNIVERSAL_FLYWHEEL_MOD, 1.29}, //Newer
      { 3.506, 0.1533, 191.9 + UNIVERSAL_FLYWHEEL_MOD, 1.30},//New TIF
      {3.730, 0.163, 194.5 + UNIVERSAL_FLYWHEEL_MOD, 1.32}, 
      { 3.989, 0.185881, 197.558 + UNIVERSAL_FLYWHEEL_MOD, 1.30}, //Newer TIF  
      {4.237, 0.190, 201 + UNIVERSAL_FLYWHEEL_MOD, 1.33},
      {4.24, 0.210, 209 + UNIVERSAL_FLYWHEEL_MOD, 1.34},
      {4.4464, 0.194, 208 + UNIVERSAL_FLYWHEEL_MOD, 1.35}, //Newer TIF
      { 4.595, 0.2102, 207.158 + UNIVERSAL_FLYWHEEL_MOD, 1.374}, //new TIF
      {4.795, 0.230, 210.7 + UNIVERSAL_FLYWHEEL_MOD, 1.4},
      { 5.074, 0.23784, 221.2 + UNIVERSAL_FLYWHEEL_MOD, 1.36}, //New TIF
      { 5.475, 0.233, 232 + UNIVERSAL_FLYWHEEL_MOD, 1.38}, //New
      { 5.770, 0.238, 240 + UNIVERSAL_FLYWHEEL_MOD, 1.66 }, //New TIF
      { 6.258, 0.248, 252 + UNIVERSAL_FLYWHEEL_MOD, 1.72 }
      };

    // Stored as distance (m), hood angle (radians), flywheel speed (m/sec), time of
    // flight (sec)
    double[][] passingData = { 
      { Units.inchesToMeters(55), 0, 112, 1},
      { 2.067517874264592, 0.35, 80, 1},
      { 2.981151807004248, 0.45, 90, 1},
      { 3.8951452800012, 0.5, 105, 1},
      { 4.809293308638241, 0.5, 140, 1},
      { 5.551, 0.5, 160, 1},
      { 10.787, 0.500, 255, 1 },
      { 14.5, 0.500, 290, 1 },
      };
    minScoreDistance = 1.0; 
    maxScoreDistance = 6.258; 
    minPassDistance = 1.0;
    maxPassDistance = 10.5;
    phaseDelay = 0.13; 

    for (double[] i : scoringData) {
      shotHoodAngleMap.put(i[0], Rotation2d.fromRadians(i[1]));
      shotFlywheelSpeedMap.put(i[0], i[2]);
      timeOfFlightMap.put(i[0], i[3]);
    }

    for (double[] i : passingData) {
      passHoodAngleMap.put(i[0], Rotation2d.fromRadians(i[1]));
      passFlywheelSpeedMap.put(i[0], i[2]);
      passTimeOfFlightMap.put(i[0], i[3]);
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
  public static ShootingParams getHubParams(Pose2d pose, ChassisSpeeds robotVelocity, ChassisSpeeds robotAcceleration) {
    Translation2d target = AllianceFlipUtil.apply(FieldConstants.hubPosition);
    return getParams(pose, robotVelocity, robotAcceleration, target, shotHoodAngleMap, shotFlywheelSpeedMap, timeOfFlightMap, true,
        minScoreDistance, maxScoreDistance);
  }

  /***
   * Gets the shooter parameters in order to pass
   * 
   * @param Pose          The robot's position
   * @param robotVelocity The robot's velocity
   */
  public static ShootingParams getPassParams(Pose2d pose, ChassisSpeeds robotVelocity, ChassisSpeeds robotAcceleration) {
    boolean isValid = true;
    Translation2d turretPose = pose.transformBy(Constants.TurretShooterConstants.ROBOT_TO_TURRET).getTranslation();
    Translation2d targetPose;
    if (GeomUtil.translation2dInBoundingBox(turretPose, AllianceFlipUtil.apply(FieldConstants.allianceZone))) {
      return new ShootingParams(false, new Rotation2d(), 0, 0,0);
    }
    if (pose.getY() > AllianceFlipUtil.apply(FieldConstants.hubPosition).getY() && !AllianceFlipUtil.shouldFlip()
        || pose.getY() < AllianceFlipUtil.apply(FieldConstants.hubPosition).getY() && AllianceFlipUtil.shouldFlip()) {
      targetPose = AllianceFlipUtil.apply(new Translation2d(FieldConstants.passTarget.getX(),
          (FieldConstants.fieldWidth - FieldConstants.passTarget.getY())));
    } else {
      targetPose = AllianceFlipUtil.apply(FieldConstants.passTarget);
    }
    Translation2d[] shotPath = { turretPose, targetPose };
    if (GeomUtil.doesLinePassThroughArea(shotPath, AllianceFlipUtil.apply(FieldConstants.passExclusionZone))) { // TODO
                                                                                                                // make
                                                                                                                // sure
                                                                                                                // alliance
                                                                                                                // flip
                                                                                                                // for
                                                                                                                // arrays
                                                                                                                // is
                                                                                                                // working
                                                                                                                // correctly
      isValid = false;
    }
    return getParams(pose, robotVelocity, robotAcceleration, targetPose, passHoodAngleMap, passFlywheelSpeedMap, passTimeOfFlightMap,
        isValid, minPassDistance, maxPassDistance);
  }

  /***
   * An internal method used for getting the shot parameters to a pose
   * 
   * @param robotPose        The position of the robot
   * @param robotVelocity    The robot's velocity
   * @param target           The target of the shot
   * @param hoodAngleMap     The interpolating tree map to use for calculating
   *                         shooter angle
   * @param flywheelSpeedMap The interpolating tree map to use for calculating the
   *                         flywheel speed
   * @param timeOfFlightMap  The interpolating tree map to use for calculating the
   *                         time of flight of a shot
   * @param isValid          If the shot is a valid shot
   */
  private static ShootingParams getParams(Pose2d robotPose, ChassisSpeeds robotVelocity, ChassisSpeeds robotAcceleration, Translation2d target,
      InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap, InterpolatingDoubleTreeMap flywheelSpeedMap,
      InterpolatingDoubleTreeMap timeOfFlightMap, boolean isValid, double minDistance, double maxDistance) {
    // Calculate the estimated robot pose when this method is done running
    Pose2d estimatedPose = robotPose
        .exp(ChassisSpeeds.fromFieldRelativeSpeeds(robotVelocity, robotPose.getRotation()).toTwist2d(phaseDelay));
    // Calculate the distance from the turret to the target
    Pose2d turretPosition = estimatedPose.transformBy(Constants.TurretShooterConstants.ROBOT_TO_TURRET);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
    SmartDashboard.putNumber("ShotCalc/Range", turretToTargetDistance);

    // Calculate field relative turret velocity
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX;
    double turretVelocityY;
    if (REMOVE_TRIG) {
      turretVelocityX = robotVelocity.vxMetersPerSecond;
      turretVelocityY = robotVelocity.vyMetersPerSecond;
    } else {
      double cos = Math.cos(robotAngle);
      double sin = Math.sin(robotAngle);
      turretVelocityX = robotVelocity.vxMetersPerSecond
          + robotVelocity.omegaRadiansPerSecond
              * (Constants.TurretShooterConstants.ROBOT_TO_TURRET.getY() * cos
                  - Constants.TurretShooterConstants.ROBOT_TO_TURRET.getX() * sin) +
                    robotAcceleration.vxMetersPerSecond * Constants.TurretShooterConstants.ACCELERATION_FACTOR;
      turretVelocityY = robotVelocity.vyMetersPerSecond
          + robotVelocity.omegaRadiansPerSecond
              * (Constants.TurretShooterConstants.ROBOT_TO_TURRET.getX() * cos
                  - Constants.TurretShooterConstants.ROBOT_TO_TURRET.getY() * sin)+
                    robotAcceleration.vyMetersPerSecond * Constants.TurretShooterConstants.ACCELERATION_FACTOR;
    }
    SmartDashboard.putNumber("ShotCalc/Velocity X", turretVelocityX);
    SmartDashboard.putNumber("ShotCalc/Velocity Y", turretVelocityY);
    double[] timeOfFlightsLog = {0,0,0,0,0,0,0,0,0,0};

    // Account for imparted velocity by robot to offset
    double timeOfFlight;
    Pose2d lookAheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < ITERATIONS; i++) {
      if (USE_TOF_TABLES) {
        timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      } else {
        timeOfFlight = lookaheadTurretToTargetDistance * MANUAL_TOF_FACTOR;
      }
      timeOfFlightsLog[i] = timeOfFlight;
      double offsetX = turretVelocityX * timeOfFlight;// + robotAcceleration.vxMetersPerSecond * 1 / 2 * timeOfFlight * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;// + robotAcceleration.vyMetersPerSecond * 1 / 2 * timeOfFlight * timeOfFlight;
      lookAheadPose = new Pose2d(
          turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
          turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookAheadPose.getTranslation());
    }

    double turretSpeed = robotVelocity.omegaRadiansPerSecond;
    // if (distSq > 1e-6) {
    //   turretSpeed += -(dx * turretVelocityY - dy * turretVelocityX) / distSq;
    // }


    // Calculate params
    Rotation2d turretAngle = target.minus(lookAheadPose.getTranslation()).getAngle().minus(new Rotation2d(robotVelocity.omegaRadiansPerSecond * thetaExtraDelay));
    double hoodAngle = hoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    double[] turretPose = {robotPose.getX(), robotPose.getY(), turretAngle.getRadians()};
    SmartDashboard.putNumberArray("ShotCalc/Turret angle pose", turretPose);
    SmartDashboard.putNumber("ShotCalc/Turret Angle", turretAngle.getRadians());
    SmartDashboard.putNumberArray("ShotCalc/Time Of Flights", timeOfFlightsLog);
    SmartDashboard.putNumber("ShotCalc/Calculated Range", lookaheadTurretToTargetDistance);
    return new ShootingParams(lookaheadTurretToTargetDistance >= minDistance
        && lookaheadTurretToTargetDistance <= maxDistance && isValid,
        turretAngle,
        hoodAngle,
        flywheelSpeedMap.get(lookaheadTurretToTargetDistance),
        turretSpeed);
  }

}
