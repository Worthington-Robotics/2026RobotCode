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

/**
 * A tool for calculating the required parameters of the robots next shot
 */
public class ShotCalculator2 {
  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;

  private static Rotation2d turretAngle;
  private static Rotation2d hoodAngle;
  private static double flywheelspeed;
  private static boolean isValid;

    // Set up interpolating tree tables
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = new InterpolatingTreeMap<>(
      InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    minDistance = 0.0; // TODO set this //Thee minimum distance the robot can shoot
    maxDistance = 10.0; // TODO set this //The maximum distance the robot can shoot
    phaseDelay = 0.03; // TODO set this

    shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    shotFlywheelSpeedMap.put(1.34, 210.0);
    shotFlywheelSpeedMap.put(1.78, 220.0);
    shotFlywheelSpeedMap.put(2.17, 220.0);
    shotFlywheelSpeedMap.put(2.81, 230.0);
    shotFlywheelSpeedMap.put(3.82, 250.0);
    shotFlywheelSpeedMap.put(4.09, 255.0);
    shotFlywheelSpeedMap.put(4.40, 260.0);
    shotFlywheelSpeedMap.put(4.77, 265.0);
    shotFlywheelSpeedMap.put(5.57, 275.0);
    shotFlywheelSpeedMap.put(5.60, 290.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public static void calculateShot(Pose2d robotPose, ChassisSpeeds speeds){
    //Finds the turret pose
    Pose2d turretPose = robotPose.transformBy(Constants.TurretShooterConstants.ROBOT_TO_TURRET);
    double robotAngle = robotPose.getRotation().getRadians();
    turretPose.rotateAround(robotPose.getTranslation(), robotPose.getRotation());

    //Calculates the turrets field relative velocity
    double turretVelocityX = speeds.vxMetersPerSecond
        + speeds.omegaRadiansPerSecond
            * (Constants.TurretShooterConstants.ROBOT_TO_TURRET.getY() * Math.cos(robotAngle)
                - Constants.TurretShooterConstants.ROBOT_TO_TURRET.getX() * Math.sin(robotAngle));
    double turretVelocityY = speeds.vyMetersPerSecond
        + speeds.omegaRadiansPerSecond
            * (Constants.TurretShooterConstants.ROBOT_TO_TURRET.getX() * Math.cos(robotAngle)
                - Constants.TurretShooterConstants.ROBOT_TO_TURRET.getY() * Math.sin(robotAngle));    

    //Calculates the hubs position relative to the turret
    Translation2d hubPose = AllianceFlipUtil.apply(FieldConstants.hubPosition);

    //distance to hub
    double hubDistance = Math.hypot(hubPose.getX(), hubPose.getY());

    //Checks if its a valid shot
    //TODO add a check to see if we're in a valid part of the field
    if(minDistance < hubDistance && hubDistance < maxDistance){
      isValid = true;
    }
    else{
      isValid = false;
    }

    double airTime = timeOfFlightMap.get(hubDistance);
    double xError = airTime * turretVelocityX;
    double yError = airTime * turretVelocityY;

    //Adjusts the hub pose by the expected x and y error
    Translation2d targetPose = new Translation2d(hubPose.getX() + xError, hubPose.getY() + yError);

    //Calculates the ajusted target distance and angle
    double targetDistance = Math.hypot(targetPose.getX(), targetPose.getY());
    turretAngle = new Rotation2d(Math.atan2(hubPose.getY(), hubPose.getX()));

    hoodAngle = shotHoodAngleMap.get(targetDistance);
    flywheelspeed = shotFlywheelSpeedMap.get(targetDistance);
  }

  public Rotation2d getTurretAngle(){
    return turretAngle;
  }

  public Rotation2d getHoodAngle(){
    return hoodAngle;
  }

  public double getFlywheelSpeed(){
    return flywheelspeed;
  }

  public boolean isValid(){
    return isValid;
  }
}