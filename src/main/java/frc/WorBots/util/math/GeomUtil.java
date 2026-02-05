// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.WorBots.Constants;
import java.util.List;

/**
 * Geometry utilities for working with translations, rotations, transforms, and poses.
 *
 * <p>Modified from team 6328.
 */
public class GeomUtil {
  /** 2pi */
  public static final double PI2 = 2 * Math.PI;

  /**
   * Creates a pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d translationToTransform(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure translating transform
   *
   * @param x The x componenet of the translation
   * @param y The y componenet of the translation
   * @return The resulting transform
   */
  public static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d rotationToTransform(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d poseToTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose2d transformToPose(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Creates a pure translated pose
   *
   * @param translation The translation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d translationToPose(Translation2d translation) {
    return new Pose2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure rotated pose
   *
   * @param rotation The rotation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d rotationToPose(Rotation2d rotation) {
    return new Pose2d(new Translation2d(), rotation);
  }

  /**
   * Multiplies a twist by a scaling factor
   *
   * @param twist The twist to multiply
   * @param factor The scaling factor for the twist components
   * @return The new twist
   */
  public static Twist2d multiplyTwist(Twist2d twist, double factor) {
    return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
  }

  /**
   * Converts a Pose3d to a Transform3d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform3d pose3dToTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose3d transform3dToPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Y). chain
   *
   * @param transform The original translation
   * @return The resulting translation
   */
  public static Translation2d translation3dTo2dXY(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getY());
  }

  /**
   * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Z). chain
   *
   * @param transform The original translation
   * @return The resulting translation
   */
  public static Translation2d translation3dTo2dXZ(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getZ());
  }

  /**
   * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Z). chain
   *
   * @param transform The original translation
   * @return The resulting translation
   */
  public static Translation3d translation2dTo3d(Translation2d translation) {
    return new Translation3d(translation.getX(), translation.getY(), 0.0);
  }

  /**
   * Checks if a Transform2d is within a bounding box of two Transform2d corners.
   *
   * @param transform The original translation
   * @param bounds The two corners of the bounding box to check for collision with
   * @return The resulting translation
   */
  public static boolean translation2dInBoundingBox(
      Translation2d translation, Translation2d[] bounds) {
    if (translation.getX() < bounds[0].getX()) {
      return false;
    }
    if (translation.getX() > bounds[1].getX()) {
      return false;
    }
    if (translation.getY() < bounds[0].getY()) {
      return false;
    }
    if (translation.getY() > bounds[1].getY()) {
      return false;
    }
    return true;
  }

  /**
   * Gets the magnitude of a ChassisSpeeds
   *
   * @param speeds The speeds to get the magnitude of
   * @return The magnitude of the speeds vector. Does not factor in rotational velocity.
   */
  public static double getChassisSpeedsMagnitude(ChassisSpeeds speeds) {
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * Applies a ChassisSpeeds to a Pose2d to get what that pose will be after a period of time
   *
   * @param pose The pose to modify
   * @param speeds The speeds to apply to the pose
   * @param period The period of time to apply over, in seconds
   * @return The modified pose
   */
  public static Pose2d applyChassisSpeeds(Pose2d pose, ChassisSpeeds speeds, double period) {
    return new Pose2d(
        pose.getX() + speeds.vxMetersPerSecond * period,
        pose.getY() + speeds.vyMetersPerSecond * period,
        pose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * period)));
  }

  /**
   * Checks if pose1's position is near pose2's position
   *
   * @param pose1 The first pose
   * @param pose2 The second pose
   * @param threshold The distance threshold
   * @return Whether the distance is <= the threshold
   */
  public static boolean isPose2dNear(Pose2d pose1, Pose2d pose2, double threshold) {
    return isTranslation2dNear(pose1.getTranslation(), pose2.getTranslation(), threshold);
  }

  /**
   * Checks if pose1's position is near pose2's position
   *
   * @param pose1 The first pose
   * @param pose2 The second pose
   * @param threshold The distance threshold
   * @return Whether the distance is <= the threshold
   */
  public static boolean isTranslation2dNear(
      Translation2d pose1, Translation2d pose2, double threshold) {
    return pose1.getDistance(pose2) <= threshold;
  }

  /**
   * Computes the dot product of two vectors
   *
   * @param v1 The first vector
   * @param v2 The second vector
   * @return The dot product
   */
  public static double dot(Translation2d v1, Translation2d v2) {
    return (v1.getX() * v2.getX()) + (v1.getY() * v2.getY());
  }

  /**
   * Discretizes a ChassisSpeeds with an additional correction for drift
   *
   * @param speeds The desired speeds
   * @param driftRate The measured drift rate of the swerve drive, in radians per second
   * @return The discretized and corrected speeds
   */
  public static ChassisSpeeds driftCorrectChassisSpeeds(ChassisSpeeds speeds, double driftRate) {
    final double correctedOmega = speeds.omegaRadiansPerSecond + driftRate;
    final Pose2d futurePose =
        new Pose2d(
            speeds.vxMetersPerSecond * Constants.ROBOT_PERIOD,
            speeds.vyMetersPerSecond * Constants.ROBOT_PERIOD,
            new Rotation2d(correctedOmega * Constants.ROBOT_PERIOD));

    final Twist2d twistForPose = new Pose2d().log(futurePose);

    final ChassisSpeeds corrected =
        new ChassisSpeeds(
            twistForPose.dx / Constants.ROBOT_PERIOD,
            twistForPose.dy / Constants.ROBOT_PERIOD,
            twistForPose.dtheta / Constants.ROBOT_PERIOD);
    return corrected;
  }

  /** Gets the distance between a point and a line segment */
  public static double pointToLineSegmentDistance(
      Translation2d p, Translation2d l1, Translation2d l2) {
    final double lengthSquared = Math.pow(l1.getDistance(l2), 2);
    if (lengthSquared == 0.0) {
      return p.getDistance(l1);
    }
    final double t = Math.max(0, Math.min(1, dot(p.minus(l1), l2.minus(l1)) / lengthSquared));
    final Translation2d projection =
        l1.plus(l2.minus(l1).times(t)); // Projection falls on the segment
    return p.getDistance(projection);
  }

  /**
   * Gets the unit vector of a translation
   *
   * @param vector The translation to get the unit of
   * @return The unit vector, with a magnitude of 1.0
   */
  public static Translation2d getUnit(Translation2d vector) {
    return vector.div(vector.getNorm());
  }

  /**
   * Gets the angle to make the robot face some position on the field
   *
   * @param robot The robot position
   * @param goal The goal position
   * @return The field-relative angle to face at to make the robot face the goal
   */
  public static Rotation2d aim(Translation2d robot, Translation2d goal) {
    final double angle = Math.atan2(robot.getY() - goal.getY(), robot.getX() - goal.getX());
    return new Rotation2d(angle);
  }

  /**
   * Translates a vector by a distance away from another vector
   *
   * @param vector The vector to translate
   * @param repellent The vector to move away from
   * @param distance How far to move from the repellent
   * @return The translated vector. If both vectors are at the same position, returns the same
   *     vector.
   */
  public static Translation2d moveAwayFrom(
      Translation2d vector, Translation2d repellent, double distance) {
    if (vector == repellent) {
      return vector;
    }
    final Translation2d normVector = getUnit(vector.minus(repellent));
    return vector.plus(normVector.times(distance));
  }

  /**
   * Translates a vector by a distance away from and to the side of another vector
   *
   * @param vector The vector to translate
   * @param repellent The vector to move away from
   * @param perpendicularDistance How far to move to away from the repellent
   * @param parallelDistance How far to move to the side of the repellent
   * @return The translated vector. If both vectors are at the same position, returns the same
   *     vector.
   */
  public static Translation2d moveAwayFrom(
      Translation2d vector,
      Translation2d repellent,
      double perpendicularDistance,
      double parallelDistance) {
    if (vector == repellent) {
      return vector;
    }
    final Translation2d perpVector = getUnit(vector.minus(repellent));
    final Translation2d parallelVector = perpVector.rotateBy(Rotation2d.fromDegrees(90.0));
    return vector
        .plus(perpVector.times(perpendicularDistance))
        .plus(parallelVector.times(parallelDistance));
  }

  public static Pose2d fastExp(Pose2d pose, Twist2d twist) {
    final double dx = twist.dx;
    final double dy = twist.dy;
    final double dtheta = twist.dtheta;

    double s;
    double c;
    if (Math.abs(dtheta) < 1E-9) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
      c = 0.5 * dtheta;
    } else {

      final double sinTheta = Math.sin(dtheta);
      final double cosTheta = Math.cos(dtheta);
      s = sinTheta / dtheta;
      c = (1.0 - cosTheta) / dtheta;
    }

    final double transformX = dx * s - dy * c;
    final double transformY = dx * c + dy * s;
    final Rotation2d poseRotation = pose.getRotation();
    final double translationX =
        transformX * poseRotation.getCos() - transformY * poseRotation.getSin();
    final double translationY =
        transformX * poseRotation.getSin() + transformY * poseRotation.getCos();

    final Translation2d newTranslation =
        new Translation2d(
            pose.getTranslation().getX() + translationX,
            pose.getTranslation().getY() + translationY);

    // WPILib doesn't have a method to create a rotation from all three components
    // so we just have to choose the faster of two evils
    final Rotation2d newRotation = new Rotation2d(dtheta + pose.getRotation().getRadians());

    return new Pose2d(newTranslation, newRotation);
  }

  /**
   * Returns the average pose from multiple Pose3ds
   *
   * @param poses The poses
   * @return The average pose
   */
  public static Pose3d averagePose3ds(List<Pose3d> poses) {
    double xSum = 0.0, ySum = 0.0, zSum = 0.0;
    double rxSum = 0.0, rySum = 0.0, rzSum = 0.0;
    for (Pose3d pose : poses) {
      xSum += pose.getX();
      ySum += pose.getY();
      zSum += pose.getZ();
      rxSum += pose.getRotation().getX();
      rySum += pose.getRotation().getY();
      rzSum += pose.getRotation().getZ();
    }

    return new Pose3d(
        xSum / poses.size(),
        ySum / poses.size(),
        zSum / poses.size(),
        new Rotation3d(rxSum / poses.size(), rySum / poses.size(), rzSum / poses.size()));
  }

  /**
   * Gets the distance between two points along some arbitrary axis by projecting them onto the
   * coordinate system of that axis
   *
   * @param axis The angle of the axis from the horizontal
   * @param p1 The first point
   * @param p2 The second point
   * @return The difference between the two points along the axis
   */
  public static double distanceAlongAxis(Rotation2d axis, Translation2d p1, Translation2d p2) {
    final double dot1 = p1.getX() * axis.getCos() + p1.getY() * axis.getSin();
    final double dot2 = p2.getX() * axis.getCos() + p2.getY() * axis.getSin();

    return dot1 - dot2;
  }

  /**
   * Gets the distance between the robot and a target pose perpendicular to the angle of the target
   *
   * @param robot The robot pose
   * @param target The target pose
   * @return The difference between the two points along the perpendicular axis
   */
  public static double perpendicularDistance(Pose2d robot, Pose2d target) {
    return distanceAlongAxis(
        target.getRotation().plus(Rotation2d.kCCW_90deg),
        robot.getTranslation(),
        target.getTranslation());
  }

  public static Translation2d projectVector(Translation2d source, Translation2d target) {
    final double norm = target.getNorm();
    final double x = dot(source, target) / norm;
    final double y = dot(source, target.rotateBy(Rotation2d.kCW_90deg)) / norm;
    return new Translation2d(x, y);
  }

  /*** Checks if three points are in counterclockwise order. Used for determining if lines intersect. */
  private static boolean ccw(Translation2d A, Translation2d B, Translation2d C){
    return (C.getY()-A.getY())*(B.getX()-A.getX()) > (B.getY()-A.getY())*(C.getX()-A.getX());
  }
  
  /*** 
   * Checks if two lines intersect
   * @param line1 The two endpoints of the first
   * @param line2 The two endpoints of the second line
   * @return Whether the two lines intersect
   * @implNote Does not support collinearity 
   */
  public static boolean doLinesIntersect(Translation2d[] line1, Translation2d[] line2 ){
    return (ccw(line1[0],line2[0],line2[1]) != ccw(line1[1],line2[0],line2[1])) && (ccw(line1[0],line1[1],line2[0]) != ccw(line1[0],line1[1],line2[1]));
  }
  
  /*** 
   * Checks if a line segment passes through an area.
   * @param line A two item Translation2d array containing the two endpoints of the line segment to test.
   * @param bounds A two item Translation2d array containing the bounding points of the area.
   * @return Whether the line passes through the area.
   */
  public static boolean doesLinePassThroughArea(Translation2d[] line, Translation2d[] bounds){
    Translation2d[] testLine1 = {new Translation2d(bounds[0].getX(), bounds[0].getY()), new Translation2d(bounds[1].getX(), bounds[1].getY())};
    Translation2d[] testLine2 = {new Translation2d(bounds[1].getX(), bounds[0].getY()), new Translation2d(bounds[0].getX(), bounds[1].getY())}; 
    return doLinesIntersect(line, testLine1) ||
    doLinesIntersect(line, testLine2) ||
    translation2dInBoundingBox(line[0], bounds) ||
    translation2dInBoundingBox(line[1], bounds);
  }
}
