// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.control;

import edu.wpi.first.math.geometry.Translation2d;
import frc.WorBots.util.math.GeomUtil;

/* A controller that provides effort to avoid some obstacle on a 2D plane */
public interface Obstacle {
  /*
   * Calculate the control effort needed depending on the position of the robot.
   * Will output between zero and one, and should be multiplied by some gain to
   * produce the speed you want
   */
  public Translation2d calculate(Translation2d pose);

  /* An obstacle that is a single point */
  public static class PointObstacle implements Obstacle {
    private final Translation2d position;
    private final double radius;
    private final double softness;

    /**
     * Create a new point obstacle with the given radius and softness
     *
     * @param position The position of the point
     * @param radius The radius of the point in meters
     * @param softness The softness of the point, which creates a linear falloff. Goes from 0 to 1,
     *     with 0 being no softness
     */
    public PointObstacle(Translation2d position, double radius, double softness) {
      this.position = position;
      this.radius = radius;
      this.softness = softness;
    }

    /**
     * Create a new point obstacle with the given radius and almost no softness
     *
     * @param position The position of the point
     * @param radius The radius of the point in meters
     */
    public PointObstacle(Translation2d position, double radius) {
      this(position, radius, 0.5);
    }

    public Translation2d calculate(Translation2d pose) {
      final double distance = this.position.getDistance(pose);

      final double power = calculatePower(distance, radius, softness);

      // Get the unit vector of the difference between the point and robot to push
      // away
      final Translation2d pushVector = GeomUtil.getUnit(pose.minus(this.position));
      return pushVector.times(power);
    }
  }

  public static class PlaneObstacle implements Obstacle {
    private final boolean isY;
    private final double position;
    private final double sign;
    private final double radius;
    private final double softness;

    /**
     * Create a new plane obstacle
     *
     * @param isY Whether or not to use the Y-axis
     * @param sign The sign for the push, with -1 pushing in the negative
     * @param position The position of the plane direction and 1 pushing positive
     * @param radius The distance for the pushing from the plane
     * @param softness The softness of the plane, which creates a linear falloff. Goes from 0 to 1,
     *     with 0 being no softness
     */
    public PlaneObstacle(
        boolean isY, double sign, double position, double radius, double softness) {
      this.isY = isY;
      this.sign = sign;
      this.position = position;
      this.radius = radius;
      this.softness = softness;
    }

    public Translation2d calculate(Translation2d pose) {
      final double robotPosition = isY ? pose.getY() : pose.getX();
      double distance;
      if (sign == -1) {
        distance = Math.max(0.0, position - robotPosition);
      } else {
        distance = Math.max(0.0, robotPosition - position);
      }

      final double power = calculatePower(distance, radius, softness);

      final Translation2d pushVector;
      if (isY) {
        pushVector = new Translation2d(0.0, power * sign);
      } else {
        pushVector = new Translation2d(power * sign, 0.0);
      }

      return pushVector;
    }
  }

  /**
   * Calculates the control output of multiple obstacles
   *
   * @param obstacles The obstacles to avoid
   * @param pose The pose of the robot
   * @param gain The gain of all the obstacles, which their control effort is multiplied by
   * @return The summed control effort
   */
  public static Translation2d calculateMultiple(
      Obstacle[] obstacles, Translation2d pose, double gain) {
    Translation2d out = new Translation2d();

    for (int i = 0; i < obstacles.length; i++) {
      out = out.plus(obstacles[i].calculate(pose));
    }

    return out.times(gain);
  }

  /**
   * Calculates the pushing power for one of the obstacles
   *
   * @param distance The distance to whatever axis the obstacle is measuring. Must be greater than
   *     0.
   * @param radius The radius of the obstacle (how far out the pushing should apply from 0).
   * @param softness The softness from 0 to 1 of the push, with 0 being no softness
   * @return The pushing power from 0 to 1
   */
  private static double calculatePower(double distance, double radius, double softness) {
    // Calculate the pushing power based on the radius and softness using a
    // piecewise function

    double power = 0.0;
    // The point where we change between constant 1.0 and the softness slope
    final double inflectionPoint = radius * (1.0 - softness);
    if (distance < inflectionPoint) {
      power = 1.0;
    } else {
      double slope = 0.0;
      if (softness != 0.0 && radius != 0.0) {
        slope = -1.0 / (radius * softness);

        // Basically just point-slope form as y = m(x - x_1) + y_1 to make it continuous
        power = slope * (distance - inflectionPoint) + 1.0;

        if (power < 0.0) {
          power = 0.0;
        }
      }
    }

    return power;
  }
}
