package frc.WorBots;

import edu.wpi.first.math.util.Units;

public class Constants {
  // Robot period in seconds
  public static final double ROBOT_PERIOD = 0.01;
  public static final double ROBOT_FREQUENCY = 1.0 / ROBOT_PERIOD;
  public static final boolean IS_COMP = false;
  public static final boolean getSim(){
    return Robot.isSimulation();
  }

  //Robot Constants
  public static final double ROBOT_LENGTH = Units.inchesToMeters(27); //m
  public static final double ROBOT_WIDTH = Units.inchesToMeters(27); //m
  public static final double ROBOT_WHEELBASE = Units.inchesToMeters(24); //m

  //Drive Constants
  public static final double DRIVE_MULTIPLIER = 1.0;
  public static final double DRIVE_GEAR_RATIO = 6.02;
  public static final double TURN_GEAR_RATIO = 287.0 / 11.0;
  public static final double DRIVE_DRIFT_RATE = 0.0; //rads/sec rotational error from driving
  public static final double DRIVE_STOP_XY_THRESHOLD = Units.inchesToMeters(1.6); // m/sec
  public static final double DRIVE_THETA_THRESHOLD = Units.degreesToRadians(1.0); // rads/sec
  public static final double DRIVE_MAX_VELOCITY = Units.feetToMeters(25.5); //m/sec
  public static final double DRIVE_MAX_ACCELERATION = Units.feetToMeters(40); //m/sec^2
  public static final double DRIVE_MAX_ROTATIONAL_VELOCITY = Units.degreesToRadians(720); //rads/sec
  public static final double DRIVE_MAX_ROTATION_ACCELERATION = Units.degreesToRadians(3000); //rad/sec^2

/**
 * The minimum speed percentage of the maximum that can be set before angle changes are ignored
 */
  public static final double ANTI_JITTER_THRESHOLD = 0.005;

  //Trajectory Constants
  /**
   * How close the robot needs to be to its goal position to stop following the trajectory.
   */
  public static final double MIN_DISTANCE = 0.1; //m
  /**
   * How close the robot needs to be to the end time of the trajectory before it stops following it.
   */
  public static final double MIN_TIME = 0.5; //s

}
