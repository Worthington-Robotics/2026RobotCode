package frc.WorBots;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final double ROBOT_PERIOD = 50.0;
  public static final boolean IS_COMP = false;
  public static final boolean getSim(){
    return Robot.isSimulation();
  }

  //Robot Constants
  public static final double ROBOT_LENGTH = 0.0; //m
  public static final double ROBOT_WIDTH = 0.0; //m
  public static final double ROBOT_WHEELBASE = 0.0; //m

  //Drive Constants
  public static final double DRIVE_MULTIPLIER = 1.0;
  public static final double DRIVE_GEAR_RATIO = 5.36;
  public static final double TURN_GEAR_RATIO = 18.75;
  public static final double DRIVE_STOP_XY_THRESHOLD = 0.0; // m/sec
  public static final double DRIVE_THETA_THRESHOLD = 0.0; // rads/sec
  //TODO decide these values
  public static final double DRIVE_MAX_VELOCITY = 5; //m/sec
  public static final double DRIVE_MAX_ACCELERATION = 0.1; //m/sec^2
  public static final double DRIVE_MAX_ROTATIONAL_VELOCITY = Units.degreesToRadians(180); //rads/sec
  public static final double DRIVE_MAX_ROTATION_ACCELERATION = Units.degreesToRadians(0.314); //rad/sec^2
    /**
 * The minimum speed percentage of the maximum that can be set before angle changes are ignored
 */
  public static final double ANTI_JITTER_THRESHOLD = 0.005;

}
