package frc.WorBots;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
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
  public static final double ROBOT_MASS = Units.lbsToKilograms(115);
  public static final double ROBOT_MOI = 5;
  
  //Drive Constants
  public static final double DRIVE_MULTIPLIER = 1;
  public static final double DRIVE_CURRENT_LIMIT = 40;
  public static final double DRIVE_GEAR_RATIO = 6.02;
  public static final double TURN_GEAR_RATIO = 287.0 / 11.0;
  public static final Translation2d[] DRIVE_MODULE_OFFSETS = new Translation2d[] {
      new Translation2d(Constants.ROBOT_WHEELBASE / 2, Constants.ROBOT_WHEELBASE / 2),
      new Translation2d(Constants.ROBOT_WHEELBASE / 2, -Constants.ROBOT_WHEELBASE / 2),
      new Translation2d(-Constants.ROBOT_WHEELBASE / 2, Constants.ROBOT_WHEELBASE / 2),
      new Translation2d(-Constants.ROBOT_WHEELBASE / 2, -Constants.ROBOT_WHEELBASE / 2)
    };
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

  //PathPlanner Constants
  public static final ModuleConfig PATHPLANNER_MODULE_CONFIG = new ModuleConfig(Units.inchesToMeters(2), DRIVE_MAX_VELOCITY, ROBOT_WHEELBASE, DCMotor.getKrakenX60(1).withReduction(TURN_GEAR_RATIO), DRIVE_CURRENT_LIMIT, 1);
  public static final RobotConfig PATHPLANNER_CONFIG = new RobotConfig(ROBOT_MASS, ROBOT_MOI, PATHPLANNER_MODULE_CONFIG, DRIVE_MODULE_OFFSETS);
}

