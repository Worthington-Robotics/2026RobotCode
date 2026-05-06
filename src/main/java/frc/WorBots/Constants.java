package frc.WorBots;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.WorBots.subsystems.superstructure.ShotCalculator.ShootingParams;

public class Constants {
  // General Constants
  public static final boolean IS_COMP = true;
  public static final boolean ENABLE_DEBUG_ROUTINES = false;

  public static final boolean getSim() {
    return Robot.isSimulation();
  }

  /** Robot Constants */
  public class RobotConstants {
    public static final double ROBOT_LENGTH = Units.inchesToMeters(35.5); // m
    public static final double ROBOT_WIDTH = Units.inchesToMeters(35.7); // m
    public static final double ROBOT_WHEELBASE = Units.inchesToMeters(24); // m
    public static final double ROBOT_MASS = Units.lbsToKilograms(115.8);
    public static final double ROBOT_MOI = 4.7089; // Kg * m^2

    public static final double ROBOT_FREQUENCY = 50.0;
    public static final double ROBOT_PERIOD = 1.0 / ROBOT_FREQUENCY;

    /** The voltage threshhold the voltage the rio recieves must drop bellow to trigger a brownout */
    public static final double BROWNOUT_THRESHHOLD = 5.5;
  }

  /** Drive Constants */
  public class DriveConstants {
    public static final double DRIVE_MULTIPLIER = 1.0;
    public static final double DRIVE_CURRENT_LIMIT = 40;
    public static final double TURN_CURRENT_LIMIT = 20;
    public static final double DRIVE_GEAR_RATIO = 6.02;
    public static final double TURN_GEAR_RATIO = 287.0 / 11.0;
    public static final Translation2d[] DRIVE_MODULE_OFFSETS = new Translation2d[] {
        new Translation2d(RobotConstants.ROBOT_WHEELBASE / 2, RobotConstants.ROBOT_WHEELBASE / 2),
        new Translation2d(RobotConstants.ROBOT_WHEELBASE / 2, -RobotConstants.ROBOT_WHEELBASE / 2),
        new Translation2d(-RobotConstants.ROBOT_WHEELBASE / 2, RobotConstants.ROBOT_WHEELBASE / 2),
        new Translation2d(-RobotConstants.ROBOT_WHEELBASE / 2, -RobotConstants.ROBOT_WHEELBASE / 2)
    };
    public static final double DRIVE_DRIFT_RATE = 0.0; // rads/sec rotational error from driving
    public static final double DRIVE_STOP_XY_THRESHOLD = Units.inchesToMeters(1.6); // m/sec
    public static final double DRIVE_THETA_THRESHOLD = Units.degreesToRadians(1.0); // rads/sec
    public static final double DRIVE_MAX_VELOCITY = Units.feetToMeters(20); // m/sec
    public static final double DRIVE_MAX_ACCELERATION = Units.feetToMeters(40); // m/sec^2
    public static final double DRIVE_MAX_ROTATIONAL_VELOCITY = Units.degreesToRadians(720); // rads/sec
    public static final double DRIVE_MAX_ROTATION_ACCELERATION = Units.degreesToRadians(3000); // rad/sec^2

    public static final double DRIVE_MAX_ACCELERATION_SHOOTING = DRIVE_MAX_ACCELERATION * 0.4;
    public static final double DRIVE_MAX_ROTATION_ACCELERATION_SHOOTING = 0.5;
    public static final double DRIVE_MAX_ROTATION_VELOCITY_SHOOTING = DRIVE_MAX_ROTATIONAL_VELOCITY * 0.4;
    public static final boolean DO_SHOOTING_ACCEL_LIMIT_IN_AUTO = false;

    /**
     * The minimum speed percentage of the maximum that can be set before angle
     * changes are ignored
     */
    public static final double ANTI_JITTER_THRESHOLD = 0.005;

    public static final double GYRO_LOCK_KP = 1.5;

    public static final double ACCELERATION_FILTER_FACTOR = 0.25; // Should be between 0 and 1
  }

  /** Spindexer Constants */
  public class SpindexerConstants {
    public static final double SPINDEXER_VOLTAGE = 9;

    public static final double SPINDEXER_VELOCITY = 150.0;
    public static final double KICKER_VELOCITY = SPINDEXER_VELOCITY * 4.2;

    public static final double SPINDEXER_MAX_TEMP = 80.0; // Celcius
    public static final double SPINDEXER_GEAR_RATIO = 1 / 6.0;
    public static final double KICKER_GEAR_RATIO = 1 / 3.0;

    public static final double SPINDEXER_JKgMETERSSQUARED = 1;
    public static final double SPINDEXER_STALL_CURRENT = 30;
    public static final double SPINDEXER_STALL_SPEED = Units.degreesToRadians(5);
    public static final double SPINDEXER_CURRENT_LIMIT = 70;
    public static final double KICKER_CURRENT_LIMIT = 80;

    // TODO tune this PID
    public static final double SPINDEXER_KP = 0.0;
    public static final double SPINDEXER_KI = 0.0;
    public static final double SPINDEXER_KD = 0.0;
    public static final double SPINDEXER_MAX_ACCEL = 20.0 * SPINDEXER_VELOCITY;
    public static final double SPINDEXER_MAX_VEL = SPINDEXER_VELOCITY * 1.1;

    public static final double SPINDEXER_VEL_TOLERANCE = 5;
    public static final double KICKER_VEL_TOLERANCE = 5;

    // TODO Tune this pid
    public static final double KICKER_KP = 0.0;
    public static final double KICKER_KI = 0.0;
    public static final double KICKER_KD = 0.0;
    public static final double KICKER_MAX_VEL = KICKER_VELOCITY * 1.1;
    public static final double KICKER_MAX_ACCEL = 20.0 * KICKER_VELOCITY;

    public static final double SPIN_KS = 0.19;
    public static final double SPIN_KV = 0.1075;
    public static final double KICKER_KS = 0.175;
    public static final double KICKER_KV = 0.0565 * 1.175;

    public static final double KICKER_UNJAM_VOLTAGE = 5.0;
    public static final double SPIN_UNJAM_VOLTAGE = 5.0;
  }

  /** Turret and Shooter Constants */
  public class TurretShooterConstants {
    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(Units.inchesToMeters(-3.922),
        Units.inchesToMeters(-3.350), new Rotation2d());
    public static final double Hood_GEAR_RATIO = 79.6;

    // Calced from rotating 180 degrees
    public static final double TURRET_GEAR_RATIO = 27;
    public static final double TURRET_ABS_GEAR_RATIO = 1 / 0.9702;
    public static final double TURRET_MOI = 494.65;
    public final static double PREFIRE_LIMIT = 1.3;
    public final static double TURRET_ABS_ENCODER_TRUE_ZERO = -0.67;

    // Current Limits
    public final static double TURRET_CURRENT_LIMIT = 40;
    public final static double FLYWHEEL_CURRENT_LIMIT = 85;
    public final static double HOOD_CURRENT_LIMIT = 20;

    // Turret Saftey Limits
    public static final double TURRET_MIN_ANGLE = -Units.degreesToRadians(190.0);
    public static final double TURRET_MAX_ANGLE = Units.degreesToRadians(190.0);
    public static final double TURRET_MIN_VOLTAGE = -7.0;
    public static final double TURRET_MAX_VOLTAGE = 7.0;

    // PID tolerances
    public final static double TURRET_POSE_TOLERANCE = Units.degreesToRadians(0.35);
    public final static double TURRET_VEL_TOLERANCE = Units.degreesToRadians(5);
    public final static double HOOD_POS_TOLERANCE = Units.degreesToRadians(0.1);
    public final static double FLYWHEEL_VEL_TOLERANCE = Units.degreesToRadians(0);

    // Ready to shoot tolerances
    public final static double TURRET_READY_TOLERANCE = Units.degreesToRadians(20);
    public final static double HOOD_READY_TOLERANCE = Units.degreesToRadians(2);
    public final static double FLYWHEEL_READY_VEL_TOLERANCE = 20;

    // Ready to pass tolerances
    public final static double TURRET_READY_PASS_TOLERANCE = Units.degreesToRadians(4);
    public final static double HOOD_READY_PASS_TOLERANCE = Units.degreesToRadians(4);
    public final static double FLYWHEEL_READY_PASS_VEL_TOLERANCE = 20;

    // Feed forward values
    public final static double HOOD_STATIC_FEEDFORWARD_VOLTAGE = .22;

    // Shot calculation constants
    public final static double ACCELERATION_FACTOR = 0; // How many seconds of acceleration to apply to velocity in shot
                                                        // calculation. Should probably be about the time we expect
                                                        // systems to take to respond
    public final static double CHANGE_TARGET_MARGIN = 0.5; // How far into an area the robot must be to change its shot
                                                           // target

    // Manual shots
    public final static ShootingParams HUB_SHOT = new ShootingParams(true, new Rotation2d(), 0, 146, 0);
    public final static ShootingParams TOWER_SHOT = new ShootingParams(true, new Rotation2d(), 0.116, 180, 0);
    public final static ShootingParams LEFT_CORNER_SHOT = new ShootingParams(true, new Rotation2d(-0.8026 + 0.00872665),
        0.383, 199.5, 0);
    public final static ShootingParams RIGHT_CORNER_SHOT = new ShootingParams(true, new Rotation2d(0.8026 - 0.00872665),
        0.383, 199.5, 0);
  }

  /** PathPlanner Constants */
  public class PathPlannerConstants {
    public static final ModuleConfig PATHPLANNER_MODULE_CONFIG = new ModuleConfig(Units.inchesToMeters(1.87),
        DriveConstants.DRIVE_MAX_VELOCITY, 1.0, DCMotor.getKrakenX60(1).withReduction(DriveConstants.TURN_GEAR_RATIO),
        DriveConstants.DRIVE_CURRENT_LIMIT, 1);
    public static final RobotConfig PATHPLANNER_CONFIG = new RobotConfig(RobotConstants.ROBOT_MASS,
        RobotConstants.ROBOT_MOI, PATHPLANNER_MODULE_CONFIG, DriveConstants.DRIVE_MODULE_OFFSETS);
  }

  /** Intake Constants */
  public class IntakeConstants {
    public static final double INTAKE_VOLTAGE = 8;
    // These values need to be modified
    public static final double INTAKE_MAX_TEMP = 80.0;
    public static final double TIME_OF_FLIGHT_THRES = 0.255;
    public static final double INTAKE_PIVOT_GR = 6.33 * 3;
    public static final double MOMENT_OF_INERTIA = 1;
    public static final double INTAKE_INTAKE_GR = 1;

    public static final double INTAKE_EXTENDING_MAX_VEL = 7.5;
    public static final double INTAKE_EXTENDING_MAX_ACEL = 10.5;

    public static final double INTAKE_EXTENDING_MAX_VEL_UP = 2.75;
    public static final double INTAKE_EXTENDING_MAX_ACEL_UP = 4.25;

    // Current limits
    public static final double INTAKE_CURRENT_LIMIT = 95;
    public static final double EXTENDER_CURRENT_LIMIT = 100;

    // Position constants for intake logic
    public static final double EXTENDER_MIN_LIMIT = 0.027;
    public static final double EXTENDER_MAX_LIMIT = Units.degreesToRadians(90);

    // PID Tolerances
    public static final double EXTENDER_POS_TOLERANCE = Units.degreesToRadians(10);

    // PID / Feedforward Values
    public static final double INTAKE_EXTEND_TOLERANCE = 0.15;
    public static final double INTAKE_EXTEND_EXTEND_POSE_TOLERANCE = 0.4;
    public static final double INTAKE_EXTENDING_KP = 12; // 3
    public static final double INTAKE_EXTENDING_KI = 0.0; // 1
    public static final double INTAKE_EXTENDING_KD = 0.00;
    public static final double EXTENDER_KS = 0.5;
    public static final double EXTENDER_KG = 0.5;
    public static final double EXTENDER_KV = 0.0;

    public static final double INTAKE_SECONDS_TO_AUTO_AGITATE = 3.7;

    public static final double INTAKE_SPIN_UP_SECONDS = 0.1;
  }


  /** Trajectory Constants */
  public class TrajectoryConstants {
    /**
     * How close the robot needs to be to its goal position to stop following the
     * trajectory.
     */
    public static final double MIN_DISTANCE = 0.1; // m

    /**
     * How close the robot needs to be to the end time of the trajectory before it
     * stops following it.
     */
    public static final double MIN_TIME = 0.5; // s
  }
}
