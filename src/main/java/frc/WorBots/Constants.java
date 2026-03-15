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
  //TODO figure out if we have all our GRs inverted
  //General Constants
  public static final boolean IS_COMP = false;
  public static final boolean ENABLE_DEBUG_ROUTINES = true;
  public static final boolean getSim(){
    return Robot.isSimulation();
  }

  /**Robot Constants*/
  public class RobotConstants{
    public static final double ROBOT_LENGTH = Units.inchesToMeters(35.5); //m
    public static final double ROBOT_WIDTH = Units.inchesToMeters(35.7); //m
    public static final double ROBOT_WHEELBASE = Units.inchesToMeters(24); //m
    public static final double ROBOT_MASS = Units.lbsToKilograms(115.8);
    public static final double ROBOT_MOI = 4.7089; //Kg * m^2
    // Robot period in seconds
    public static final double ROBOT_PERIOD = 0.01;
    public static final double ROBOT_FREQUENCY = 1.0 / ROBOT_PERIOD;
  }
  
  /**Drive Constants*/
  public class DriveConstants{
    public static final double DRIVE_MULTIPLIER = 1.0;
    public static final double DRIVE_SLOW_MULTIPLIER = DRIVE_MULTIPLIER * 0.4;
    public static final double DRIVE_CURRENT_LIMIT = 40;
    public static final double DRIVE_GEAR_RATIO = 6.02;
    public static final double TURN_GEAR_RATIO = 287.0 / 11.0;
    public static final Translation2d[] DRIVE_MODULE_OFFSETS = new Translation2d[] {
        new Translation2d(RobotConstants.ROBOT_WHEELBASE / 2, RobotConstants.ROBOT_WHEELBASE / 2),
        new Translation2d(RobotConstants.ROBOT_WHEELBASE / 2, -RobotConstants.ROBOT_WHEELBASE / 2),
        new Translation2d(-RobotConstants.ROBOT_WHEELBASE / 2, RobotConstants.ROBOT_WHEELBASE / 2),
        new Translation2d(-RobotConstants.ROBOT_WHEELBASE / 2, -RobotConstants.ROBOT_WHEELBASE / 2)
      };
    public static final double DRIVE_DRIFT_RATE = 0.0; //rads/sec rotational error from driving
    public static final double DRIVE_STOP_XY_THRESHOLD = Units.inchesToMeters(1.6); // m/sec
    public static final double DRIVE_THETA_THRESHOLD = Units.degreesToRadians(1.0); // rads/sec
    public static final double DRIVE_MAX_VELOCITY = Units.feetToMeters(20); //m/sec
    public static final double DRIVE_MAX_ACCELERATION = Units.feetToMeters(40); //m/sec^2
    public static final double DRIVE_MAX_ROTATIONAL_VELOCITY = Units.degreesToRadians(720); //rads/sec
    public static final double DRIVE_MAX_ROTATION_ACCELERATION = Units.degreesToRadians(3000); //rad/sec^2

    /**
    * The minimum speed percentage of the maximum that can be set before angle changes are ignored
    */
    public static final double ANTI_JITTER_THRESHOLD = 0.005;
  }

  /**Spindexer Constants*/
  public class SpindexerConstants{
    //TODO find all these values
    public static final double SPINDEXER_MAX_TEMP = 80.0; //Celcius
    public static final double SPINDEXER_GEAR_RATIO = 1;
    public static final double SPINDEXER_JKgMETERSSQUARED = 1;
    public static final double SPINDEXER_KS = 1;
    public static final double SPINDEXER_KV = 1;
    public static final double SPINDEXER_STALL_CURRENT = 40;
    public static final double SPINDEXER_STALL_SPEED = Units.degreesToRadians(5);
    public static final double SPINDEXER_CURRENT_LIMIT = 40;
    public static final double KICKER_CURRENT_LIMIT = 40;
    //PID Tolerance
    public static final double SPINDEXER_VEL_TOLERANCE = Units.degreesToRadians(30);
  }

  /**Turret and Shooter Constants*/
    public class TurretShooterConstants{
    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(Units.inchesToMeters(-3.922), Units.inchesToMeters(-3.350), new Rotation2d());
    //TODO add real values
    public static final double SHOOTER_ERROR_THRES = 0.0;
    public static final double SHOOTER_MAX_VOLTS = 0.0;
    public static final double Hood_GEAR_RATIO = 79.6;
    //Calced from rotating 180 degrees
    public static final double TURRET_GEAR_RATIO = 27;
    public static final double TURRET_ABS_GEAR_RATIO = 1 / 0.9702;
    public static final double TURRET_MOI = 494.65;
    public static final double TURRET_LOCK_POSITION = 0.0; //TODO add real value
    public final static double PREFIRE_LIMIT = 1.3; //TODO add documentation
    public final static double TURRET_ABS_ENCODER_TRUE_ZERO = -0.67;
    
    //Current Limits
    public final static double TURRET_CURRENT_LIMIT = 40;
    public final static double FLYWHEEL_CURRENT_LIMIT = 40;
    public final static double HOOD_CURRENT_LIMIT = 40;

    //Turret Saftey Limits
    public static final double TURRET_MIN_ANGLE = Units.degreesToRadians(-180.0);
    public static final double TURRET_MAX_ANGLE = Units.degreesToRadians(180.0);
    public static final double TURRET_MIN_VOLTAGE = -7.0;
    public static final double TURRET_MAX_VOLTAGE = 7.0;

    //PID tolerances
    public final static double TURRET_POSE_TOLERANCE = Units.degreesToRadians(0.5);
    public final static double TURRET_VEL_TOLERANCE = Units.degreesToRadians(5);
    public final static double HOOD_POS_TOLERANCE = Units.degreesToRadians(0.1);
    public final static double FLYWHEEL_VEL_TOLERANCE = Units.degreesToRadians(30);

    //Ready to shoot tolerances
    //TODO set real numbers
    public final static double TURRET_READY_TOLERANCE = Units.degreesToRadians(2);
    public final static double HOOD_READY_TOLERANCE = Units.degreesToRadians(3);
    public final static double FLYWHEEL_READY_VEL_TOLERANCE = 20;

    // Feed forward values
    public final static double HOOD_STATIC_FEEDFORWARD_VOLTAGE = .22;

    // Shot calculator constant
    public final static double SHOT_CALC_PARA_VEL_GAIN_TOWARDS = -0.7;
    public final static double SHOT_CALC_PARA_VEL_GAIN_AWAY = -0.8;
    public final static double SHOT_CALC_PERP_VEL_A_GAIN = 1.1;
    public final static double SHOT_CALC_PERP_VEL_B_GAIN = 2.0;

    //Manual shots
    public final static ShootingParams HUB_SHOT = new ShootingParams(true, new Rotation2d(), 0, 144,0);
    public final static ShootingParams TOWER_SHOT = new ShootingParams(true, new Rotation2d(), 0.26, 150,0);
    public final static ShootingParams LEFT_CORNER_SHOT = new ShootingParams(true, null, 0, 0,0);
    public final static ShootingParams RIGHT_CORNER_SHOT = new ShootingParams(true, null, 0, 0,0);
  }

  /**PathPlanner Constants*/
  public class PathPlannerConstants{
    public static final ModuleConfig PATHPLANNER_MODULE_CONFIG = new ModuleConfig(Units.inchesToMeters(1.87), DriveConstants.DRIVE_MAX_VELOCITY, 1.0, DCMotor.getKrakenX60(1).withReduction(DriveConstants.TURN_GEAR_RATIO), DriveConstants.DRIVE_CURRENT_LIMIT, 1);
    public static final RobotConfig PATHPLANNER_CONFIG = new RobotConfig(RobotConstants.ROBOT_MASS, RobotConstants.ROBOT_MOI, PATHPLANNER_MODULE_CONFIG, DriveConstants.DRIVE_MODULE_OFFSETS);
  }

  /**Intake Constants*/
  public class IntakeConstants{
    public static final double INTAKE_VOLTAGE = 7;
    //These values need to be modified
    public static final double INTAKE_MAX_TEMP = 80.0;
    public static final double TIME_OF_FLIGHT_THRES = 0.255;
    public static final double INTAKE_PIVOT_GR = 6.33;
    public static final double MOMENT_OF_INERTIA = 1;
    public static final double INTAKE_INTAKE_GR = 1;
    
    public static final double INTAKE_EXTENDING_MAX_VEL = 3.5;
    public static final double INTAKE_EXTENDING_MAX_ACEL = 3.5;

    //Current limits
    public static final double INTAKE_CURRENT_LIMIT = 120;
    public static final double EXTENDER_CURRENT_LIMIT = 200;
 
    //Position constants for intake logic
    public static final double EXTENDER_MIN_LIMIT = 0.027;
    public static final double EXTENDER_MAX_LIMIT = Units.degreesToRadians(90);
    public static final double EXTENDER_FRICTION_ZONE = 1.367;
    public static final double EXTENDER_FRICTION_ZONE_KS = 0.5;

    //PID Tolerances
    public static final double EXTENDER_POS_TOLERANCE = Units.degreesToRadians(10);

    //PID / Feedforward Values
    public static final double INTAKE_EXTEND_TOLERANCE = 0.15;
    public static final double INTAKE_EXTEND_EXTEND_POSE_TOLERANCE = 0.4;
    public static final double INTAKE_EXTENDING_KP = 3.0; //3
    public static final double INTAKE_EXTENDING_KI = 1.0; //1
    public static final double INTAKE_EXTENDING_KD = 0.00;
    public static final double EXTENDER_KS = 0.1;
    public static final double EXTENDER_KG = 3.0; 
    public static final double EXTENDER_KV = 0.5;
    
    //New intake controls constants
    public static final double EXTEND_MULT_DOWNWARD = 1.6; //Controls downward force when extending
    public static final double EXTEND_MULT_UPWARD = 1.53; //Controls upward force when extending
    public static final double RETRACT_MULT = 7.0; // Controls upward force when retracting

    public static final double INTAKE_PULSE_INTAKING_SEC = 3.0;
    public static final double INTAKE_PULSE_SPIT_SEC = 0.3;
  }

  public class ClimberConstants{
    public static final double MAX_ROTATIONAL_VELOCITY = 1;
    public static final double MAX_ROTATIONAL_ACCELERATION = 2;
    //TODO invert the motor
    public static final double READY_CLIMBER_POSITION = Units.rotationsToRadians(470);
    public static final double CLIMB_POSITION = Units.rotationsToRadians(931.019);
    public static final double CLIMBER_CURRENT_LIMIT = 40;
    //PID Tolerance
    public static final double CLIMBER_POS_TOLERANCE = Units.degreesToRadians(5);
  }

  /**Trajectory Constants*/
  public class TrajectoryConstants{
    /**
    * How close the robot needs to be to its goal position to stop following the trajectory.
    */
    public static final double MIN_DISTANCE = 0.1; //m

    /**
    * How close the robot needs to be to the end time of the trajectory before it stops following it.
    */
    public static final double MIN_TIME = 0.5; //s
  }
}

