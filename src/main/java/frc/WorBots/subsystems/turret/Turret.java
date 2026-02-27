package frc.WorBots.subsystems.turret;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.shooter.ShotCalculator.ShootingParams;
import frc.WorBots.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase{
  //TODO figure out how to implement the right time to wrap around 

  private final NetworkTable turret = NetworkTableInstance.getDefault().getTable("Turret");
  private final BooleanPublisher absConnectedPub =
     turret.getBooleanTopic("AbsConnected").publish();
  private final DoublePublisher absAnglePub =
     turret.getDoubleTopic("AbsAngle").publish();
  private final DoublePublisher relAnglePub =
     turret.getDoubleTopic("RelAngle").publish();
  private final DoublePublisher fusedAnglePub =
     turret.getDoubleTopic("FusedAngle").publish();
  private final StringPublisher controlModePub =
     turret.getStringTopic("ControlMode").publish();
  private final BooleanPublisher atsetpointPub =
     turret.getBooleanTopic("AtSetPoint").publish();
  private final DoublePublisher debugVoltagePub =
     turret.getDoubleTopic("DebugVoltage").publish();
  private final DoublePublisher positionPub =
     turret.getDoubleTopic("Position").publish();
  private final DoublePublisher goalPosePub = 
      turret.getDoubleTopic("Goal Position").publish();

  public final TurretIO io;

  private TurretIOInputs inputs = new TurretIOInputs();
  private turretControlMode controlMode = turretControlMode.Disabled;
  
  private final SimpleMotorFeedforward turretFeedForward = new SimpleMotorFeedforward(0.0, 4.0);
  private  ProfiledPIDController turretFeedBack = new ProfiledPIDController(10, 0, 0,
    new TrapezoidProfile.Constraints(Math.PI , 2 * Math.PI));

  public enum turretControlMode {
    Disabled,
    Position,
    Voltage;
  }

  private double debugVoltage = 0;
  private double goalPosition = 0;

  //TODO move these
  public static final double MIN_ANGLE = Units.degreesToRadians(-260.0);
  public static final double MAX_ANGLE = Units.degreesToRadians(260.0);
  public static final double MIN_VOLTAGE = -10.0;
  public static final double MAX_VOLTAGE = 10.0;
  public static final double TURRET_STOP_TOLERANCE = Units.degreesToRadians(1);

  public Turret(TurretIO io) {
    this.io = io;
  } 

  public void disable() {
    controlMode = turretControlMode.Disabled;
  }

  public void periodic() {
    io.updateInputs(inputs);
    if(controlMode == turretControlMode.Disabled){
      io.setVoltage(0);
    }
    if(controlMode == turretControlMode.Voltage){
      debugVoltage = MathUtil.clamp(debugVoltage, MIN_VOLTAGE, MAX_ANGLE);
      io.setVoltage(debugVoltage);
    }
    if(controlMode == turretControlMode.Position){
      final double feedback = turretFeedBack.calculate(inputs.turretFusedAngle, goalPosition);
      final double feedforward = turretFeedForward.calculate(turretFeedBack.getSetpoint().velocity);

      double volts = feedback + feedforward;

      MathUtil.clamp(volts, MIN_VOLTAGE, MAX_VOLTAGE);
      io.setVoltage(volts);
    }

    absConnectedPub.set(inputs.absEncoderConnected);
    absAnglePub.set(inputs.turretAbsAngle);
    relAnglePub.set(inputs.turretRelAngle);
    fusedAnglePub.set(inputs.turretFusedAngle);
    controlModePub.set(controlMode.toString());
    atsetpointPub.set(atGoal());
    debugVoltagePub.set(debugVoltage);
    positionPub.set(getPosition());
    goalPosePub.set(goalPosition);
  }

  private double clampSetpoint(double setpoint) {
     return MathUtil.clamp(setpoint, MIN_ANGLE, MAX_ANGLE);
  }


  public void stopTurret() {
    io.setVoltage(0.0);
  }

  public double getPosition() {
    return inputs.turretFusedAngle;
  }

  public boolean atGoal(){
    return Math.abs(turretFeedBack.getGoal().position - inputs.turretFusedAngle) < TURRET_STOP_TOLERANCE;
  }

  /**
   * Optimizes a setpoint to minimize turret movement
   * @param positionRads The position to optimize
   * @return The optimized setpoint
   */
  public double optimizeSetpoint(double positionRads){
    positionRads = MathUtil.angleModulus(positionRads);
    if (controlMode != controlMode.Position) {
      turretFeedBack.reset(inputs.turretFusedAngle);
    }
    //Bounding logic starts here
      ArrayList<Double> dThetas = new ArrayList<>();
      //Calculates the adjustment for each of the three paths we can take
      double dTheta = positionRads - inputs.turretFusedAngle;
      double dTheta2 = dTheta + Units.degreesToRadians(360);
      double dTheta3 = dTheta + Units.degreesToRadians(-360);
      dThetas.add(dTheta);
      dThetas.add(dTheta2);
      dThetas.add(dTheta3);

      //Removes any paths that take us beyond our limits
      for(int i = 0; i < dThetas.size(); i++){
        double endPos = inputs.turretFusedAngle + dThetas.get(i);
        if(endPos > MAX_ANGLE || endPos < MIN_ANGLE){
          dThetas.remove(i);
          i--;
        }
      }

      //finds the shortest path
      double shortestPathLength = Double.MAX_VALUE;
      double shortestPath = 999;

      for(double i : dThetas){
        if(Math.abs(i) < shortestPathLength){
          shortestPath = i; 
          shortestPathLength = Math.abs(i);
        }
      }
      goalPosition = inputs.turretFusedAngle + shortestPath;
      controlMode = turretControlMode.Position;
      return goalPosition;
  }

  public void setPosition(double positionRads){
    turretFeedBack.setGoal(optimizeSetpoint(positionRads));
  }

  public void setVoltage(double volts){
    debugVoltage = volts;
    if(controlMode != turretControlMode.Voltage){
      controlMode = turretControlMode.Voltage;
    }
  }

  /**
   * Sets the position of the turret.
   * @param position The position to set the turret to 
   */
  public void setPosition(Rotation2d position){
    setPosition(position.getRadians());
  }


  /**
   * Sets the position of the turret
   * @param params The shooting params to get turret angle from
   * @param robotPose The position of the robot
   */
  public void setPosition(ShootingParams params, Pose2d robotPose){
    setFieldRelativePosition(params.turretAngle(), robotPose.getRotation());
  }

  /**
   * Sets the goal position and velocity of the turret
   * @param position The goal position
   * @param velocity The goal velocity
   * @param robotAngle The current angle of the robot
   * @implNote Field relative by default
   */
  public void setPositionAndVelocity(Rotation2d position, double velocity, Rotation2d robotAngle){
    turretFeedBack.setGoal(new TrapezoidProfile.State(optimizeSetpoint(position.getRadians()- MathUtil.angleModulus(robotAngle.getRadians())), velocity));
  }
  //TODO new code; test
  /**
   * Sets the goal position and velocity of the turret
   * @param params The params containing goal position and velocity
   * @param robotPose The current position of the robot
   */
  public void setPositionAndVelocity(ShootingParams params, Pose2d robotPose){
    setPositionAndVelocity(params.turretAngle(), params.turretVelocity(), robotPose.getRotation());
  }

  /**
   * Sets the position of the turret
   * @param params The shooting params to get the turret angle from
   * @param robotAngle The angle of the robot
   */
  public void setPosition(ShootingParams params, Rotation2d robotAngle){
    setFieldRelativePosition(params.turretAngle(), robotAngle);
  }

  /**
   * Sets the position of the turret.
   * @param position The field relative angle to set the turret to 
   * @param robotAngle The robot's angle
   */
  public void setFieldRelativePosition(Rotation2d position, Rotation2d robotAngle){
    setPosition(position.getRadians() - MathUtil.angleModulus(robotAngle.getRadians()));
  }

  /**
   * Sets the position of the turret
   * @param position The field relative angle to set the turret to 
   * @param robotPose The robot's position
   */
  public void setFieldRelativePosition(Rotation2d position, Pose2d robotPose){
    setPosition(position.getRadians() - MathUtil.angleModulus(robotPose.getRotation().getRadians()));
  }

  /**
   * Sets the position of the turret, minimizing the distance the turret is from 0 
   * @param position The position to set the turret to
   */
  public void setPositionMinDistFromZero(double position){
    goalPosition = position;
  }

  /**
   * Sets the position of the turret, minimizing the distance the turret is from 0 
   * @param position The position to set the turret to
   */
   public void setPositionMinDistFromZero(Rotation2d position){
    goalPosition = position.getRadians();
  }

  /**
   * Sets the position of the turret, minimizing the distance the turret is from 0 
   * @param position The position to set the turret to
   * @param robotAngle The angle of the robot
   */
  public void setPositionMinDistFromZeroFieldRel(Rotation2d position, Rotation2d robotAngle){
    goalPosition = MathUtil.angleModulus(position.getRadians()-robotAngle.getRadians());
  }

  /**
   * Sets the position of the turret, minimizing the distance the turret is from 0 
   * @param position The position to set the turret to
   * @param robotPose The position of the robot
   */
  public void setPositionMinDistFromZeroFieldRel(Rotation2d position, Pose2d robotPose){
    goalPosition = MathUtil.angleModulus(position.getRadians()-robotPose.getRotation().getRadians());
  }

}
