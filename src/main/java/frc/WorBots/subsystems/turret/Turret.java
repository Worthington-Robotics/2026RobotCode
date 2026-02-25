package frc.WorBots.subsystems.turret;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret {
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
  private final BooleanPublisher shouldReadAbsEncoderPub =
     turret.getBooleanTopic("ShouldReadAbsEncoder").publish();
  private final StringPublisher controlModePub =
     turret.getStringTopic("ControlMode").publish();
  private final BooleanPublisher atsetpointPub =
     turret.getBooleanTopic("AtSetPoint").publish();
  private final DoublePublisher debugVoltagePub =
     turret.getDoubleTopic("DebugVoltage").publish();
  private final DoublePublisher positionPub =
     turret.getDoubleTopic("Position").publish();

  public final TurretIO io;

  private TurretIOInputs inputs = new TurretIOInputs();
  private turretControlMode controlMode = turretControlMode.Disabled;
  
  private final SimpleMotorFeedforward turretFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
  private  ProfiledPIDController turretFeedBack = new ProfiledPIDController(2, 0, 0,
    new TrapezoidProfile.Constraints(0.0, 0.0));

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
  public static final double MIN_VOLTAGE = 0.0;
  public static final double MAX_VOLTAGE = 10.0;

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

      //Bounding logic starts here

      ArrayList<Double> dThetas = new ArrayList<>();
      //Calculates the adjustment for each of the three paths we can take
      double dTheta = goalPosition - inputs.turretAbsAngle;
      double dTheta2 = dTheta + Units.degreesToRadians(360);
      double dTheta3 = dTheta + Units.degreesToRadians(-360);
      dThetas.add(dTheta);
      dThetas.add(dTheta2);
      dThetas.add(dTheta3);

      //Removes any paths that take us beyond our limits
      for(int i = 0; i < dThetas.size(); i++){
        double endPos = inputs.turretAbsAngle + dThetas.get(i);
        if(endPos > Units.degreesToRadians(MAX_ANGLE) || endPos < MIN_ANGLE){
          dThetas.remove(i);
          i--;
        }
      }

      //finds the shortest path
      double shortestPathLength = Double.MAX_VALUE;
      double shortestPath = 0;

      for(double i : dThetas){
        if(Math.abs(i) < shortestPathLength){
          shortestPath = i; 
          shortestPathLength = Math.abs(i);
        }
      }

      goalPosition = inputs.turretAbsAngle + shortestPath;
      
      //End of bounding logic

      final double feedback = turretFeedBack.calculate(inputs.turretAbsAngle, goalPosition);
      final double feedforward = turretFeedForward.calculate(turretFeedBack.getSetpoint().velocity);

      double volts = feedback + feedforward;

      MathUtil.clamp(volts, MIN_VOLTAGE, MAX_VOLTAGE);
      io.setVoltage(volts);
    }

    absConnectedPub.set(inputs.absEncoderConnected);
    absAnglePub.set(inputs.turretAbsAngle);
    relAnglePub.set(inputs.turretRelAngle);
    fusedAnglePub.set(inputs.turretFusedAngle);
    shouldReadAbsEncoderPub.set(inputs.shouldReadAbsEncoder);
    controlModePub.set(controlMode.toString());
    atsetpointPub.set(atGoal());
    debugVoltagePub.set(debugVoltage);
    positionPub.set(getPosition());
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
    return turretFeedBack.atGoal();
  }

  public void setPosition(double positionRads){
    positionRads = clampSetpoint(positionRads);
    if (controlMode != controlMode.Position) {
      turretFeedBack.reset(inputs.turretFusedAngle);
    }

    if (positionRads != goalPosition) {
      turretFeedBack.setGoal(positionRads);
    }
   
    goalPosition = positionRads;
    controlMode = turretControlMode.Position;
  }

  public void setVoltage(double volts){
    debugVoltage = volts;
    if(controlMode != turretControlMode.Voltage){
      controlMode = turretControlMode.Voltage;
    }
  }
  
}
