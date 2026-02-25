package frc.WorBots.subsystems.turret;

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
  private double setpointPosition = 0;


  public static final double MIN_ANGLE = Units.degreesToRadians(-270.0);
  public static final double MAX_ANGLE = Units.degreesToRadians(270.0);
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
      //TODO add logic to stop turret from going out of bounds while still taking the best path
      /*Probably will need a true heading (-270 to 270) and a relative heading
      relative heading can be used to calculate fastest path, while true heading can be used to check if a adjustment is needed
      This upgrade should also probably include softlimiting velocity as 270 is approached just in case, though limits should probably be closer to 260 to prevent damage*/
      
      final double feedback = turretFeedBack.calculate(inputs.turretFusedAngle, setpointPosition);
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
    controlModePub.set(inputs.controlMode.toString());
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

    if (positionRads != setpointPosition) {
      turretFeedBack.setGoal(positionRads);
    }
   
    setpointPosition = positionRads;
    controlMode = turretControlMode.Position;
  }

  public void setVoltage(double volts){
    debugVoltage = volts;
    if(controlMode != turretControlMode.Voltage){
      controlMode = turretControlMode.Voltage;
    }
  }
  
}
