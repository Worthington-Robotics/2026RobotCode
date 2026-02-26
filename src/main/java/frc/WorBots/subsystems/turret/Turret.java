package frc.WorBots.subsystems.turret;

import edu.wpi.first.math.MathUtil;
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
  
  

  public enum turretControlMode {
    Disabled,
    Position,
    Voltage;
  }

  public Turret(TurretIO io) {
    this.io = io;
  } 

  public void disable() {
    controlMode = turretControlMode.Disabled;
  }

  public void periodic() {
    io.updateInputs(inputs);
    //TODO update voltage in periodic instead of updateInputs

    absConnectedPub.set(inputs.absEncoderConnected);
    absAnglePub.set(inputs.turretAbsAngle);
    relAnglePub.set(inputs.turretRelAngle);
    fusedAnglePub.set(inputs.turretFusedAngle);
    shouldReadAbsEncoderPub.set(inputs.shouldReadAbsEncoder);
    controlModePub.set(inputs.controlMode.toString());
    atsetpointPub.set(io.atSetPoint());
    debugVoltagePub.set(inputs.debugVoltage);
    positionPub.set(getPosition());
  }


  public void stopTurret() {
    io.setVoltage(0.0);
  }

  public double getPosition() {
    return inputs.turretFusedAngle;
  }

  public boolean atGoal(){
    return io.atSetPoint();
  }
  
}
