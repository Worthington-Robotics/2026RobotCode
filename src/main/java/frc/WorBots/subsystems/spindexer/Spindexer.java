package frc.WorBots.subsystems.spindexer;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.spindexer.SpindexerIO.SpindexerIOInputs;
import frc.WorBots.util.debug.StatusPage;

public class Spindexer extends SubsystemBase{
  SpindexerIO io;
  SpindexerIOInputs inputs = new SpindexerIOInputs();
  private double goalVelocity;
  private double goalVoltage;
  private boolean override = false;

  private enum ControlMode{
    Disabled,
    Voltage
  }

  private ControlMode controlMode = ControlMode.Disabled;

  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static final String TABLE_NAME = "Spindexer";
  private final NetworkTable spinTable = instance.getTable(TABLE_NAME);

  private final BooleanPublisher activePublisher = spinTable.getBooleanTopic("Active").publish();
  private final BooleanPublisher jammedPublisher = spinTable.getBooleanTopic("Jammed").publish();
  private final BooleanPublisher overridePublisher = spinTable.getBooleanTopic("Overrided").publish();
  private final BooleanPublisher connectionPublisher = spinTable.getBooleanTopic("Connected").publish();
  private final DoublePublisher velocityPublisher = spinTable.getDoubleTopic("Rotational Velocity").publish();
  private final DoublePublisher goalVelocityPublisher = spinTable.getDoubleTopic("Goal Rotational Velocity").publish();
  private final DoublePublisher voltagePublisher = spinTable.getDoubleTopic("Voltage").publish();
  private final DoublePublisher currentPublisher = spinTable.getDoubleTopic("Current").publish();
  private final DoublePublisher temperaturePublisher = spinTable.getDoubleTopic("Temperature").publish();

  public Spindexer(SpindexerIO spindexerIo){
    io = spindexerIo;
  }
    
  public void periodic(){
    io.updateInputs(inputs);
    StatusPage.reportStatus(StatusPage.SPINDEXER_SUBSYSTEM, inputs.talon.isConnected && inputs.follower.isConnected);
    StatusPage.reportStatus(StatusPage.SPINDEXER_JAM, inputs.jammed);
    if(DriverStation.isDisabled() || inputs.talon.temperatureCelsius > Constants.SpindexerConstants.SPINDEXER_MAX_TEMP){
      controlMode = ControlMode.Disabled;
    }
    //TODO add something to try to resolve jamming
    if(controlMode == ControlMode.Disabled){
      goalVelocity = 0;
      goalVoltage = 0;
      io.stop();
    } else {
      io.setVoltage(goalVoltage);
    }

    activePublisher.set(inputs.active);
    jammedPublisher.set(inputs.jammed);
    overridePublisher.set(override);
    velocityPublisher.set(inputs.talon.velocityRadsPerSec);
    goalVelocityPublisher.set(goalVelocity);
    voltagePublisher.set(inputs.talon.supplyVoltage);
    currentPublisher.set(inputs.talon.currentDrawAmps);
    temperaturePublisher.set(inputs.talon.temperatureCelsius);
    connectionPublisher.set(inputs.talon.isConnected);
  }

  public void runSpindexerVoltage(double voltage){
    controlMode = ControlMode.Voltage;
    this.goalVoltage = voltage;
  }

  public void disable(){
    controlMode = ControlMode.Disabled;
  }

  public void stopSpindexer(){
    io.stop();
    runSpindexerVoltage(0);;
  }

  public void toggleOverrideJam(){
    if(this.override){
      this.override = false;
    }
    else{
      this.override = true;
    }
  }

  public boolean isJammed(){
    return false;
  }

  public boolean isActive(){
    return inputs.active;
  }

  public boolean isConnected(){
    return inputs.talon.isConnected;
  }

  public double getVoltage(){
    return goalVoltage;
  }

  public void setContolMode(ControlMode controlmode){
    this.controlMode = controlmode;
  }
}
