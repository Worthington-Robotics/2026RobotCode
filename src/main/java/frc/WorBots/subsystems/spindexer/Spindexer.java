package frc.WorBots.subsystems.spindexer;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.spindexer.SpindexerIO.SpindexerIOInputs;

public class Spindexer extends SubsystemBase{
  SpindexerIO io;
  SpindexerIOInputs inputs = new SpindexerIOInputs();
  private double goalVelocity;
  private boolean override = false;

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
    //TODO add something to try to resolve jamming
    if(DriverStation.isDisabled() || inputs.talon.temperatureCelsius > Constants.SPINDEXER_MAX_TEMP){
      goalVelocity = 0;
      io.stop();
    } else {
      io.setVelocity(goalVelocity);
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

  public void runSpindexer(){
    this.goalVelocity = 5;
  }

  public void stopSpindexer(){
    io.stop();
    goalVelocity = 0;
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
    return inputs.jammed;
  }

  public boolean isActive(){
    return inputs.active;
  }

  public boolean isConnected(){
    return inputs.talon.isConnected;
  }

  public double getVoltage(){
    return inputs.talon.supplyVoltage;
  }
}
