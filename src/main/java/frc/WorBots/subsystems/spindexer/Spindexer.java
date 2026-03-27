package frc.WorBots.subsystems.spindexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.spindexer.SpindexerIO.SpindexerIOInputs;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.debug.TunableDouble;
import frc.WorBots.util.debug.TunablePIDController;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;

public class Spindexer extends SubsystemBase{
  SpindexerIO io;
  SpindexerIOInputs inputs = new SpindexerIOInputs();
  private double spinGoalVelocity;
  private double kickerGoalVelocity;
  private double goalVoltage;
  private boolean override = false;

  private TunablePIDController spinPid = new TunablePIDController("Spindexer", "Spin PID", Constants.SpindexerConstants.SPINDEXER_KP, Constants.SpindexerConstants.SPINDEXER_KI, Constants.SpindexerConstants.SPINDEXER_KD);
  private TunablePIDController kickerPid = new TunablePIDController("Spindexer", "Kicker PID", Constants.SpindexerConstants.KICKER_KP, Constants.SpindexerConstants.KICKER_KI, Constants.SpindexerConstants.KICKER_KD);
  private SimpleMotorFeedforward kickerFeedforward = new SimpleMotorFeedforward(Constants.SpindexerConstants.KICKER_KS, Constants.SpindexerConstants.KICKER_KV);
  private SimpleMotorFeedforward spinFeedforward = new SimpleMotorFeedforward(Constants.SpindexerConstants.SPIN_KS, Constants.SpindexerConstants.SPIN_KV);

  private enum ControlMode{
    Disabled,
    Voltage,
    Velocity
  }

  private ControlMode controlMode = ControlMode.Disabled;

  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static final String TABLE_NAME = "Spindexer";
  private final NetworkTable spinTable = instance.getTable(TABLE_NAME);

  private final BooleanPublisher activePublisher = spinTable.getBooleanTopic("Active").publish();
  private final BooleanPublisher jammedPublisher = spinTable.getBooleanTopic("Jammed").publish();
  private final BooleanPublisher overridePublisher = spinTable.getBooleanTopic("Overrided").publish();
  private final BooleanPublisher connectionPublisher = spinTable.getBooleanTopic("Connected").publish();
  private final DoublePublisher velocityPublisher = spinTable.getDoubleTopic("Spindexer Rotational Velocity").publish();
  private final DoublePublisher kickerVelocityPublisher = spinTable.getDoubleTopic("Kicker Rotational Velocity").publish();
  private final DoublePublisher spinGoalVelocityPublisher = spinTable.getDoubleTopic("Spindexer Goal Rotational Velocity").publish();
  private final DoublePublisher kickerGoalVelocityPublisher = spinTable.getDoubleTopic("Kicker Goal Rotational Velocity").publish();
  private final DoublePublisher voltagePublisher = spinTable.getDoubleTopic("Voltage").publish();
  private final DoublePublisher currentPublisher = spinTable.getDoubleTopic("Current").publish();
  private final DoublePublisher temperaturePublisher = spinTable.getDoubleTopic("Temperature").publish();

  public Spindexer(SpindexerIO spindexerIo){
    io = spindexerIo;
    spinPid.pid.setTolerance(Constants.SpindexerConstants.SPINDEXER_VEL_TOLERANCE);
  }
    
  public void periodic(){
    spinPid.update();
    kickerPid.update();

    io.updateInputs(inputs);
    StatusPage.reportStatus(StatusPage.SPINDEXER_SUBSYSTEM, inputs.talon.isConnected && inputs.follower.isConnected);
    StatusPage.reportStatus(StatusPage.SPINDEXER_JAM, isJammed());
    if(DriverStation.isDisabled() || inputs.talon.temperatureCelsius > Constants.SpindexerConstants.SPINDEXER_MAX_TEMP){
      controlMode = ControlMode.Disabled;
    }
    //TODO add something to try to resolve jamming
    if(controlMode == ControlMode.Disabled){
      spinGoalVelocity = 0;
      kickerGoalVelocity = 0;
      goalVoltage = 0;
      io.stop();
    } else  if (controlMode == ControlMode.Voltage){
      io.setSpinVoltage(goalVoltage);
      io.setKickerVoltage(goalVoltage);
    } else {
      double kickerFeedback = MathUtil.clamp(kickerPid.pid.calculate(inputs.kickerVelocity, kickerGoalVelocity),0,12);
      double spinFeedback = spinPid.pid.calculate(inputs.spinVelocity, spinGoalVelocity);
      io.setKickerVoltage(kickerFeedback + kickerFeedforward.calculate(kickerGoalVelocity));
      io.setSpinVoltage(spinFeedback + spinFeedforward.calculate(spinGoalVelocity));
    }

    activePublisher.set(inputs.active);
    jammedPublisher.set(inputs.jammed);
    overridePublisher.set(override);
    velocityPublisher.set(inputs.spinVelocity);
    kickerVelocityPublisher.set(inputs.kickerVelocity);
    spinGoalVelocityPublisher.set(spinGoalVelocity);
    kickerGoalVelocityPublisher.set(kickerGoalVelocity);
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
    runSpindexerVoltage(0);
    spinGoalVelocity = 0;
    kickerGoalVelocity = 0;
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
    return goalVoltage;
  }

  public void setContolMode(ControlMode controlmode){
    this.controlMode = controlmode;
  }

  public void setVelocity(double spinVelocity, double kickerVelocity){
    controlMode = ControlMode.Velocity;
    spinGoalVelocity = spinVelocity;
    kickerGoalVelocity = kickerVelocity;
  }
}
