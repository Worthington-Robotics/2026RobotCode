package frc.WorBots.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret {

  public final TurretIO io;
  //TODO don't use turretIOTalon in turret.java, instead just use io which will be either a Talon or Sim depending on the state
  private TurretIOTalon pid;
  private double setpointPosition;
  private TurretIOInputs inputs = new TurretIOInputs();
  private turretControlMode controlMode = turretControlMode.Disabled;
  public static final double MIN_ANGLE = Units.degreesToRadians(-270.0);
  public static final double MAX_ANGLE = Units.degreesToRadians(270.0);
  public static final double MAX_VOLTAGE = 0.0; // TODO tune this value\

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
    //TODO update the turret's voltage commanded every period if in positional control mode
  }

  private double clampSetpoint(double setpoint) {
    return MathUtil.clamp(setpoint, MIN_ANGLE, MAX_ANGLE);
  }

  public void setPosition(double positionRads) {
    positionRads = clampSetpoint(positionRads);
    if (positionRads != setpointPosition) {
      //TODO command the pid in turretIOTalon
      pid.turretFeedBack.setGoal(positionRads);
    }
    setpointPosition = positionRads;
    controlMode = turretControlMode.Position;
  }

  public void stopTurret() {
    io.setVoltage(0.0);
  }

  public double getPosition() {
    return inputs.turretFusedAngle;
  }

  public boolean atSetpoint() {
    //TODO move all interactions with pid to turretIOTalon
    return pid.turretFeedBack.atGoal();
  }

}
