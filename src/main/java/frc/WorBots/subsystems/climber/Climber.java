package frc.WorBots.subsystems.climber;

import frc.WorBots.Constants;
import frc.WorBots.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputs inputs = new ClimberIOInputs();

  private double setPointVoltage = 0.0;
  private Rotation2d setpointPosition;

  ClimberControlMode controlMode = ClimberControlMode.Disabled;
  public enum ClimberControlMode {
    Voltage,
    Position,
    Disabled;
  }

  private final TunableProfiledPIDController climberController = new TunableProfiledPIDController("Climber",
      "Climber PID");

  private final SimpleMotorFeedforward climberFeedforward = new SimpleMotorFeedforward(0.5, 1);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    inputs.motor.publish();
    if (controlMode == ClimberControlMode.Disabled) {
      io.setMotorVolts(0);
    } else {
      if (controlMode == ClimberControlMode.Position) {
        climberController.pid.setGoal(setpointPosition.getRadians());
        final double feedback = climberController.pid.calculate(inputs.motor.positionRads);
        final double out = climberFeedforward.calculate(feedback);

        io.setMotorVolts(out);
      }
      if (controlMode == ClimberControlMode.Voltage) {
        io.setMotorVolts(setPointVoltage);
      }
    }
  }

  public Climber(ClimberIO io) {
    this.io = io;
    climberController.setGains(5, 0, 0);
    climberController.setConstraints(
      Constants.ClimberConstants.MAX_ROTATIONAL_VELOCITY, Constants.ClimberConstants.MAX_ROTATIONAL_ACCELERATION);
  }

  public void setVolts(double volts) {
    controlMode = ClimberControlMode.Voltage;
    setPointVoltage = volts;
  }

  public void setPosition(Rotation2d pos) {
    controlMode = ClimberControlMode.Position;
    this.setpointPosition = pos;
  }

  public void disable() {
    controlMode = ClimberControlMode.Disabled;
    io.setMotorVolts(0.0);
  }

  public Rotation2d getPosition() {
    return new Rotation2d(this.inputs.motor.positionRads);
  }

  public ClimberControlMode getControlMode() {
    return controlMode;
  }

  public boolean atGoal(){
    return climberController.pid.atGoal();
  }
}
