package frc.WorBots.subsystems.climber;

import frc.WorBots.Constants;
import frc.WorBots.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputs inputs = new ClimberIOInputs();

  private double setPointVoltage = 0.0;
  private Rotation2d setpointPosition;

  ClimberControlMode controlMode = ClimberControlMode.Disabled;

  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static final String TABLE_NAME = "Climber";
  private final NetworkTable climbTable = instance.getTable(TABLE_NAME);

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
        System.out.println("Climber pose running");
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
    climberController.setConstraints(Constants.DRIVE_MAX_ROTATIONAL_VELOCITY, Constants.DRIVE_MAX_ACCELERATION);
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
}
