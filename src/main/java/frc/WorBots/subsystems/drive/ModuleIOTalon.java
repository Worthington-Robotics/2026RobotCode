package frc.WorBots.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.HardwareUtils.TalonSignals;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.util.OdometryThread;
import frc.WorBots.util.debug.TunablePIDController;
import frc.WorBots.util.debug.TunablePIDController.TunablePIDGains;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;
import frc.WorBots.util.debug.TunablePIDController.TunableTrapezoidConstraints;
import java.util.Queue;

public class ModuleIOTalon implements ModuleIO {
  private ModuleIOInputs inputs;

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.18868, 2.3);
  private static final TunablePIDGains driveFeedbackGains =
      new TunablePIDGains("Drive/Gains", "SModule Drive Feedback");
  private final TunablePIDController driveFeedback = new TunablePIDController(driveFeedbackGains);
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.00, 0.05);
  private static final TunablePIDGains turnFeedbackGains =
      new TunablePIDGains("Drive/Gains", "SModule Turn Feedback");
  private static final TunableTrapezoidConstraints turnFeedbackConstraints =
      new TunableTrapezoidConstraints("Drive/Gains", "SModule Turn Constraints");
  private final TunableProfiledPIDController turnFeedback =
      new TunableProfiledPIDController(turnFeedbackGains, turnFeedbackConstraints);

  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder absoluteEncoder;

  // private final Rotation2d encoderOffset;
  private final double wheelRadius;
  private final int id;

  private final TalonSignals driveSignals;
  private final TalonSignalsPositional turnSignals;
  private final StatusSignal<AngularVelocity> driveVelocitySignal;

  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIOTalon(int index) {
    driveFeedbackGains.setGains(0.3, 0.000, 0.0);
    turnFeedbackGains.setGains(5.0, 0.00, 0.0);
    turnFeedback.pid.enableContinuousInput(-Math.PI, Math.PI);
    turnFeedbackConstraints.setConstraints(360.0, 1500.0);

    inputs = new ModuleIOInputs(index);

    switch (index) {
      case 0: // Front Left
        driveMotor = new TalonFX(CanIDs.Swerve.FRONT_LEFT_DRIVE_ID, CanIDs.Swerve.CAN_BUS);
        HardwareUtils.setInverted(driveMotor, true);
        turnMotor = new TalonFX(CanIDs.Swerve.FRONT_LEFT_TURN_ID, CanIDs.Swerve.CAN_BUS);
        absoluteEncoder =
            new CANcoder(CanIDs.Swerve.FRONT_LEFT_ENCODER_ID, CanIDs.Swerve.CAN_BUS);
        // encoderOffset =
        //     new Rotation2d(
        //         1.7840 - 0.03 + Units.degreesToRadians(180.0) + Units.degreesToRadians(90.0));
        wheelRadius = Units.inchesToMeters(1.856);
        break;
      case 1: // Front Right
        driveMotor = new TalonFX(CanIDs.Swerve.FRONT_RIGHT_DRIVE_ID, CanIDs.Swerve.CAN_BUS);
        HardwareUtils.setInverted(driveMotor, false);
        turnMotor = new TalonFX(CanIDs.Swerve.FRONT_RIGHT_TURN_ID, CanIDs.Swerve.CAN_BUS);
        absoluteEncoder =
            new CANcoder(CanIDs.Swerve.FRONT_RIGHT_ENCODER_ID, CanIDs.Swerve.CAN_BUS);
        //encoderOffset = new Rotation2d(0.2883 + 0.007);
        wheelRadius = Units.inchesToMeters(1.873);
        break;
      case 2: // Back Left
        driveMotor = new TalonFX(CanIDs.Swerve.BACK_LEFT_DRIVE_ID, CanIDs.Swerve.CAN_BUS);
        HardwareUtils.setInverted(driveMotor, true);
        turnMotor = new TalonFX(CanIDs.Swerve.BACK_LEFT_TURN_ID, CanIDs.Swerve.CAN_BUS);
        absoluteEncoder =
            new CANcoder(CanIDs.Swerve.BACK_LEFT_ENCODER_ID, CanIDs.Swerve.CAN_BUS);
        //    new Rotation2d(-1.5942 + Units.degreesToRadians(180.0) + Units.degreesToRadians(90.0));
        wheelRadius = Units.inchesToMeters(1.867);
        break;
      case 3: // Back Right
        driveMotor = new TalonFX(CanIDs.Swerve.BACK_RIGHT_DRIVE_ID, CanIDs.Swerve.CAN_BUS);
        HardwareUtils.setInverted(driveMotor, true);
        turnMotor = new TalonFX(CanIDs.Swerve.BACK_RIGHT_TURN_ID, CanIDs.Swerve.CAN_BUS);
        absoluteEncoder =
            new CANcoder(CanIDs.Swerve.BACK_RIGHT_ENCODER_ID, CanIDs.Swerve.CAN_BUS);
        //encoderOffset = new Rotation2d(-1.1990 - 0.032 + Units.degreesToRadians(90.0));
        wheelRadius = Units.inchesToMeters(1.867);
        break;
      default:
        throw new RuntimeException("Invalid swerve module index");
    }

    id = index;

    // Configure devices
    HardwareUtils.setCurrentLimit(driveMotor, 46);
    HardwareUtils.setCurrentLimit(turnMotor, 40);

    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    turnMotor.setNeutralMode(NeutralModeValue.Brake);

    HardwareUtils.setInverted(turnMotor, false);

    driveMotor.setPosition(0.0);
    turnMotor.setPosition(0.0);

    // Signals
    driveSignals = new TalonSignals(driveMotor);
    turnSignals = new TalonSignalsPositional(turnMotor);
    driveVelocitySignal = driveMotor.getVelocity();
    driveVelocitySignal.setUpdateFrequency(Constants.ROBOT_PERIOD);

    // Odometry queues
    final var driveDistanceSignal = driveMotor.getPosition();
    final var turnAbsPosSignal = absoluteEncoder.getAbsolutePosition();

    StatusSignal.setUpdateFrequencyForAll(
        1.0 / OdometryThread.PERIOD, turnAbsPosSignal, driveDistanceSignal);
    drivePositionQueue = OdometryThread.getInstance().registerSignal(driveDistanceSignal);
    turnPositionQueue = OdometryThread.getInstance().registerSignal(turnAbsPosSignal);

    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();
    absoluteEncoder.optimizeBusUtilization();
  }

  public void updateInputs() {
    driveFeedback.update();
    turnFeedback.update();

    driveSignals.update(inputs.drive, driveMotor);
    turnSignals.update(inputs.turn, turnMotor);
    driveVelocitySignal.refresh();

    inputs.drive.positionRads /= Constants.DRIVE_GEAR_RATIO;
    inputs.drive.velocityRadsPerSec =
        driveVelocitySignal.getValue().in(edu.wpi.first.units.Units.RadiansPerSecond)
            / Constants.DRIVE_GEAR_RATIO;
    inputs.driveVelocityMetersPerSec =
        inputs.drive.velocityRadsPerSec * wheelRadius * Constants.DRIVE_MULTIPLIER;

    // Update odometry from signals into queues
    inputs.driveDistanceUpdates.clear();
    if (!drivePositionQueue.isEmpty()) {
      while (drivePositionQueue.size() > 0) {
        final double distance = drivePositionQueue.poll();
        inputs.driveDistanceUpdates.add(
            Units.rotationsToRadians(distance) / Constants.DRIVE_GEAR_RATIO * wheelRadius * Constants.DRIVE_MULTIPLIER);
      }
      final double lastUpdate =
          inputs.driveDistanceUpdates.get(inputs.driveDistanceUpdates.size() - 1);
      inputs.driveDistanceMeters = lastUpdate;
      inputs.drive.positionRads = lastUpdate / wheelRadius / Constants.DRIVE_MULTIPLIER;
    }
    inputs.turnPositionUpdates.clear();
    if (!turnPositionQueue.isEmpty()) {
      while (turnPositionQueue.size() > 0) {
        final double angle = turnPositionQueue.poll();
        inputs.turnPositionUpdates.add(
            MathUtil.angleModulus(Units.rotationsToRadians(angle)));
            // - encoderOffset.getRadians()
      }
      inputs.turnAbsolutePositionRad =
          inputs.turnPositionUpdates.get(inputs.turnPositionUpdates.size() - 1);
    }

    if (id == 0) {
      SmartDashboard.putNumberArray(
          "Drive Angles", inputs.turnPositionUpdates.toArray(new Double[0]));
    }

    inputs.turnAbsoluteVelocityRadsPerSec = inputs.turn.velocityRadsPerSec * Constants.TURN_GEAR_RATIO;

    inputs.turnPositionErrorRad = turnFeedback.pid.getPositionError();

    inputs.isConnected = inputs.turn.isConnected && inputs.drive.isConnected;
  }

  public ModuleIOInputs getInputs() {
    return this.inputs;
  }

  public void setDriveSpeed(double speedMetersPerSecond) {
    final double driveVolts =
        driveFeedforward.calculate(speedMetersPerSecond)
            + driveFeedback.pid.calculate(inputs.driveVelocityMetersPerSec, speedMetersPerSecond);
    setDriveVoltage(driveVolts);
  }

  public void setAngle(double angleRadians) {
    final double feedback =
        turnFeedback.pid.calculate(inputs.turnAbsolutePositionRad, angleRadians);
    final double feedforward = turnFeedforward.calculate(turnFeedback.pid.getSetpoint().velocity);
    setTurnVoltage(feedback + feedforward);
  }

  public void setDriveVoltage(double volts) {
    driveSignals.setVoltage(driveMotor, volts, 11.0);
  }

  public void setTurnVoltage(double volts) {
    turnSignals.setVoltage(turnMotor, volts, 11.0);
  }
}
